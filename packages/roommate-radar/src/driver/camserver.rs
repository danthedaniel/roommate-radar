//! camserver: publish the newest normal-exposure SLAM and side camera frames to shared memory. Runs on a thread inside vrserver
//! (lib.rs): pidfd_getfd on XRService needs ptrace rights, which Yama (ptrace_scope=1) only grants to XRService's
//! ancestors (vrserver is its parent) or CAP_SYS_PTRACE.
//! Read-only with respect to XRService: VIDIOC_G_FMT and VIDIOC_QUERYBUF on its capture queues, mmap and
//! DMA_BUF_IOCTL_SYNC (read) on its dmabufs, nothing that changes their state.
use std::collections::HashMap;
use std::collections::hash_map::Entry;
use std::fs;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::shm;

static QUIT: AtomicBool = AtomicBool::new(false);

// linux/videodev2.h and linux/dma-buf.h on aarch64 (checked against the SDK's headers)
const VIDIOC_QUERYBUF: libc::c_ulong = 0xc058_5609;
const VIDIOC_G_FMT: libc::c_ulong = 0xc0d0_5604;
const DMA_BUF_IOCTL_SYNC: libc::c_ulong = 0x4008_6200;
const DMA_BUF_SYNC_START: u64 = 0;
const DMA_BUF_SYNC_END: u64 = 4;
const DMA_BUF_SYNC_READ: u64 = 1;
const V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE: u32 = 9;
const V4L2_BUF_FLAG_QUEUED: u32 = 2;
const VIDEO_MAX_PLANES: usize = 8;

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct V4l2Plane {
    bytesused: u32,
    length: u32,
    m: u64, // union { mem_offset, userptr, fd }
    data_offset: u32,
    reserved: [u32; 11],
}

#[repr(C)]
struct V4l2Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    timestamp: libc::timeval,
    timecode: [u32; 4],
    sequence: u32,
    memory: u32,
    m: u64, // union { offset, userptr, planes, fd }
    length: u32,
    reserved2: u32,
    request_fd: u32,
}
const _: () = assert!(size_of::<V4l2Plane>() == 64 && size_of::<V4l2Buffer>() == 88);

#[repr(C)]
#[derive(Clone, Copy)]
struct V4l2PlanePixFormat {
    sizeimage: u32,
    bytesperline: u32,
    reserved: [u16; 6],
}

#[repr(C)]
struct V4l2Format {
    type_: u32,
    _pad: u32, // the fmt union holds a pointer
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    colorspace: u32,
    plane_fmt: [V4l2PlanePixFormat; VIDEO_MAX_PLANES],
    rest: [u8; 20],
}
const _: () = assert!(size_of::<V4l2Format>() == 208);

/// Where XRService puts slam_left, slam_right, upper_left and upper_right, and how. It sends them through the ISP (VFE0/1: NV12, Y stride 1152;
/// bytesperline says 1056, so the stride is checked through the image size, Y + UV = 1152 x 1536), unless the color
/// passthrough camera add-on (arcimx616) is connected: that takes VFE0/1, and the SLAM cameras come raw from VFE4/3
/// (GREY, packed). XRService logs its choice ("TrackingCameraInit: index: 0. video device: ..."; index 0 = slam_right,
/// 1 = slam_left), and re-sets up its cameras when the add-on is plugged in or out.
/// (Without the add-on: video0 = og01a1b 4-0036 = slam_right, video3 = og01a1b 4-0060 = slam_left; verified by
/// rectifying both with the factory calibration: only this assignment gives an upright pair with purely horizontal
/// disparity.) The side cameras: upper_left is og0ve10 5-0060, upper_right 5-003e (XRService's log, "DeckardCaptureSource:
/// Found camera"); they're optional, the SLAM ones aren't.
struct Layout {
    nodes: [&'static str; 4], // slam left, slam right, side left, side right
    slam: Fmt,
    side: Fmt,
}

/// A camera's format as XRService sets it, and where its Y plane goes in shared memory.
struct Fmt {
    w: usize,
    h: usize,
    fourcc: [u8; 4],
    stride: usize,
    sizeimage: usize,
    out_stride: usize,
}

const SLAM_NV12: Fmt = Fmt {
    w: shm::W,
    h: shm::H,
    fourcc: *b"NV12",
    stride: shm::STRIDE,
    sizeimage: shm::FRAME * 3 / 2,
    out_stride: shm::STRIDE,
};
const LAYOUTS: [Layout; 2] = [
    // Stock Frame, no add-on: the SLAM cameras get the ISP (VFE0/1); video9/13 are the side cameras (NV12, stride 640).
    Layout {
        nodes: ["/dev/video3", "/dev/video0", "/dev/video13", "/dev/video9"],
        slam: SLAM_NV12,
        side: Fmt {
            w: shm::SIDE_W,
            h: shm::SIDE_H,
            fourcc: *b"NV12",
            stride: shm::SIDE_W,
            sizeimage: shm::SIDE_FRAME * 3 / 2,
            out_stride: shm::SIDE_W,
        },
    },
    // Arcturus Vision color passthrough add-on (arcimx616 x2) attached and owned by XRService or the Arcturus app:
    // video0/3 are the add-on's cameras (2464x2464), the SLAM cameras are raw on VFE4/3, the side ones on video6/7
    // (GREY, packed).
    Layout {
        nodes: ["/dev/video13", "/dev/video9", "/dev/video6", "/dev/video7"],
        slam: Fmt { fourcc: *b"GREY", stride: shm::W, sizeimage: shm::W * shm::H, ..SLAM_NV12 },
        side: Fmt {
            w: shm::SIDE_W,
            h: shm::SIDE_H,
            fourcc: *b"GREY",
            stride: shm::SIDE_W,
            sizeimage: shm::SIDE_FRAME,
            out_stride: shm::SIDE_W,
        },
    },
];

fn check_format(vfd: &OwnedFd, l: &Fmt) -> Result<(), String> {
    let mut f: V4l2Format = unsafe { std::mem::zeroed() };
    f.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if unsafe { libc::ioctl(vfd.as_raw_fd(), VIDIOC_G_FMT, &mut f) } < 0 {
        return Err(format!("VIDIOC_G_FMT: {}", std::io::Error::last_os_error()));
    }
    let size = f.plane_fmt[0].sizeimage;
    if (f.width, f.height, f.pixelformat, size)
        != (l.w as u32, l.h as u32, u32::from_le_bytes(l.fourcc), l.sizeimage as u32)
    {
        let fourcc = String::from_utf8_lossy(&f.pixelformat.to_le_bytes()).into_owned();
        return Err(format!(
            "{}x{} {fourcc} {size} B, not {}x{} {} {} B",
            f.width,
            f.height,
            l.w,
            l.h,
            String::from_utf8_lossy(&l.fourcc),
            l.sizeimage
        ));
    }
    Ok(())
}

fn find_xrservice() -> Option<i32> {
    // Match the executable: the main thread renames itself (comm is "XRServiceLoopTh").
    fs::read_dir("/proc").ok()?.flatten().find_map(|e| {
        let pid: i32 = e.file_name().to_str()?.parse().ok()?;
        let exe = fs::read_link(format!("/proc/{pid}/exe")).ok()?;
        (exe.file_name()? == "XRService").then_some(pid)
    })
}

fn find_fd(pid: i32, target: &str) -> Option<RawFd> {
    fs::read_dir(format!("/proc/{pid}/fd")).ok()?.flatten().find_map(|e| {
        (fs::read_link(e.path()).ok()? == std::path::Path::new(target)).then(|| e.file_name().to_str()?.parse().ok())?
    })
}

fn pidfd_getfd(pidfd: &OwnedFd, fd: RawFd) -> Option<OwnedFd> {
    let r = unsafe { libc::syscall(libc::SYS_pidfd_getfd, pidfd.as_raw_fd(), fd, 0) };
    (r >= 0).then(|| unsafe { OwnedFd::from_raw_fd(r as RawFd) })
}

/// Finds XRService's fd for camera c at `node`, checking its format.
fn find_cam(c: &mut Camera, pid: i32, pidfd: &OwnedFd, node: &'static str, fmt: &'static Fmt) -> Result<(), String> {
    c.node = node;
    c.fmt = fmt;
    c.remote_vfd = find_fd(pid, node).ok_or_else(|| format!("XRService has no {node} open"))?;
    let vfd = pidfd_getfd(pidfd, c.remote_vfd)
        .ok_or_else(|| format!("pidfd_getfd {node}: {}", std::io::Error::last_os_error()))?;
    check_format(&vfd, fmt).map_err(|e| format!("{node}: {e}"))
}

/// XRService's pid and pidfd, with each camera's video fd found and its format checked (a side camera that isn't is
/// left out, remote_vfd -1). Err: why not (short, it's the tab's status).
fn attach(cams: &mut [Camera]) -> Result<(i32, OwnedFd), String> {
    let err = || std::io::Error::last_os_error();
    let pid = find_xrservice().ok_or("XRService not running")?;
    let r = unsafe { libc::syscall(libc::SYS_pidfd_open, pid, 0) };
    if r < 0 {
        return Err(format!("pidfd_open XRService: {}", err()));
    }
    let pidfd = unsafe { OwnedFd::from_raw_fd(r as RawFd) };
    let mut why = Vec::new();
    for l in &LAYOUTS {
        match cams[..2].iter_mut().zip(l.nodes).try_for_each(|(c, node)| find_cam(c, pid, &pidfd, node, &l.slam)) {
            Ok(()) => {
                for (c, node) in cams[2..].iter_mut().zip(&l.nodes[2..]) {
                    if let Err(e) = find_cam(c, pid, &pidfd, node, &l.side) {
                        log!("no side camera: {e}\n");
                        c.remote_vfd = -1;
                    }
                }
                return Ok((pid, pidfd));
            }
            Err(e) => why.push(e),
        }
    }
    Err(why.join("; "))
}

/// SteamOS build and SteamVR version, for telling which update changed something.
fn versions() -> String {
    let os = fs::read_to_string("/etc/os-release").unwrap_or_default();
    let build = os.lines().find_map(|l| l.strip_prefix("BUILD_ID=")).unwrap_or("?").to_string();
    // vrserver is <steamvr>/bin/linuxarm64/vrserver
    let vr = fs::read_link("/proc/self/exe")
        .ok()
        .and_then(|e| fs::read_to_string(e.parent()?.parent()?.join("version.txt")).ok())
        .unwrap_or("?".into());
    format!("SteamOS {build}, SteamVR {}", vr.trim())
}

struct Mapping {
    fd: OwnedFd, // our dup of the dmabuf
    ino: u64,
    p: *const u8,
    len: usize,
}

impl Drop for Mapping {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.p as *mut libc::c_void, self.len) };
    }
}

struct Camera {
    node: &'static str,
    fmt: &'static Fmt,
    remote_vfd: RawFd,             // XRService's fd for /dev/videoN
    maps: HashMap<RawFd, Mapping>, // by XRService's dmabuf fd
    last_published: Option<u32>,   // V4L2 sequence number
    normal_mean: f64,              // running brightness of normal-exposure frames
}

impl Camera {
    fn close_all(&mut self) {
        self.maps.clear();
        self.remote_vfd = -1;
        self.last_published = None; // a restarted XRService counts from 0 again
        // and may bring a new exposure: the mean only follows frames it accepts
        self.normal_mean = 0.0;
    }
}

struct Buf {
    seq: u32,
    ts_ns: u64,
    remote_fd: RawFd,
}

// The video fd is dup'd only for the duration of the query: a dup we kept would keep XRService's open file (and its
// capture queue) alive after XRService closes it or exits, and the restarted XRService would get EBUSY on the camera.
// False when XRService is gone or the fd isn't the camera any more (the format catches a node XRService gave to
// another camera, as when the passthrough add-on is plugged in).
fn query(c: &Camera, pidfd: &OwnedFd, out: &mut Vec<Buf>) -> bool {
    out.clear();
    let Some(vfd) = pidfd_getfd(pidfd, c.remote_vfd) else {
        return false;
    };
    if check_format(&vfd, c.fmt).is_err() {
        return false;
    }
    for index in 0..64 {
        let mut planes = [V4l2Plane::default(); VIDEO_MAX_PLANES];
        let mut b: V4l2Buffer = unsafe { std::mem::zeroed() };
        b.index = index;
        b.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.length = VIDEO_MAX_PLANES as u32;
        b.m = planes.as_mut_ptr() as u64;
        if unsafe { libc::ioctl(vfd.as_raw_fd(), VIDIOC_QUERYBUF, &mut b) } < 0 {
            // EINVAL = past last index
            return std::io::Error::last_os_error().raw_os_error() == Some(libc::EINVAL);
        }
        if b.flags & V4L2_BUF_FLAG_QUEUED != 0 || b.timestamp.tv_sec == 0 {
            continue;
        }
        out.push(Buf {
            seq: b.sequence,
            ts_ns: b.timestamp.tv_sec as u64 * 1_000_000_000 + b.timestamp.tv_usec as u64 * 1000,
            remote_fd: planes[0].m as i32,
        });
    }
    true
}

// The cached mapping is only reused while XRService's fd still names the same dmabuf (each has its own inode): fd
// numbers come back for new buffers when XRService reallocates, and the old mapping would serve a frozen frame.
fn map_buf<'a>(c: &'a mut Camera, pidfd: &OwnedFd, remote_fd: RawFd) -> Option<&'a Mapping> {
    let fd = pidfd_getfd(pidfd, remote_fd)?;
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    // The fd number may have been reused for something smaller (a dmabuf's st_size is its size): reading past its end
    // would SIGBUS vrserver.
    let len = c.fmt.stride * c.fmt.h; // the Y plane
    if unsafe { libc::fstat(fd.as_raw_fd(), &mut st) } < 0 || (st.st_size as usize) < len {
        return None;
    }
    let ino = st.st_ino;
    if c.maps.get(&remote_fd).is_some_and(|m| m.ino != ino) {
        c.maps.remove(&remote_fd);
    }
    match c.maps.entry(remote_fd) {
        Entry::Occupied(e) => Some(e.into_mut()),
        Entry::Vacant(e) => {
            let p =
                unsafe { libc::mmap(std::ptr::null_mut(), len, libc::PROT_READ, libc::MAP_SHARED, fd.as_raw_fd(), 0) };
            (p != libc::MAP_FAILED).then(|| &*e.insert(Mapping { fd, ino, p: p as *const u8, len }))
        }
    }
}

fn sample_mean(y: &[u8], f: &Fmt) -> f64 {
    let (mut s, mut n) = (0u64, 0u64);
    for r in (32..f.h - 32).step_by(16) {
        for x in (32..f.w - 32).step_by(16) {
            s += y[r * f.out_stride + x] as u64;
            n += 1;
        }
    }
    s as f64 / n as f64
}

fn dma_buf_sync(fd: &OwnedFd, flags: u64) {
    unsafe { libc::ioctl(fd.as_raw_fd(), DMA_BUF_IOCTL_SYNC, &flags) };
}

pub fn stop() {
    QUIT.store(true, Ordering::Relaxed);
}

/// Runs until stop().
pub fn run() {
    let file = fs::OpenOptions::new().read(true).write(true).create(true).truncate(false).mode(0o644).open(shm::PATH);
    let file = match file.and_then(|f| {
        f.set_permissions(fs::Permissions::from_mode(0o644))?;
        f.set_len(size_of::<shm::Header>() as u64)?;
        Ok(f)
    }) {
        Ok(f) => f,
        Err(e) => return log!("{}: {e}\n", shm::PATH),
    };
    let p = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            size_of::<shm::Header>(),
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            file.as_raw_fd(),
            0,
        )
    };
    if p == libc::MAP_FAILED {
        return log!("mmap {}: {}\n", shm::PATH, std::io::Error::last_os_error());
    }
    let hdr = unsafe { &*(p as *const shm::Header) };

    let mut cams: [Camera; 4] = std::array::from_fn(|_| Camera {
        node: "",
        fmt: &SLAM_NV12,
        remote_vfd: -1,
        maps: HashMap::new(),
        last_published: None,
        normal_mean: 0.0,
    });
    // Camera clock - CLOCK_MONOTONIC (at arrival), one for all: they're hardware-synchronized on one clock, and the
    // app pairs their frames by time. Tracked per camera, each one's arrival jitter moved its own, until they were
    // > 2 ms apart and no frames paired.
    let mut clock_offset_ns: Option<i64> = None;
    let mut frame = vec![0u8; shm::FRAME];
    let mut bufs = Vec::new();
    let mut pidfd: Option<OwnedFd> = None;
    let mut why_last = String::new();
    let (mut published, mut last_report) = (0u64, shm::now_ns());

    while !QUIT.load(Ordering::Relaxed) {
        if pidfd.is_none() {
            for c in &mut cams {
                c.close_all();
            }
            clock_offset_ns = None; // a restarted XRService may bring a new camera clock: the offset only creeps down
            match attach(&mut cams) {
                Ok((pid, fd)) => {
                    log!(
                        "attached to XRService pid {pid} ({}=left, {}=right, sides {} {}; {})\n",
                        cams[0].node,
                        cams[1].node,
                        if cams[2].remote_vfd < 0 { "-" } else { cams[2].node },
                        if cams[3].remote_vfd < 0 { "-" } else { cams[3].node },
                        versions()
                    );
                    shm::set_problem(hdr, "");
                    why_last.clear();
                    pidfd = Some(fd);
                }
                Err(why) => {
                    if why != why_last {
                        log!("waiting for XRService: {why} ({})\n", versions());
                        shm::set_problem(hdr, &why);
                        why_last = why;
                    }
                    std::thread::sleep(Duration::from_secs(1));
                    continue;
                }
            }
        }
        let pfd = pidfd.as_ref().unwrap();
        let mut lost = false;
        for (ci, c) in cams.iter_mut().enumerate() {
            if c.remote_vfd < 0 {
                continue; // a side camera that isn't there
            }
            // XRService went away or reset its queue. Without a side camera the SLAM ones still work, until they're
            // lost too and everything is attached again.
            if !query(c, pfd, &mut bufs) {
                if ci >= 2 {
                    log!("{}: side camera lost\n", c.node);
                    c.close_all();
                    continue;
                }
                lost = true;
                break;
            }
            let arrival = shm::now_ns();
            bufs.sort_by(|a, b| b.seq.cmp(&a.seq));
            // XRService restarted streaming on the same fd: its sequence counts from 0 again
            if c.last_published.is_some_and(|l| bufs.first().is_some_and(|b| b.seq < l)) {
                c.last_published = None;
                clock_offset_ns = None;
            }
            if bufs.is_empty() || Some(bufs[0].seq) == c.last_published {
                continue;
            }
            // The camera clock runs ahead of CLOCK_MONOTONIC (~2 s on the Frame, in the status line below). Track
            // the offset as the max of (timestamp - arrival), over all cameras: the least-delayed observation bounds
            // it best.
            let off = bufs[0].ts_ns as i64 - arrival as i64;
            clock_offset_ns = Some(clock_offset_ns.map_or(off, |o| (o - 1000).max(off)));

            // Newest completed frame that is a normal exposure (not the ~37us controller-LED one).
            for b in &bufs {
                if c.last_published.is_some_and(|l| b.seq <= l) {
                    break;
                }
                let f = c.fmt;
                let Some(mb) = map_buf(c, pfd, b.remote_fd) else {
                    continue;
                };
                dma_buf_sync(&mb.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
                let src = unsafe { std::slice::from_raw_parts(mb.p, mb.len) };
                for (dst, row) in frame.chunks_mut(f.out_stride).zip(src.chunks(f.stride)).take(f.h) {
                    dst[..f.w].copy_from_slice(&row[..f.w]);
                }
                dma_buf_sync(&mb.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
                let m = sample_mean(&frame, f);
                // brightness heuristic; black level ~16 (~4 raw), controller frames sit right on it
                let normal = if c.normal_mean == 0.0 { m > 20.0 } else { m > 16.0 + 0.3 * (c.normal_mean - 16.0) };
                if !normal {
                    continue;
                }
                c.normal_mean = if c.normal_mean == 0.0 { m } else { 0.9 * c.normal_mean + 0.1 * m };
                let t = (b.ts_ns as i64 - clock_offset_ns.unwrap()) as u64;
                match ci {
                    0 | 1 => shm::write(&hdr.slots[ci], &frame, t),
                    _ => shm::write(&hdr.side[ci - 2], &frame, t),
                }
                c.last_published = Some(b.seq);
                published += 1;
                break;
            }
        }
        if lost {
            pidfd = None;
        }

        let t = shm::now_ns();
        if t - last_report > 5_000_000_000 {
            log!(
                "published {:.1} frames/s; clock offset {:.2} ms; mean {:.0} / {:.0}, sides {:.0} / {:.0}\n",
                published as f64 / ((t - last_report) as f64 / 1e9),
                clock_offset_ns.unwrap_or(0) as f64 / 1e6,
                cams[0].normal_mean,
                cams[1].normal_mean,
                cams[2].normal_mean,
                cams[3].normal_mean
            );
            published = 0;
            last_report = t;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    unsafe { libc::munmap(p, size_of::<shm::Header>()) };
}
