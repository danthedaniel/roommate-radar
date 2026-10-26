//! roommate-radar's stereo YOLO detector thread. Reads the two SLAM camera frames published by camserver, rectifies
//! each into a virtual pinhole camera aligned with the head frame (so the pair is a rectified stereo pair, calib.rs),
//! runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across views and triangulates
//! depth from disparity (stereo.rs). Results (head frame at capture time) go to DETS.
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::time::Duration;

use crate::calib::{self, Rectifier};
use crate::ncnn;
use crate::shm::{self, now_ns};
use crate::stereo::{self, Det, Obj};
use crate::v3::v3;

pub static QUIT: AtomicBool = AtomicBool::new(false);
pub static PAUSED: AtomicBool = AtomicBool::new(false); // skip frames (no GPU work) while set
pub static SIZE: AtomicI32 = AtomicI32::new(0); // per-view size to run at; a change reloads
pub static HZ: AtomicF32 = AtomicF32::new(0.0); // runs per second, changeable while running
pub static LOADING: AtomicI32 = AtomicI32::new(0); // size being loaded (model, then its first run compiles shaders); 0 = done
pub static YOLO_MS: AtomicF32 = AtomicF32::new(0.0); // mean YOLO time over the last stats period
pub static DETS: Mutex<DetectionSet> = Mutex::new(DetectionSet { id: 0, t_capture_ns: 0, objs: Vec::new() });

pub struct AtomicF32(AtomicU32);

impl AtomicF32 {
    pub const fn new(v: f32) -> AtomicF32 {
        AtomicF32(AtomicU32::new(v.to_bits()))
    }
    pub fn load(&self) -> f32 {
        f32::from_bits(self.0.load(Ordering::Relaxed))
    }
    pub fn store(&self, v: f32) {
        self.0.store(v.to_bits(), Ordering::Relaxed)
    }
}

#[derive(Clone, Default)]
pub struct DetectionSet {
    pub id: u64,
    pub t_capture_ns: u64,
    pub objs: Vec<Obj>,
}

pub const HFOV_DEG: f32 = 100.0; // the rectified views' field of view
const CONF: f32 = 0.1; // ByteTrack wants the weak detections too
const IOU: f32 = 0.5; // overlapping detections/tracks above this IoU keep only the most confident
const CONTAIN: f32 = 0.8; // ...same if this fraction of the smaller box lies inside the larger one
const MIN_BOX: f32 = 0.05; // drop boxes whose larger side is under this fraction of the view (a cat at 3 m: ~6% long)
const LATENCY_MS: f32 = 8.0; // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion

/// Duplicate if they overlap a lot (IoU) or one is mostly inside the other (intersection over the
/// smaller area): a half-size box nested in a bigger one is only ~25% IoU but clearly the same object.
pub fn duplicate(a: &Det, b: &Det) -> bool {
    let ix = (a.x1.min(b.x1) - a.x0.max(b.x0)).max(0.0);
    let iy = (a.y1.min(b.y1) - a.y0.max(b.y0)).max(0.0);
    let inter = ix * iy;
    let (aa, ab) = ((a.x1 - a.x0) * (a.y1 - a.y0), (b.x1 - b.x0) * (b.y1 - b.y0));
    inter / (aa + ab - inter + 1e-6) > IOU || inter > CONTAIN * aa.min(ab)
}

/// Class-agnostic: overlapping boxes of different classes (laptop/tv/keyboard on one object) keep only the best.
fn nms(d: &mut Vec<Det>) {
    d.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut keep: Vec<Det> = Vec::new();
    for x in d.drain(..) {
        if !keep.iter().any(|k| duplicate(k, &x)) {
            keep.push(x);
        }
    }
    *d = keep;
}

/// YOLO's output [4 + 80, N] (cx, cy, w, h in input pixels, then per-class sigmoid scores) as boxes in each S x S
/// view.
fn decode(out: &ncnn::Mat, s: usize) -> [Vec<Det>; 2] {
    let mut views = [Vec::new(), Vec::new()];
    let rows: Vec<&[f32]> = (0..4 + 80).map(|r| out.row(r)).collect();
    let s = s as f32;
    for i in 0..out.w() {
        let (mut best, mut bs) = (0, 0.0);
        for k in 0..80 {
            if rows[4 + k][i] > bs {
                (best, bs) = (k, rows[4 + k][i]);
            }
        }
        if bs < CONF {
            continue;
        }
        let (cx, cy, w, h) = (rows[0][i], rows[1][i], rows[2][i], rows[3][i]);
        let v = (cy >= s) as usize; // top half = left view, bottom half = right view
        let oy = v as f32 * s;
        let d = Det {
            x0: cx - w / 2.0,
            y0: (cy - h / 2.0).max(oy) - oy,
            x1: cx + w / 2.0,
            y1: (cy + h / 2.0).min(oy + s) - oy,
            score: bs,
            cls: best,
        };
        // > 4 px tall: not a sliver clipped at the seam between the views
        if d.y1 - d.y0 > 4.0 && (d.x1 - d.x0).max(d.y1 - d.y0) >= MIN_BOX * s {
            views[v].push(d);
        }
    }
    views
}

/// The frames driver_roommate_radar publishes (mapped read-only).
struct Frames(*const shm::Header);

impl std::ops::Deref for Frames {
    type Target = shm::Header;
    fn deref(&self) -> &shm::Header {
        unsafe { &*self.0 }
    }
}

impl Drop for Frames {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.0 as *mut libc::c_void, size_of::<shm::Header>()) };
    }
}

/// The frames, once driver_roommate_radar publishes them; None on quit.
fn open_frames() -> Result<Option<Frames>, String> {
    // driver_roommate_radar creates the file before sizing it; touching a mapping past its end is SIGBUS.
    let mut logged = false;
    let file = loop {
        if QUIT.load(Ordering::Relaxed) {
            return Ok(None);
        }
        if let Ok(f) = std::fs::File::open(shm::PATH)
            && f.metadata().is_ok_and(|m| m.len() >= size_of::<shm::Header>() as u64)
        {
            break f;
        }
        if !logged {
            eprintln!("waiting for driver_roommate_radar ({}; restart SteamVR after the first run)", shm::PATH);
            logged = true;
        }
        std::thread::sleep(Duration::from_secs(1));
    };
    use std::os::fd::AsRawFd;
    let size = size_of::<shm::Header>();
    let p = unsafe { libc::mmap(std::ptr::null_mut(), size, libc::PROT_READ, libc::MAP_SHARED, file.as_raw_fd(), 0) };
    if p == libc::MAP_FAILED {
        return Err(format!("mmap {}: {}", shm::PATH, std::io::Error::last_os_error()));
    }
    Ok(Some(Frames(p as *const shm::Header)))
}

/// The SLAM frames are dim and low-contrast; stretch the 1st..99th percentile to full range.
/// Pixels outside the fisheye are 0 -> YOLO's letterbox gray (114), so they don't look like edges.
fn stretch(input: &mut [u8]) {
    let mut hist = [0u32; 256];
    for &p in input.iter().step_by(7) {
        hist[p as usize] += 1;
    }
    let total: u32 = hist[1..].iter().sum();
    let (mut acc, mut lo, mut hi) = (0, 1, 255);
    for (v, &n) in hist.iter().enumerate().skip(1) {
        acc += n;
        if acc < total / 100 {
            lo = v as i32;
        }
        if acc <= total * 99 / 100 {
            hi = v as i32;
        }
    }
    let mut lut = [114u8; 256];
    for (v, l) in lut.iter_mut().enumerate().skip(1) {
        *l = ((v as i32 - lo) * 255 / (hi - lo).max(1)).clamp(0, 255) as u8;
    }
    for p in input {
        *p = lut[*p as usize];
    }
}

/// One run at size S, until SIZE changes or quit.
fn run(s: usize) -> Result<(), String> {
    LOADING.store(s as i32, Ordering::Relaxed);
    let t_load = now_ns();
    let model = format!("models/yolo11n-grayscale_{s}_gray.ncnn");
    let net = ncnn::Net::load(&model)?;

    let Some(hdr) = open_frames()? else {
        return Ok(());
    };
    // After the frames: the calibration is the driver's copy of the factory one
    let cal = [calib::load(shm::CALIB_DIR, shm::CAM_NAMES[0])?, calib::load(shm::CALIB_DIR, shm::CAM_NAMES[1])?];
    let rect = [Rectifier::new(&cal[0], s, HFOV_DEG), Rectifier::new(&cal[1], s, HFOV_DEG)];
    let f = rect[0].f;
    let baseline = cal[1].t[0] - cal[0].t[0]; // right cam is at +x in the head frame
    println!("detector: {s}x{s} views, f={f:.1} px, baseline {:.1} mm", baseline * 1000.0);

    // A torn read leaves garbage in its buffer, so frames are read into `scratch` and swapped in only when intact.
    let mut raw = [vec![0u8; shm::FRAME], vec![0u8; shm::FRAME]];
    let mut scratch = vec![0u8; shm::FRAME];
    let mut input = vec![0u8; s * s * 2];
    let (mut last_seq, mut t_ns, mut last_pair) = ([0u64; 2], [0u64; 2], 0u64);
    let (mut frames, mut stat_t, mut last_run) = (0u64, now_ns(), 0u64);
    let (mut t_rect, mut t_net, mut t_post) = (0f64, 0f64, 0f64);

    while !QUIT.load(Ordering::Relaxed) && SIZE.load(Ordering::Relaxed) == s as i32 {
        if PAUSED.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }
        let (t, due) = (now_ns(), last_run + (1e9 / HZ.load()) as u64);
        if t < due {
            std::thread::sleep(Duration::from_nanos((due - t).min(100_000_000)));
            continue;
        }
        for i in 0..2 {
            if let Some(t) = shm::read(&hdr.slots[i], &mut last_seq[i], &mut scratch) {
                std::mem::swap(&mut raw[i], &mut scratch);
                t_ns[i] = t;
            }
        }
        // Both cameras are hardware-synchronized: wait for a pair with the same capture time.
        if t_ns[0] == 0 || t_ns[0] == last_pair || (t_ns[0] as i64 - t_ns[1] as i64).abs() > 2_000_000 {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        last_pair = t_ns[0];

        let a = now_ns();
        last_run = a;
        let (l_img, r_img) = input.split_at_mut(s * s);
        rect[0].apply(&raw[0], l_img);
        rect[1].apply(&raw[1], r_img);
        stretch(&mut input);
        let b = now_ns();

        // prep_model.py folded GRAY2RGB and the 1/255 scale into the first conv.
        let out = net.run_gray(&input, s, 2 * s);
        if LOADING.load(Ordering::Relaxed) != 0 {
            println!("detector: {s}px ready {:.1} s after loading started", (now_ns() - t_load) as f64 / 1e9);
            LOADING.store(0, Ordering::Relaxed);
        }
        let c = now_ns();
        let Some(out) = out else {
            continue;
        };
        if out.h() < 4 + 80 {
            return Err(format!("{model}: {} output rows, not YOLO's 4 + 80", out.h()));
        }
        let mut views = decode(&out, s);
        nms(&mut views[0]);
        nms(&mut views[1]);
        let origin = v3(cal[0].t[0], cal[0].t[1], cal[0].t[2]);
        let objs = stereo::match_views(&views, &input[..s * s], &input[s * s..], s, f, baseline, origin);
        let e = now_ns();
        let stereo_n = objs.iter().filter(|o| o.stereo).count();
        {
            let mut d = DETS.lock().unwrap();
            d.id += 1;
            d.t_capture_ns = t_ns[0] - (LATENCY_MS * 1e6) as u64;
            d.objs = objs;
        }

        frames += 1;
        t_rect += (b - a) as f64 / 1e6;
        t_net += (c - b) as f64 / 1e6;
        t_post += (e - c) as f64 / 1e6;
        let t = now_ns();
        if t - stat_t > 5_000_000_000 {
            let n = frames as f64;
            println!(
                "detector: {:.1} fps | rectify {:.1} ms, yolo {:.1} ms, match {:.1} ms | {}+{} dets, {stereo_n} objects | \
                 age {:.0} ms",
                n / ((t - stat_t) as f64 / 1e9),
                t_rect / n,
                t_net / n,
                t_post / n,
                views[0].len(),
                views[1].len(),
                (t - t_ns[0]) as f64 / 1e6
            );
            YOLO_MS.store((t_net / n) as f32);
            (frames, t_rect, t_net, t_post, stat_t) = (0, 0.0, 0.0, 0.0, t);
        }
    }
    Ok(())
}

/// The detector thread: runs at SIZE, reloading when it changes, until quit.
pub fn run_loop() -> Result<(), String> {
    while !QUIT.load(Ordering::Relaxed) {
        run(SIZE.load(Ordering::Relaxed) as usize)?;
    }
    Ok(())
}
