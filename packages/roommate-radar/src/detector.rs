//! roommate-radar's stereo YOLO detector thread. Reads the two SLAM camera frames published by camserver, rectifies
//! each into a virtual pinhole camera aligned with the head frame (so the pair is a rectified stereo pair, calib.rs),
//! runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across views and triangulates
//! depth from disparity (stereo.rs). Every second run it does the same for the two side cameras, without stereo (Side).
//! Results (head frame at capture time; the side cameras' in their views' frames, yawed by SIDE_YAW) go to DETS.
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::calib::{self, Rectifier, Valid};
use crate::measure::yawed;
use crate::ncnn;
use crate::shm::{self, now_ns};
use crate::stereo::{self, Det, Obj};
use crate::tracker::CLASSES;
use crate::v3::{V3, v3};

pub static QUIT: AtomicBool = AtomicBool::new(false);
pub static PAUSED: AtomicBool = AtomicBool::new(false); // skip frames (no GPU work) while set
pub static SIZE: AtomicI32 = AtomicI32::new(0); // per-view size to run at; a change reloads
pub static HZ: AtomicF32 = AtomicF32::new(0.0); // runs per second, changeable while running
pub static LOADING: AtomicI32 = AtomicI32::new(0); // size being loaded (model, then its first run compiles shaders); 0 = done
pub static PROBLEM: Mutex<String> = Mutex::new(String::new()); // why there are no detections, for the tab ("" = none)
pub static WAITING: AtomicBool = AtomicBool::new(false); // no pair of camera frames for over a second (headset idle)
pub static YOLO_MS: AtomicF32 = AtomicF32::new(0.0); // mean YOLO time over the last stats period
// The rectification's correction (rad, head axes: the left view's rotation relative to the right; selfcal.rs), half on
// each camera. A change rebuilds the rectifiers.
pub static CORRECTION: Mutex<[f32; 3]> = Mutex::new([0.0; 3]);
const ALIGN_EVERY: u64 = 8; // runs between stereo::alignment()s, which selfcal.rs estimates the correction from
pub static DETS: Mutex<DetectionSet> = Mutex::new(DetectionSet {
    id: 0,
    t_capture_ns: 0,
    objs: Vec::new(),
    views: [Vec::new(), Vec::new()],
    view: None,
    align: None,
    correction: [0.0; 3],
    side: None,
});

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

#[derive(Clone)]
pub struct DetectionSet {
    pub id: u64,
    pub t_capture_ns: u64,
    pub objs: Vec<Obj>,
    pub views: [Vec<Det>; 2],              // each view's boxes (px), for the trace
    pub view: Option<Arc<View>>,           // what the views see, for the tracker (measure::observable)
    pub align: Option<Arc<Vec<[f32; 5]>>>, // stereo::alignment()'s matches, every ALIGN_EVERY runs (no images leave here)
    pub correction: [f32; 3],              // CORRECTION as these were rectified with
    pub side: Option<Arc<Side>>,           // the side cameras' run, when there was one with this set
}

/// The side cameras' boxes, from every second run (half HZ). Each camera is rectified to a pinhole like the SLAM views
/// but looking straight left or right: the head frame turned by SIDE_YAW (measure::yawed), level, where a person beside
/// you is (the cameras themselves look ~25 deg up). Seen by one camera only, so objects on the 1 m plane, as
/// stereo::match_views makes one-view boxes; measure::measurements gives them a depth from the floor.
pub struct Side {
    pub objs: [Vec<Obj>; 2],    // left, right; in their view's frame
    pub t_capture_ns: [u64; 2], // 0: no frame from that camera yet
}

pub const SIDE_YAW: [f32; 2] = [std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2];

/// The rectified views' geometry.
pub struct View {
    pub s: usize,
    pub f: f32,            // px
    pub origins: [V3; 2],  // cameras in the head frame
    pub valid: [Valid; 2], // where each has image
}

pub const HFOV_DEG: f32 = 100.0; // the rectified views' field of view
const CONF: f32 = 0.1; // ByteTrack wants the weak detections too
const COCO: usize = 80; // the model's classes
const IOU: f32 = 0.5; // overlapping detections above this IoU keep only the most confident
const MIN_BOX: f32 = 0.05; // drop boxes whose larger side is under this fraction of the view (a cat at 3 m: ~6% long)
const LATENCY_MS: f32 = 8.0; // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion

/// Duplicate if they overlap a lot (IoU). Not when one is merely inside the other: someone farther away is, in the box
/// of someone in front of them (a cutout 3 m away hid, from one view, the person 2 m away it was inside of). One
/// object's nested boxes (upper body and whole) are one at a similar depth, and are merged after stereo, in 3D
/// (tracker.rs).
fn duplicate(a: &Det, b: &Det) -> bool {
    let ix = (a.x1.min(b.x1) - a.x0.max(b.x0)).max(0.0);
    let iy = (a.y1.min(b.y1) - a.y0.max(b.y0)).max(0.0);
    let inter = ix * iy;
    let (aa, ab) = ((a.x1 - a.x0) * (a.y1 - a.y0), (b.x1 - b.x0) * (b.y1 - b.y0));
    inter / (aa + ab - inter + 1e-6) > IOU
}

/// Overlapping boxes keep only the best, whatever their class (a person and a dog box on one body). Only boxes of our
/// classes get here (decode): suppressing across all of COCO's, a "couch" box around someone sitting on it hid them.
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

/// YOLO's output [4 + 80, N] (cx, cy, w, h in input pixels, then sigmoid scores for COCO's classes) as boxes in each
/// S x S view: those whose best class is one of tracker::CLASSES. A chair that scores 0.3 as a person and 0.8 as a
/// chair is a chair.
fn decode(out: &ncnn::Mat, s: usize) -> [Vec<Det>; 2] {
    let mut views = [Vec::new(), Vec::new()];
    let rows: Vec<&[f32]> = (0..4 + COCO).map(|r| out.row(r)).collect();
    let mut top = vec![(0, 0f32); out.w()]; // per anchor: (best class, its score), a row at a time
    for (cls, row) in rows[4..].iter().enumerate() {
        for (t, &v) in top.iter_mut().zip(*row) {
            if v > t.1 {
                *t = (cls, v);
            }
        }
    }
    let s = s as f32;
    for (i, &(best, bs)) in top.iter().enumerate() {
        if bs < CONF || !CLASSES.contains(&best) {
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
            // Missing or too small: no driver, or an older one still loaded after an update
            LOADING.store(0, Ordering::Relaxed);
            *PROBLEM.lock().unwrap() = "Restart SteamVR to load the updated driver".into();
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
    let model = format!("models/yolo11n-grayscale_{s}_rr.ncnn");
    let net = ncnn::Net::load(&model)?;

    let Some(hdr) = open_frames()? else {
        return Ok(());
    };
    LOADING.store(s as i32, Ordering::Relaxed);
    // After the frames: the calibration is the driver's copy of the factory one
    let cal = [calib::load(shm::CALIB_DIR, shm::CAM_NAMES[0])?, calib::load(shm::CALIB_DIR, shm::CAM_NAMES[1])?];
    let baseline = cal[1].t[0] - cal[0].t[0]; // right cam is at +x in the head frame
    let origins = cal.map(|c| v3(c.t[0], c.t[1], c.t[2]));
    // The rectifiers with the correction w, half of it on each camera, and the views they make
    let rectify = |w: [f32; 3]| {
        let (l, r) = (w.map(|v| v / 2.0), w.map(|v| -v / 2.0));
        let rect = [
            Rectifier::new(&calib::rotated(&cal[0], l), s, HFOV_DEG, shm::STRIDE),
            Rectifier::new(&calib::rotated(&cal[1], r), s, HFOV_DEG, shm::STRIDE),
        ];
        let view = Arc::new(View { s, f: rect[0].f, origins, valid: [rect[0].valid(), rect[1].valid()] });
        (rect, view)
    };
    // The side views: rectifier, camera position in its view's frame, where it has image. None without their calibration
    let side = match shm::SIDE_NAMES.map(|n| calib::load(shm::CALIB_DIR, n)) {
        [Ok(l), Ok(r)] => Some([(l, 0), (r, 1)].map(|(c, k)| {
            let rect = Rectifier::new(&calib::rotated(&c, [0.0, -SIDE_YAW[k], 0.0]), s, HFOV_DEG, shm::SIDE_W);
            let valid = rect.valid();
            (rect, yawed(v3(c.t[0], c.t[1], c.t[2]), -SIDE_YAW[k]), valid)
        })),
        [l, r] => {
            println!("detector: no side cameras: {:?} {:?}", l.err(), r.err());
            None
        }
    };
    let mut correction = *CORRECTION.lock().unwrap();
    let (mut rect, mut view) = rectify(correction);
    let f = rect[0].f;
    println!("detector: {s}x{s} views, f={f:.1} px, baseline {:.1} mm", baseline * 1000.0);
    // 105.6 mm on the Frame: anything else is swapped cameras or a changed calibration format
    if !(0.095..0.116).contains(&baseline) {
        return Err(format!("calibration: baseline {:.1} mm, not ~105.6", baseline * 1000.0));
    }

    // A torn read leaves garbage in its buffer, so frames are read into `scratch` and swapped in only when intact.
    let mut raw = [vec![0u8; shm::FRAME], vec![0u8; shm::FRAME]];
    let mut scratch = vec![0u8; shm::FRAME];
    let mut input = vec![0u8; s * s * 2];
    let (mut side_raw, mut side_scratch) =
        ([vec![0u8; shm::SIDE_FRAME], vec![0u8; shm::SIDE_FRAME]], vec![0u8; shm::SIDE_FRAME]);
    let (mut side_seq, mut side_t, mut side_last) = ([0u64; 2], [0u64; 2], [0u64; 2]);
    let (mut last_seq, mut t_ns, mut last_pair, mut last_pair_ns) = ([0u64; 2], [0u64; 2], 0u64, now_ns());
    let (mut frames, mut stat_t, mut last_run) = (0u64, now_ns(), 0u64);
    let mut runs = 0u64; // for ALIGN_EVERY
    let (mut t_rect, mut t_net, mut t_post, mut t_align, mut t_side) = (0f64, 0f64, 0f64, 0f64, 0f64);
    let mut why = stereo::AlignStats::default(); // summed over the log's period

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
            *PROBLEM.lock().unwrap() = shm::problem(&hdr);
            WAITING.store(now_ns() - last_pair_ns > 1_000_000_000, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        (last_pair, last_pair_ns) = (t_ns[0], now_ns());
        WAITING.store(false, Ordering::Relaxed);

        let a = now_ns();
        last_run = a;
        let want = *CORRECTION.lock().unwrap();
        if want != correction {
            correction = want;
            (rect, view) = rectify(correction);
            let deg = correction.map(f32::to_degrees);
            println!("detector: rectification corrected by {:.3} {:.3} {:.3} deg (x y z)", deg[0], deg[1], deg[2]);
        }
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
        if out.h() != 4 + COCO {
            return Err(format!("{model}: {} output rows, not 4 + {COCO}", out.h()));
        }
        let mut views = decode(&out, s);
        nms(&mut views[0]);
        nms(&mut views[1]);
        let (l_img, r_img) = input.split_at(s * s);
        let objs = stereo::match_views(&views, [l_img, r_img], s, f, baseline, origins, &view.valid);
        let e = now_ns();
        let stereo_n = objs.iter().filter(|o| o.stereo).count();
        runs += 1;
        let align = runs.is_multiple_of(ALIGN_EVERY).then(|| {
            let (a, st) = stereo::alignment(l_img, r_img, s); // a few ms
            why.tried += st.tried;
            why.flat += st.flat;
            why.weak += st.weak;
            why.edge_dy += st.edge_dy;
            why.edge_d += st.edge_d;
            why.unclear += st.unclear;
            why.found += st.found;
            Arc::new(a)
        });
        // The side cameras, every second run, if either has a new frame
        let g0 = now_ns();
        let mut side_set = None;
        if let Some(side) = &side
            && runs.is_multiple_of(2)
        {
            for k in 0..2 {
                if let Some(t) = shm::read(&hdr.side[k], &mut side_seq[k], &mut side_scratch) {
                    std::mem::swap(&mut side_raw[k], &mut side_scratch);
                    side_t[k] = t;
                }
            }
            if side_t != side_last {
                side_last = side_t;
                let (l_img, r_img) = input.split_at_mut(s * s);
                side[0].0.apply(&side_raw[0], l_img);
                side[1].0.apply(&side_raw[1], r_img);
                stretch(&mut input);
                if let Some(out) = net.run_gray(&input, s, 2 * s) {
                    let mut views = decode(&out, s);
                    // One camera each: an empty other view leaves every box one-view, on the 1 m plane
                    let objs = [0, 1].map(|k| {
                        nms(&mut views[k]);
                        let (o, v) = (side[k].1, &side[k].2);
                        let views = [std::mem::take(&mut views[k]), Vec::new()];
                        let img: &[u8] = &input;
                        stereo::match_views(&views, [img, img], s, f, baseline, [o, o], &[v.clone(), v.clone()])
                    });
                    let t_capture_ns = side_t.map(|t| if t == 0 { 0 } else { t - (LATENCY_MS * 1e6) as u64 });
                    side_set = Some(Arc::new(Side { objs, t_capture_ns }));
                }
            }
        }
        let g = now_ns();
        t_side += (g - g0) as f64 / 1e6;
        {
            let mut d = DETS.lock().unwrap();
            d.id += 1;
            d.t_capture_ns = t_ns[0] - (LATENCY_MS * 1e6) as u64;
            d.objs = objs;
            d.views = views.clone();
            d.view = Some(view.clone());
            d.align = align;
            d.correction = correction;
            d.side = side_set;
        }

        frames += 1;
        t_rect += (b - a) as f64 / 1e6;
        t_net += (c - b) as f64 / 1e6;
        t_post += (e - c) as f64 / 1e6;
        t_align = t_align.max((g0 - e) as f64 / 1e6);
        let t = now_ns();
        if t - stat_t > 5_000_000_000 {
            let n = frames as f64;
            println!(
                "detector: {:.1} fps | rectify {:.1} ms, yolo {:.1} ms, match {:.1} ms, align {:.0} ms, sides {:.1} ms | \
                 {}+{} dets, \
                 {stereo_n} objects | age {:.0} ms | align patches: {} tried, {} flat, {} weak, {} at dy's end, \
                 {} at d's end, {} unclear, {} found",
                n / ((t - stat_t) as f64 / 1e9),
                t_rect / n,
                t_net / n,
                t_post / n,
                t_align,
                t_side / n,
                views[0].len(),
                views[1].len(),
                (t - t_ns[0]) as f64 / 1e6,
                why.tried,
                why.flat,
                why.weak,
                why.edge_dy,
                why.edge_d,
                why.unclear,
                why.found
            );
            why = stereo::AlignStats::default();
            YOLO_MS.store((t_net / n) as f32);
            (frames, t_rect, t_net, t_post, t_align, t_side, stat_t) = (0, 0.0, 0.0, 0.0, 0.0, 0.0, t);
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

#[cfg(test)]
mod tests {
    use super::*;

    fn det(x0: f32, y0: f32, x1: f32, y1: f32, score: f32) -> Det {
        Det { x0, y0, x1, y1, score, cls: 0 }
    }

    #[test]
    fn nms_keeps_people_behind_people() {
        // A person 2 m away (tall box) with someone smaller, farther, inside it and more confident: both stay. Two
        // boxes on one person, mostly overlapping: one.
        let mut d = vec![det(100.0, 50.0, 160.0, 260.0, 0.45), det(110.0, 60.0, 135.0, 130.0, 0.6)];
        nms(&mut d);
        assert_eq!(d.len(), 2);
        let mut d = vec![det(100.0, 50.0, 160.0, 260.0, 0.45), det(102.0, 55.0, 158.0, 250.0, 0.6)];
        nms(&mut d);
        assert!(d.len() == 1 && d[0].score == 0.6);
    }
}
