//! roommate-radar's stereo YOLO detector thread. Reads the two SLAM camera frames published by camserver, rectifies
//! each into a virtual pinhole camera aligned with the head frame (so the pair is a rectified stereo pair, rectify.rs),
//! runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across views and triangulates
//! depth from disparity (stereo.rs). Results (head frame at capture time) go to DETECTIONS.
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::ncnn;
use crate::rectify::{self, Rectifier, ValidRegion};
use crate::shm::{self, now_ns};
use crate::stereo::{self, Detection, Obj};
use crate::v3::{V3, v3};

pub static QUIT: AtomicBool = AtomicBool::new(false);
pub static PAUSED: AtomicBool = AtomicBool::new(false); // skip frames (no GPU work) while set
pub static SIZE: AtomicI32 = AtomicI32::new(0); // per-view size to run at; a change reloads
pub static HZ: AtomicF32 = AtomicF32::new(0.0); // runs per second, changeable while running
pub static LOADING_SIZE: AtomicI32 = AtomicI32::new(0); // size being loaded (model, then its first run compiles shaders); 0 = done
pub static PROBLEM: Mutex<String> = Mutex::new(String::new()); // why there are no detections, for the tab ("" = none)
pub static WAITING: AtomicBool = AtomicBool::new(false); // no pair of camera frames for over a second (headset idle)
pub static YOLO_MS: AtomicF32 = AtomicF32::new(0.0); // mean YOLO time over the last stats period
// The rectification's correction (rad, head axes: the left view's rotation relative to the right; selfcal.rs), half on
// each camera. A change rebuilds the rectifiers.
pub static CORRECTION: Mutex<[f32; 3]> = Mutex::new([0.0; 3]);
const ALIGN_EVERY: u64 = 8; // runs between stereo::alignment()s, which selfcal.rs estimates the correction from
pub static DETECTIONS: Mutex<DetectionSet> = Mutex::new(DetectionSet {
    id: 0,
    t_capture_ns: 0,
    objs: Vec::new(),
    boxes: [Vec::new(), Vec::new()],
    view: None,
    align_matches: None,
    correction: [0.0; 3],
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
    pub boxes: [Vec<Detection>; 2],                // each view's boxes (px), for the trace
    pub view: Option<Arc<View>>,                   // what the views see, for the tracker (measure::observable)
    pub align_matches: Option<Arc<Vec<[f32; 5]>>>, // stereo::alignment()'s matches, every ALIGN_EVERY runs (no images leave here)
    pub correction: [f32; 3],                      // CORRECTION as these were rectified with
}

/// The rectified views' geometry.
pub struct View {
    pub size_px: usize,
    pub focal_px: f32,
    pub origins: [V3; 2],        // cameras in the head frame
    pub valid: [ValidRegion; 2], // where each has image
}

pub const HFOV_DEG: f32 = 100.0; // the rectified views' field of view
pub const CLASSES: [usize; 3] = [0, 15, 16]; // person, cat, dog: COCO's ids for what we track
const MIN_SCORE: f32 = 0.1; // ByteTrack wants the weak detections too
const COCO_CLASSES: usize = 80; // the model's classes
const NMS_IOU: f32 = 0.5; // overlapping detections above this IoU keep only the most confident
const MIN_BOX_FRAC: f32 = 0.05; // drop boxes whose larger side is under this fraction of the view (a cat at 3 m: ~6% long)
const LATENCY_MS: f32 = 8.0; // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion

/// Duplicate if they overlap a lot (IoU). Not when one is merely inside the other: someone farther away is, in the box
/// of someone in front of them (a cutout 3 m away hid, from one view, the person 2 m away it was inside of). One
/// object's nested boxes (upper body and whole) are one at a similar depth, and are merged after stereo, in 3D
/// (tracker.rs).
fn duplicate(a: &Detection, b: &Detection) -> bool {
    let ix = (a.x1.min(b.x1) - a.x0.max(b.x0)).max(0.0);
    let iy = (a.y1.min(b.y1) - a.y0.max(b.y0)).max(0.0);
    let inter = ix * iy;
    let (aa, ab) = ((a.x1 - a.x0) * (a.y1 - a.y0), (b.x1 - b.x0) * (b.y1 - b.y0));
    inter / (aa + ab - inter + 1e-6) > NMS_IOU
}

/// Overlapping boxes keep only the best, whatever their class (a person and a dog box on one body). Only boxes of our
/// classes get here (decode): suppressing across all of COCO's, a "couch" box around someone sitting on it hid them.
fn nms(boxes: &mut Vec<Detection>) {
    boxes.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut keep: Vec<Detection> = Vec::new();
    for x in boxes.drain(..) {
        if !keep.iter().any(|k| duplicate(k, &x)) {
            keep.push(x);
        }
    }
    *boxes = keep;
}

/// YOLO's output [4 + 80, N] (cx, cy, w, h in input pixels, then sigmoid scores for COCO's classes) as boxes in each
/// S x S view: those whose best class is one of CLASSES. A chair that scores 0.3 as a person and 0.8 as a
/// chair is a chair.
fn decode(out: &ncnn::Mat, s: usize) -> [Vec<Detection>; 2] {
    let mut boxes = [Vec::new(), Vec::new()];
    let rows: Vec<&[f32]> = (0..4 + COCO_CLASSES).map(|r| out.row(r)).collect();
    let mut top = vec![(0, 0f32); out.w()]; // per anchor: (best class, its score), a row at a time
    for (cls, row) in rows[4..].iter().enumerate() {
        for (t, &v) in top.iter_mut().zip(*row) {
            if v > t.1 {
                *t = (cls, v);
            }
        }
    }
    let s = s as f32;
    for (i, &(best, best_score)) in top.iter().enumerate() {
        if best_score < MIN_SCORE || !CLASSES.contains(&best) {
            continue;
        }
        let (cx, cy, w, h) = (rows[0][i], rows[1][i], rows[2][i], rows[3][i]);
        let view_idx = (cy >= s) as usize; // top half = left view, bottom half = right view
        let oy = view_idx as f32 * s;
        let d = Detection {
            x0: cx - w / 2.0,
            y0: (cy - h / 2.0).max(oy) - oy,
            x1: cx + w / 2.0,
            y1: (cy + h / 2.0).min(oy + s) - oy,
            score: best_score,
            cls: best,
        };
        // > 4 px tall: not a sliver clipped at the seam between the views
        if d.y1 - d.y0 > 4.0 && (d.x1 - d.x0).max(d.y1 - d.y0) >= MIN_BOX_FRAC * s {
            boxes[view_idx].push(d);
        }
    }
    boxes
}

/// The frames driver_roommate_radar publishes (mapped read-only).
struct Frames(*const shm::Region);

impl std::ops::Deref for Frames {
    type Target = shm::Region;
    fn deref(&self) -> &shm::Region {
        unsafe { &*self.0 }
    }
}

impl Drop for Frames {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.0 as *mut libc::c_void, size_of::<shm::Region>()) };
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
        if let Ok(f) = std::fs::File::open(shm::CAMS_PATH)
            && f.metadata().is_ok_and(|m| m.len() >= size_of::<shm::Region>() as u64)
        {
            break f;
        }
        if !logged {
            eprintln!("waiting for driver_roommate_radar ({}; restart SteamVR after the first run)", shm::CAMS_PATH);
            // Missing or too small: no driver, or an older one still loaded after an update
            LOADING_SIZE.store(0, Ordering::Relaxed);
            *PROBLEM.lock().unwrap() = "Restart SteamVR to load the updated driver".into();
            logged = true;
        }
        std::thread::sleep(Duration::from_secs(1));
    };
    use std::os::fd::AsRawFd;
    let size = size_of::<shm::Region>();
    let p = unsafe { libc::mmap(std::ptr::null_mut(), size, libc::PROT_READ, libc::MAP_SHARED, file.as_raw_fd(), 0) };
    if p == libc::MAP_FAILED {
        return Err(format!("mmap {}: {}", shm::CAMS_PATH, std::io::Error::last_os_error()));
    }
    Ok(Some(Frames(p as *const shm::Region)))
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

/// Camera `name`'s calibration, if it fits its w x h frames: Rectifier::apply reads within its size.
fn load_cal(name: &str, w: usize, h: usize) -> Result<rectify::CamCal, String> {
    let c = rectify::load(shm::CALIB_DIR, name)?;
    if !(2..=w).contains(&c.w) || !(2..=h).contains(&c.h) {
        return Err(format!("calibration: {name} is {}x{}, its frames {w}x{h}", c.w, c.h));
    }
    Ok(c)
}

/// One run at size S, until SIZE changes or quit.
fn run_at_size(s: usize) -> Result<(), String> {
    LOADING_SIZE.store(s as i32, Ordering::Relaxed);
    let t_load = now_ns();
    let model_path = format!("models/yolo11n-grayscale_{s}_mixed.ncnn");
    let net = ncnn::Net::load(&model_path)?;

    let Some(frames) = open_frames()? else {
        return Ok(());
    };
    LOADING_SIZE.store(s as i32, Ordering::Relaxed);
    // After the frames: the calibration is the driver's copy of the factory one
    let cal = [load_cal(shm::CAM_NAMES[0], shm::W, shm::H)?, load_cal(shm::CAM_NAMES[1], shm::W, shm::H)?];
    let baseline = cal[1].t[0] - cal[0].t[0]; // right cam is at +x in the head frame
    let origins = cal.map(|c| v3(c.t[0], c.t[1], c.t[2]));
    // The rectifiers with `correction`, half of it on each camera, and the views they make
    let rectify = |correction: [f32; 3]| {
        let (l, r) = (correction.map(|v| v / 2.0), correction.map(|v| -v / 2.0));
        let rectifiers = [
            Rectifier::new(&rectify::rotated(&cal[0], l), s, HFOV_DEG),
            Rectifier::new(&rectify::rotated(&cal[1], r), s, HFOV_DEG),
        ];
        let valid = [rectifiers[0].valid(), rectifiers[1].valid()];
        let view = Arc::new(View { size_px: s, focal_px: rectifiers[0].focal_px, origins, valid });
        (rectifiers, view)
    };
    let mut correction = *CORRECTION.lock().unwrap();
    let (mut rectifiers, mut view) = rectify(correction);
    let focal_px = rectifiers[0].focal_px;
    println!("detector: {s}x{s} views, f={focal_px:.1} px, baseline {:.1} mm", baseline * 1000.0);
    // 105.6 mm on the Frame: anything else is swapped cameras or a changed calibration format
    if !(0.095..0.116).contains(&baseline) {
        return Err(format!("calibration: baseline {:.1} mm, not ~105.6", baseline * 1000.0));
    }

    // A torn read leaves garbage in its buffer, so frames are read into `scratch` and swapped in only when intact.
    let mut raw = [vec![0u8; shm::Y_BYTES], vec![0u8; shm::Y_BYTES]];
    let mut scratch = vec![0u8; shm::Y_BYTES];
    let mut input = vec![0u8; s * s * 2];
    let mut last_seq = [0u64; 2];
    let mut capture_ns = [0u64; 2]; // each camera's newest frame (CLOCK_MONOTONIC)
    // The last pair run on: its capture time, and when it arrived here (now_ns)
    let (mut last_pair_capture_ns, mut last_pair_seen_ns) = (0u64, now_ns());
    let (mut period_runs, mut period_start_ns, mut last_run_ns) = (0u64, now_ns(), 0u64);
    let mut runs = 0u64; // for ALIGN_EVERY
    // Over the log's period: summed per stage, align's the longest
    let (mut rectify_ms, mut yolo_ms, mut match_ms, mut align_ms_max) = (0f64, 0f64, 0f64, 0f64);
    let mut align_stats_sum = stereo::AlignStats::default();

    while !QUIT.load(Ordering::Relaxed) && SIZE.load(Ordering::Relaxed) == s as i32 {
        if PAUSED.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }
        let (now, due) = (now_ns(), last_run_ns + (1e9 / HZ.load()) as u64);
        if now < due {
            std::thread::sleep(Duration::from_nanos((due - now).min(100_000_000)));
            continue;
        }
        for i in 0..2 {
            if let Some(t_mono_ns) = shm::read(&frames.slots[i], &mut last_seq[i], &mut scratch) {
                std::mem::swap(&mut raw[i], &mut scratch);
                capture_ns[i] = t_mono_ns;
            }
        }
        // Both cameras are hardware-synchronized: wait for a pair with the same capture time.
        if capture_ns[0] == 0
            || capture_ns[0] == last_pair_capture_ns
            || capture_ns[0].abs_diff(capture_ns[1]) > 2_000_000
        {
            *PROBLEM.lock().unwrap() = shm::problem(&frames);
            WAITING.store(now_ns() - last_pair_seen_ns > 1_000_000_000, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        (last_pair_capture_ns, last_pair_seen_ns) = (capture_ns[0], now_ns());
        WAITING.store(false, Ordering::Relaxed);

        let start_ns = now_ns();
        last_run_ns = start_ns;
        let requested = *CORRECTION.lock().unwrap();
        if requested != correction {
            correction = requested;
            (rectifiers, view) = rectify(correction);
            let deg = correction.map(f32::to_degrees);
            println!("detector: rectification corrected by {:.3} {:.3} {:.3} deg (x y z)", deg[0], deg[1], deg[2]);
        }
        let (l_img, r_img) = input.split_at_mut(s * s);
        rectifiers[0].apply(&raw[0], l_img);
        rectifiers[1].apply(&raw[1], r_img);
        stretch(&mut input);
        let rectified_ns = now_ns();

        // prep_model.py folded GRAY2RGB and the 1/255 scale into the first conv.
        let out = net.run_gray(&input, s, 2 * s);
        if LOADING_SIZE.load(Ordering::Relaxed) != 0 {
            println!("detector: {s}px ready {:.1} s after loading started", (now_ns() - t_load) as f64 / 1e9);
            LOADING_SIZE.store(0, Ordering::Relaxed);
        }
        let yolo_done_ns = now_ns();
        let Some(out) = out else {
            continue;
        };
        if out.h() != 4 + COCO_CLASSES {
            return Err(format!("{model_path}: {} output rows, not 4 + {COCO_CLASSES}", out.h()));
        }
        let mut boxes = decode(&out, s);
        nms(&mut boxes[0]);
        nms(&mut boxes[1]);
        let (l_img, r_img) = input.split_at(s * s);
        let objs = stereo::match_views(&boxes, [l_img, r_img], s, focal_px, baseline, origins, &view.valid);
        let matched_ns = now_ns();
        let stereo_n = objs.iter().filter(|o| o.stereo).count();
        runs += 1;
        let align_matches = runs.is_multiple_of(ALIGN_EVERY).then(|| {
            let (matches, stats) = stereo::alignment(l_img, r_img, s); // a few ms
            align_stats_sum.tried += stats.tried;
            align_stats_sum.flat += stats.flat;
            align_stats_sum.weak += stats.weak;
            align_stats_sum.dy_at_limit += stats.dy_at_limit;
            align_stats_sum.d_at_limit += stats.d_at_limit;
            align_stats_sum.unclear += stats.unclear;
            align_stats_sum.found += stats.found;
            Arc::new(matches)
        });
        let aligned_ns = now_ns();
        {
            let mut latest = DETECTIONS.lock().unwrap();
            latest.id += 1;
            latest.t_capture_ns = capture_ns[0] - (LATENCY_MS * 1e6) as u64;
            latest.objs = objs;
            latest.boxes = boxes.clone();
            latest.view = Some(view.clone());
            latest.align_matches = align_matches;
            latest.correction = correction;
        }

        period_runs += 1;
        rectify_ms += (rectified_ns - start_ns) as f64 / 1e6;
        yolo_ms += (yolo_done_ns - rectified_ns) as f64 / 1e6;
        match_ms += (matched_ns - yolo_done_ns) as f64 / 1e6;
        align_ms_max = align_ms_max.max((aligned_ns - matched_ns) as f64 / 1e6);
        let now = now_ns();
        if now - period_start_ns > 5_000_000_000 {
            let n = period_runs as f64;
            println!(
                "detector: {:.1} fps | rectify {:.1} ms, yolo {:.1} ms, match {:.1} ms, align {:.0} ms | {}+{} dets, \
                 {stereo_n} objects | age {:.0} ms | align patches: {} tried, {} flat, {} weak, {} at dy's end, \
                 {} at d's end, {} unclear, {} found",
                n / ((now - period_start_ns) as f64 / 1e9),
                rectify_ms / n,
                yolo_ms / n,
                match_ms / n,
                align_ms_max,
                boxes[0].len(),
                boxes[1].len(),
                (now - capture_ns[0]) as f64 / 1e6,
                align_stats_sum.tried,
                align_stats_sum.flat,
                align_stats_sum.weak,
                align_stats_sum.dy_at_limit,
                align_stats_sum.d_at_limit,
                align_stats_sum.unclear,
                align_stats_sum.found
            );
            align_stats_sum = stereo::AlignStats::default();
            YOLO_MS.store((yolo_ms / n) as f32);
            (period_runs, rectify_ms, yolo_ms, match_ms, align_ms_max, period_start_ns) = (0, 0.0, 0.0, 0.0, 0.0, now);
        }
    }
    Ok(())
}

/// The detector thread: runs at SIZE, reloading when it changes, until quit.
pub fn run_loop() -> Result<(), String> {
    while !QUIT.load(Ordering::Relaxed) {
        run_at_size(SIZE.load(Ordering::Relaxed) as usize)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn det(x0: f32, y0: f32, x1: f32, y1: f32, score: f32) -> Detection {
        Detection { x0, y0, x1, y1, score, cls: 0 }
    }

    #[test]
    fn nms_keeps_people_behind_people() {
        // A person 2 m away (tall box) with someone smaller, farther, inside it and more confident: both stay. Two
        // boxes on one person, mostly overlapping: one.
        let mut boxes = vec![det(100.0, 50.0, 160.0, 260.0, 0.45), det(110.0, 60.0, 135.0, 130.0, 0.6)];
        nms(&mut boxes);
        assert_eq!(boxes.len(), 2);
        let mut boxes = vec![det(100.0, 50.0, 160.0, 260.0, 0.45), det(102.0, 55.0, 158.0, 250.0, 0.6)];
        nms(&mut boxes);
        assert!(boxes.len() == 1 && boxes[0].score == 0.6);
    }
}
