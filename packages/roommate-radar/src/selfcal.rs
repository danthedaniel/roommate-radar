//! Keeps the stereo pair rectified while running. The factory calibration (/persist) is off by about a degree on the
//! Frame: XRService refines the cameras' extrinsics while it runs (its log: "Current extrinsics differences to factory
//! calibration ... cam0 0.84 deg, cam1 0.53 deg") but doesn't publish them, and a degree between the views is 3-4 px of
//! disparity at 480, so stereo read a person 4 m away as 2.5 m. So the views' relative rotation is estimated from the
//! detector's own matches (stereo::alignment, every few runs) and corrected in the rectification (detector.rs: half of it
//! on each camera, so neither view's direction moves by more than half the error):
//!   - about the head's x and z axes (pitch, roll) from the matches' row offsets, which a rectified pair has none of,
//!     whatever the scene (pooled over the last POOL_RUNS runs: a plain room leaves too few in one);
//!   - about y (yaw: an offset in disparity) from the floor, whose depth along each ray below the horizon follows from
//!     the head pose (SteamVR's floor, y = 0). The floor is the farthest thing such a ray can meet, so the floor's matches
//!     are the low edge of (measured - floor) disparity, at the offset: FLOOR_EDGE_QUANTILE of all the floor samples, per unit of
//!     1 + x^2 (how a yaw shifts disparity across the view). Fitting that edge per floor distance, with a slope for an
//!     error in the floor's height, was tried: the slope came out anywhere from -0.15 to +0.33 between sessions (no floor
//!     is 20 cm off), and extrapolating it to infinity turned that into 0.3-0.6 deg of yaw error. Floor samples are only
//!     taken once pitch and roll have converged: before, they read 0.25 deg high.
//!
//! Each estimate moves the correction GAIN of the way; the correction is saved (settings.rs, `align`) so the next start
//! begins aligned. Measured on the Frame: the misalignment holds steady over head angle and across reboots (0.1 px), and
//! drifts by hundredths of a degree a minute, so this tracks it slowly. Replayed on the recordings, yaw from the floor came
//! out 0.73 deg (10-90% 0.69-0.82) in each session; people at known distances put it at 0.7-0.8.
use std::collections::VecDeque;

use crate::detector::{self, DetectionSet};
use crate::openvr::HmdMatrix34_t;
use crate::v3::{V3, transform, v3};

const GAIN: f32 = 0.3;
const PUBLISH_RAD: f32 = 0.02 * std::f32::consts::PI / 180.0; // the rectification is rebuilt for a change of this much
const MAX_CORRECTION_RAD: f32 = 3.0 * std::f32::consts::PI / 180.0; // more than this is something else going wrong: no further
const MIN_NCC: f32 = 0.85; // matches used
const MIN_ROW_MATCHES: usize = 20; // matches a pitch/roll estimate needs
const POOL_RUNS: usize = 8; // from this many alignment runs: a plain room gives ~5 a run
const ROW_OUTLIER_PX: f32 = 1.0; // after a first fit, matches off it by more than this are left out of the second
const CONVERGED_RAD: f32 = 0.1 * std::f32::consts::PI / 180.0; // pitch/roll this close: floor samples are taken
const MAX_FLOOR_SAMPLES: usize = 3000; // floor samples kept (a minute or two of looking around)
const MIN_FLOOR_SAMPLES: usize = 500; // and needed for a yaw estimate
const FLOOR_EDGE_QUANTILE: f32 = 0.1; // the floor's edge: this quantile of all the floor samples
const MIN_DOWN_SIN: f32 = 0.05; // floor samples: rays at least this far below the horizon (sine)
const FLOOR_DEPTH_M: (f32, f32) = (0.8, 8.0); // and meeting the floor this far away (m)
const LOG_NS: u64 = 30_000_000_000; // a status line this often

/// A floor sample: (measured - floor) disparity over 1 + x^2 (px), with the yaw correction in effect when measured.
struct FloorSample {
    excess_px: f32,
    applied_yaw: f32,
}

#[derive(Default)]
pub struct SelfCal {
    pub correction: [f32; 3], // the correction (rad, about the head's x, y, z): the left view's rotation relative to the right
    pool: VecDeque<(Vec<NormMatch>, [f32; 3])>, // the last POOL_RUNS runs' matches, with the correction applied to each
    floor_samples: VecDeque<FloorSample>,
    yaw_estimate: Option<f32>, // the floor's last (rad), for the log
    last_log_ns: u64,
}

/// A match in normalized view coordinates (X right, Y up) with its disparity and row offset (px).
type NormMatch = (f32, f32, f32, f32);

/// Pitch and roll (rad) that would take the matches' row offsets away: least squares of
/// dy = f (wx (1 + Y^2) + wz X), twice (the second without the first's outliers). Matches: (X, Y, d, dy).
fn fit_rows(pts: &[NormMatch], f: f32) -> Option<(f32, f32)> {
    let solve = |keep: &dyn Fn(&NormMatch) -> bool| {
        let (mut a, mut b, mut c, mut p, mut q, mut n) = (0f64, 0f64, 0f64, 0f64, 0f64, 0);
        for m @ &(x, y, _, dy) in pts {
            if !keep(m) {
                continue;
            }
            let (gx, gz, r) = ((f * (1.0 + y * y)) as f64, (f * x) as f64, dy as f64);
            (a, b, c, p, q, n) = (a + gx * gx, b + gx * gz, c + gz * gz, p + gx * r, q + gz * r, n + 1);
        }
        let det = a * c - b * b;
        (n >= MIN_ROW_MATCHES && det.abs() > 1e-9)
            .then(|| (((c * p - b * q) / det) as f32, ((a * q - b * p) / det) as f32))
    };
    let (wx, wz) = solve(&|_| true)?;
    solve(&|&(x, y, _, dy)| (dy - f * (wx * (1.0 + y * y) + wz * x)).abs() < ROW_OUTLIER_PX)
}

/// The disparity offset (px, per unit of 1 + x^2) at the floor samples' edge, with them as if measured at yaw
/// correction `yaw`; None until there are MIN_FLOOR_SAMPLES.
fn floor_edge(floor_samples: &VecDeque<FloorSample>, yaw: f32, f: f32) -> Option<f32> {
    if floor_samples.len() < MIN_FLOOR_SAMPLES {
        return None;
    }
    // A yaw correction w takes f w (1 + x^2) off the disparity: per unit of 1 + x^2, f w.
    let mut adjusted: Vec<f32> = floor_samples.iter().map(|s| s.excess_px - f * (yaw - s.applied_yaw)).collect();
    let k = (adjusted.len() as f32 * FLOOR_EDGE_QUANTILE) as usize;
    Some(*adjusted.select_nth_unstable_by(k, f32::total_cmp).1)
}

impl SelfCal {
    /// Starting from `correction` (rad), which the detector is given too.
    pub fn new(correction: [f32; 3]) -> SelfCal {
        *detector::CORRECTION.lock().unwrap() = correction;
        SelfCal { correction, ..SelfCal::default() }
    }

    /// A detector frame, with the head pose at its capture: its matches (when it has some) go into the estimate, and
    /// the correction to the detector once it has moved by PUBLISH_RAD.
    pub fn step(&mut self, dets: &DetectionSet, head: &HmdMatrix34_t) {
        let (Some(align_matches), Some(v)) = (&dets.align_matches, &dets.view) else {
            return;
        };
        self.update(
            align_matches,
            dets.correction,
            head,
            v.focal_px,
            v.size_px,
            v.origins[0],
            v.origins[1].x - v.origins[0].x,
        );
        let mut published = detector::CORRECTION.lock().unwrap();
        if self.moved_from(*published) {
            *published = self.correction;
        }
        if dets.t_capture_ns > self.last_log_ns + LOG_NS {
            self.last_log_ns = dets.t_capture_ns;
            let deg = self.correction.map(f32::to_degrees);
            let yaw = self.yaw_estimate.map_or("none yet".into(), |y| format!("{:.3} deg", y.to_degrees()));
            println!(
                "selfcal: {:.3} {:.3} {:.3} deg (x y z) | {} matches | floor: {} samples, yaw estimate {yaw}",
                deg[0],
                deg[1],
                deg[2],
                align_matches.len(),
                self.floor_samples.len()
            );
        }
    }

    /// Whether the correction has moved by PUBLISH_RAD from `saved`.
    pub fn moved_from(&self, saved: [f32; 3]) -> bool {
        (0..3).any(|k| (self.correction[k] - saved[k]).abs() > PUBLISH_RAD)
    }

    /// One frame's alignment matches (stereo::alignment: u, v, d, dy, ncc), measured with the correction `applied` in
    /// the rectification; head: the head pose at capture; f, size_px: the views' focal length and size; left_cam_head,
    /// baseline: the left camera in the head frame and the pair's baseline (m).
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        matches: &[[f32; 5]],
        applied: [f32; 3],
        head: &HmdMatrix34_t,
        f: f32,
        size_px: usize,
        left_cam_head: V3,
        baseline: f32,
    ) {
        let half_px = size_px as f32 / 2.0;
        let pts: Vec<NormMatch> = matches
            .iter()
            .filter(|m| m[4] >= MIN_NCC)
            .map(|m| ((m[0] + 0.5 - half_px) / f, -(m[1] + 0.5 - half_px) / f, m[2], m[3]))
            .collect();
        self.pool.push_back((pts.clone(), applied));
        if self.pool.len() > POOL_RUNS {
            self.pool.pop_front();
        }
        // Each run's row offsets as if measured with `applied`: a correction w more takes f (wx (1 + Y^2) + wz X) off.
        let pooled: Vec<NormMatch> = self
            .pool
            .iter()
            .flat_map(|(ms, applied_then)| {
                let (d_pitch, d_roll) = (applied[0] - applied_then[0], applied[2] - applied_then[2]);
                ms.iter().map(move |&(x, y, d, dy)| (x, y, d, dy - f * (d_pitch * (1.0 + y * y) + d_roll * x)))
            })
            .collect();
        let Some((wx, wz)) = fit_rows(&pooled, f) else {
            return;
        };
        self.correction[0] += GAIN * (applied[0] + wx - self.correction[0]);
        self.correction[2] += GAIN * (applied[2] + wz - self.correction[2]);
        if wx.abs() < CONVERGED_RAD && wz.abs() < CONVERGED_RAD {
            let left_cam_universe = transform(head, left_cam_head);
            let m = &head.m;
            for &(x, y, d, _) in &pts {
                // the ray in the standing universe (the head frame's (x, y, -1), rotated)
                let r = |i: usize| m[i][0] * x + m[i][1] * y - m[i][2];
                let ray = v3(r(0), r(1), r(2));
                let floor_depth = -left_cam_universe.y / ray.y; // depth (along head -z) at which it meets y = 0
                if ray.y < -MIN_DOWN_SIN * ray.len() && (FLOOR_DEPTH_M.0..FLOOR_DEPTH_M.1).contains(&floor_depth) {
                    let d_floor = f * baseline / floor_depth;
                    self.floor_samples
                        .push_back(FloorSample { excess_px: (d - d_floor) / (1.0 + x * x), applied_yaw: applied[1] });
                }
            }
            while self.floor_samples.len() > MAX_FLOOR_SAMPLES {
                self.floor_samples.pop_front();
            }
        }
        if let Some(edge_px) = floor_edge(&self.floor_samples, applied[1], f) {
            let target = applied[1] + edge_px / f;
            self.yaw_estimate = Some(target);
            self.correction[1] += GAIN * (target - self.correction[1]);
        }
        self.correction = self.correction.map(|w| w.clamp(-MAX_CORRECTION_RAD, MAX_CORRECTION_RAD));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rectify::rodrigues;

    const VIEW_480: (usize, f32) = (480, 201.4);
    const VIEW_640: (usize, f32) = (640, 268.5);
    const BASELINE_M: f32 = 0.1056;
    const LEFT_CAM: V3 = v3(-0.0528, -0.0186, -0.0369);
    const HEAD: HmdMatrix34_t = HmdMatrix34_t { m: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 1.6], [0.0, 0.0, 1.0, 0.0]] };

    fn mv(r: &[f32; 9], p: V3) -> V3 {
        v3(
            r[0] * p.x + r[1] * p.y + r[2] * p.z,
            r[3] * p.x + r[4] * p.y + r[5] * p.z,
            r[6] * p.x + r[7] * p.y + r[8] * p.z,
        )
    }

    /// Pixel of head-frame point p in an s x s view (focal length f) from `origin`, its content rotated by `rot`.
    fn project(p: V3, origin: V3, rot: &[f32; 9], (s, f): (usize, f32)) -> (f32, f32) {
        let q = mv(rot, p - origin);
        (s as f32 / 2.0 + f * q.x / -q.z - 0.5, s as f32 / 2.0 - f * q.y / -q.z - 0.5)
    }

    /// Matches of a room: the floor at `floor_y` (head frame), things on it (nearer along the same rays), and walls,
    /// with the left view's content off by `err` relative to the right after `corr` is applied.
    fn matches(err: [f32; 3], corr: [f32; 3], floor_y: f32, seed: &mut u32, view: (usize, f32)) -> Vec<[f32; 5]> {
        let mut rnd = || {
            *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (*seed >> 8) as f32 / (1u32 << 24) as f32
        };
        let rot = rodrigues([err[0] + corr[0], err[1] + corr[1], err[2] + corr[2]]);
        let id = rodrigues([0.0; 3]);
        let mut out = Vec::new();
        for _ in 0..80 {
            let (x, z) = (rnd() * 6.0 - 3.0, -1.0 - rnd() * 6.0);
            let mut p = v3(x, floor_y, z);
            if rnd() < 0.4 {
                p = p * (0.5 + 0.45 * rnd()); // something standing on the floor's ray, nearer
            }
            if rnd() < 0.3 {
                p = v3(x, rnd() * 1.5 - 0.5, z); // a wall, above and below the horizon
            }
            let ((ul, vl), (ur, vr)) =
                (project(p, LEFT_CAM, &rot, view), project(p, LEFT_CAM + v3(BASELINE_M, 0.0, 0.0), &id, view));
            if [ul, vl, ur, vr].iter().all(|&c| (0.0..view.0 as f32).contains(&c)) {
                out.push([ul, vl, ul - ur, vl - vr, 0.95]);
            }
        }
        out
    }

    fn converge(err: [f32; 3], floor_y: f32, view: (usize, f32)) -> [f32; 3] {
        let mut cal = SelfCal::default();
        let mut seed = 7;
        for _ in 0..80 {
            let applied = cal.correction;
            let m = matches(err, applied, floor_y, &mut seed, view);
            cal.update(&m, applied, &HEAD, view.1, view.0, LEFT_CAM, BASELINE_M);
        }
        cal.correction
    }

    #[test]
    fn finds_the_misalignment() {
        // The left view off by 0.66 deg pitch, 1.1 deg yaw, -0.5 deg roll (the Frame's size; its yaw is ~0.75): the
        // correction converges to undo it, to a few hundredths of a degree.
        let err = [0.66f32, 1.1, -0.5].map(f32::to_radians);
        for view in [VIEW_480, VIEW_640] {
            let w = converge(err, -1.6, view);
            for k in 0..3 {
                let (got, want) = (w[k].to_degrees(), -err[k].to_degrees());
                assert!((got - want).abs() < 0.04, "{} px, axis {k}: {got:.3} deg vs {want:.3}", view.0);
            }
        }
    }

    #[test]
    fn few_matches_a_run_are_pooled() {
        // A plain room: 5 matches a run, too few on their own; pooled over runs, pitch and roll still converge.
        let err = [0.66f32, 0.0, -0.5].map(f32::to_radians);
        let mut cal = SelfCal::default();
        let mut seed = 7;
        for _ in 0..80 {
            let applied = cal.correction;
            let m = matches(err, applied, -1.6, &mut seed, VIEW_480);
            cal.update(&m[..5], applied, &HEAD, VIEW_480.1, VIEW_480.0, LEFT_CAM, BASELINE_M);
        }
        for k in [0, 2] {
            let (got, want) = (cal.correction[k].to_degrees(), -err[k].to_degrees());
            assert!((got - want).abs() < 0.04, "axis {k}: {got:.3} deg vs {want:.3}");
        }
    }

    #[test]
    fn floor_height_error_moves_the_yaw_little() {
        // SteamVR's floor 3 cm off the real one (it's taken as right): the yaw comes out a few hundredths of a degree
        // off, not more.
        let err = [0.3f32, -0.8, 0.2].map(f32::to_radians);
        let w = converge(err, -1.63, VIEW_480);
        let (got, want) = (w[1].to_degrees(), -err[1].to_degrees());
        assert!((got - want).abs() < 0.06, "{got:.3} deg vs {want:.3}");
    }

    #[test]
    fn nothing_to_see_changes_nothing() {
        let mut cal = SelfCal::new([0.01, 0.02, 0.03]);
        cal.update(&[], [0.01, 0.02, 0.03], &HEAD, VIEW_480.1, VIEW_480.0, LEFT_CAM, BASELINE_M);
        assert_eq!(cal.correction, [0.01, 0.02, 0.03]);
    }
}
