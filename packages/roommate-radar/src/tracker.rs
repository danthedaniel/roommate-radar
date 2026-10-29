//! Multi-object tracker in 3D (standing universe), ByteTrack-style with Mahalanobis gating. Each track is one
//! constant-velocity Kalman filter over its box center's position and velocity (6 states, kept as 3x3 blocks) plus a
//! smoothed metric box size. Measurements come with their real noise shape: stereo depth is far worse than bearing, so
//! each one is a long thin ellipsoid along its viewing ray (measure.rs). Matching uses the squared Mahalanobis distance
//! under S = P + R with a chi-square gate, plus ln|S| so a vague far track doesn't claim every detection (UCMCTrack).
//! A depth jump along the ray then still matches its track instead of starting a second one. Per detector frame:
//!   1. merge detections of one object (within each other's noise),
//!   2. predict every track to the frame's capture time,
//!   3. match tracks to confident detections (Hungarian), then the tracks left over to weak ones (low confidence, or
//!      seen in one view only): weak ones keep a track going, with more noise, but never start one; below LOW, nothing,
//!   4. unmatched confident detections start tentative tracks, unless they're within an existing track's gate,
//!   5. tracks that have become indistinguishable are merged; tentative tracks that miss are dropped, confirmed ones
//!      after missing for MAX_UNSEEN while they should have been seen (`seen`: in view and in range; out of view a
//!      track waits to be found again), or MAX_AGE after their last match in any case.
//!
//! Each track keeps its last few filter states (position, velocity, size) for drawing (boxes.rs).
use std::collections::VecDeque;

use crate::v3::{V3, v3};

#[derive(Clone, Copy, Debug)]
pub struct Meas {
    pub p: V3, // box center (m)
    pub w: f32,
    pub h: f32,         // box size (m)
    pub ray: V3,        // unit vector from the camera to p: stereo's depth error is along it
    pub sigma_ray: f32, // position std dev along the ray (m)
    pub sigma_lat: f32, // and across it (m)
    pub score: f32,
    pub cls: usize,
    pub eye: V3,      // head position at capture
    pub clip_w: bool, // the box's width (height) is cut off by the view's edge: it's not the object's
    pub clip_h: bool,
    pub weak: bool, // seen in one view only (depth from where it meets the floor)
}

const HIGH: f32 = 0.5; // detections at least this confident can start tracks (step 3); the rest only extend them
const LOW: f32 = 0.25; // and below this they don't even do that: a person track walked onto a bicycle at 0.15
const WEAK_NOISE: f32 = 2.0; // weak detections' position noise (std dev) is this many times what they measured
const GATE: f32 = 11.34; // chi-square, 3 dof, 99%: max squared Mahalanobis distance for a match
const BIRTH_GATE: f32 = 16.27; // 99.9%: an unmatched detection this close to a track doesn't start another
const MERGE_GATE: f32 = 11.34; // tracks closer than this (squared Mahalanobis under both covariances) are one
const MAX_DIST: f32 = 4.0; // and nothing matches or merges farther apart than this (m), whatever the noise says
const MERGE_SPEED: f32 = 0.8; // nor tracks whose velocities differ by more than this (m/s): two people passing
pub const MIN_HITS: u32 = 2; // matches before a track is confirmed (shown)
const MAX_UNSEEN: f64 = 1.0; // drop a track after missing for this long (s) while it should have been seen
const MAX_AGE: f64 = 5.0; // and this long (s) after its last match in any case: out of view, it can still be found
pub const SHOW_S: f64 = 0.6; // tracks matched within this long (s) are drawn; boxes.rs fades them out after
const REACQUIRE: f64 = 0.75; // matched again after this long (s) without: restart its motion from the detection
const ACCEL: f32 = 2.0; // process noise: std dev of a person's acceleration (m/s^2)
const VEL0: f32 = 1.0; // a new track's velocity std dev (m/s)
const HISTORY: usize = 4; // filter states kept per track, for drawing
// Size and class: a step is 90% there after this long (s), whatever the detection rate.
const SETTLE_S: f64 = 1.0;
pub const CLASSES: [usize; 3] = [0, 15, 16]; // person, cat, dog: COCO's ids for what we track
const CLASS_SWITCH: f32 = 1.5; // a track's class changes when another one's score is this many times its own

type M3 = [[f32; 3]; 3];

fn mat(f: impl Fn(usize, usize) -> f32) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| f(i, j)))
}

fn mul(a: &M3, b: &M3) -> M3 {
    mat(|i, j| (0..3).map(|k| a[i][k] * b[k][j]).sum())
}

fn add(a: &M3, b: &M3) -> M3 {
    mat(|i, j| a[i][j] + b[i][j])
}

fn sub(a: &M3, b: &M3) -> M3 {
    mat(|i, j| a[i][j] - b[i][j])
}

fn tr(a: &M3) -> M3 {
    mat(|i, j| a[j][i])
}

fn diag(s: f32) -> M3 {
    mat(|i, j| if i == j { s } else { 0.0 })
}

fn sym(a: &M3) -> M3 {
    mat(|i, j| (a[i][j] + a[j][i]) / 2.0)
}

fn mv(a: &M3, v: V3) -> V3 {
    let r = |i: usize| a[i][0] * v.x + a[i][1] * v.y + a[i][2] * v.z;
    v3(r(0), r(1), r(2))
}

/// Inverse and determinant; None if (nearly) singular.
fn inv(a: &M3) -> Option<(M3, f32)> {
    let c = |i: usize, j: usize| {
        let (i1, i2, j1, j2) = ((i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3);
        a[i1][j1] * a[i2][j2] - a[i1][j2] * a[i2][j1]
    };
    let det = a[0][0] * c(0, 0) + a[0][1] * c(0, 1) + a[0][2] * c(0, 2);
    if det.abs() < 1e-12 || !det.is_finite() {
        return None;
    }
    Some((mat(|i, j| c(j, i) / det), det))
}

/// The measurement's position covariance: sigma_lat across its ray, sigma_ray along it; times scale^2.
fn noise(m: &Meas, scale: f32) -> M3 {
    let (l, r) = ((m.sigma_lat * scale).powi(2), (m.sigma_ray * scale).powi(2));
    let u = [m.ray.x, m.ray.y, m.ray.z];
    mat(|i, j| (if i == j { l } else { 0.0 }) + (r - l) * u[i] * u[j])
}

/// How much noisier than measured a detection is taken to be: weak ones (low confidence, one view) count for less.
fn noise_scale(m: &Meas) -> f32 {
    if m.weak || m.score < HIGH { WEAK_NOISE } else { 1.0 }
}

/// Squared Mahalanobis distance between a and b under covariance s, and ln|s|; None if too far apart or singular.
fn mahalanobis(a: V3, b: V3, s: &M3) -> Option<(f32, f32)> {
    let d = a - b;
    if d.len() > MAX_DIST {
        return None;
    }
    let (si, det) = inv(s)?;
    Some((d.dot(mv(&si, d)), det.max(1e-30).ln()))
}

fn class_index(cls: usize) -> usize {
    CLASSES.iter().position(|&c| c == cls).unwrap_or(0)
}

/// One filter state, for drawing: time (s), position and velocity, size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub p: V3,
    pub v: V3,
    pub w: f32,
    pub h: f32,
}

#[derive(Clone, Debug)]
pub struct Track {
    pub id: u64,
    pub p: V3, // box center position (m) and velocity (m/s)
    pub v: V3,
    pa: M3, // covariance: position, position-velocity, velocity
    pb: M3,
    pc: M3,
    pub w: f32, // box size (m), smoothed
    pub h: f32,
    pub t: f64,          // time the filter state is at (s)
    pub last_seen: f64,  // time of the last match (s)
    pub shown_from: f64, // when it was confirmed, or matched again after a gap: boxes.rs fades it in from there
    unseen: f64,         // time missed while it should have been seen, since the last match (s)
    pub hits: u32,
    pub cls: usize,
    class_score: [f32; 3], // per CLASSES, decaying sums of the matched detections' scores
    pub score: f32,
    pub history: VecDeque<Sample>, // last HISTORY filter states after a match, oldest first
}

impl Track {
    fn new(id: u64, m: &Meas, t: f64) -> Track {
        let mut tr = Track {
            id,
            p: m.p,
            v: V3::default(),
            pa: noise(m, 1.0),
            pb: diag(0.0),
            pc: diag(VEL0 * VEL0),
            w: m.w,
            h: m.h,
            t,
            last_seen: t,
            shown_from: t,
            unseen: 0.0,
            hits: 1,
            cls: m.cls,
            class_score: [0.0; 3],
            score: m.score,
            history: VecDeque::new(),
        };
        tr.class_score[class_index(m.cls)] = m.score;
        tr.remember();
        tr
    }

    /// The position covariance.
    pub fn cov(&self) -> M3 {
        self.pa
    }

    /// Constant velocity to time t, with white-noise acceleration (ACCEL).
    fn predict(&mut self, t: f64) {
        let dt = (t - self.t).max(0.0) as f32;
        let q = ACCEL * ACCEL;
        let (a, b, c) = (&self.pa, &self.pb, &self.pc);
        let bt = tr(b);
        self.pa = mat(|i, j| {
            a[i][j] + dt * (b[i][j] + bt[i][j]) + dt * dt * c[i][j] + if i == j { q * dt.powi(3) / 3.0 } else { 0.0 }
        });
        self.pb = mat(|i, j| b[i][j] + dt * c[i][j] + if i == j { q * dt * dt / 2.0 } else { 0.0 });
        self.pc = mat(|i, j| c[i][j] + if i == j { q * dt } else { 0.0 });
        // Moving at the last velocity for at most REACQUIRE after the last match: a track out of view is looked for
        // around where it was last seen, with the uncertainty growing, not sent off along a guess.
        let moving = ((self.last_seen + REACQUIRE).min(t) - self.t).max(0.0) as f32;
        self.p = self.p + self.v * moving;
        self.t = self.t.max(t);
    }

    /// m's noise for this track. A box that changed size moved its center without the object moving (arms reaching
    /// out widened a seated person's box from 1.2 to 2.4 m, and its depth jumped too), by up to half the change.
    fn meas_noise(&self, m: &Meas) -> M3 {
        let k = (self.p - m.eye).len() / (m.p - m.eye).len().max(0.1);
        let dw = if m.clip_w { 0.0 } else { (m.w * k - self.w).abs() };
        let dh = if m.clip_h { 0.0 } else { (m.h * k - self.h).abs() };
        add(&noise(m, noise_scale(m)), &diag((dw.max(dh) / 2.0).powi(2)))
    }

    /// Squared Mahalanobis distance of m from the prediction, and ln|S|.
    fn distance(&self, m: &Meas) -> Option<(f32, f32)> {
        mahalanobis(m.p, self.p, &add(&self.pa, &self.meas_noise(m)))
    }

    /// Kalman update with m (position measured, H = [I 0]); sizes and class smoothed.
    fn update(&mut self, m: &Meas, t: f64) {
        if t - self.last_seen > REACQUIRE {
            // Back after a long gap (out of view, occluded): the coasted velocity is a guess, start over from here.
            (self.p, self.v, self.pa, self.pb, self.pc) =
                (m.p, V3::default(), noise(m, 1.0), diag(0.0), diag(VEL0 * VEL0));
            self.history.clear();
        } else if let Some((si, _)) = inv(&add(&self.pa, &self.meas_noise(m))) {
            let (kp, kv) = (mul(&self.pa, &si), mul(&tr(&self.pb), &si));
            let y = m.p - self.p;
            self.p = self.p + mv(&kp, y);
            self.v = self.v + mv(&kv, y);
            let (a, b, c) = (self.pa, self.pb, self.pc);
            self.pa = sym(&sub(&a, &mul(&kp, &a)));
            self.pb = sub(&b, &mul(&kp, &b));
            self.pc = sym(&sub(&c, &mul(&kv, &b)));
        }
        let alpha = alpha(t - self.last_seen);
        // Size from the box's angular size at the filtered distance (not the measured one: stereo depth noise would
        // make it breathe), only from sides that aren't cut off by the view's edge.
        let k = (self.p - m.eye).len() / (m.p - m.eye).len().max(0.1);
        if !m.clip_w {
            self.w += alpha * (m.w * k - self.w);
        }
        if !m.clip_h {
            self.h += alpha * (m.h * k - self.h);
        }
        for s in &mut self.class_score {
            *s *= 1.0 - alpha;
        }
        self.class_score[class_index(m.cls)] += alpha * m.score;
        let (best, cur) =
            (self.class_score.iter().copied().fold(0.0, f32::max), self.class_score[class_index(self.cls)]);
        if best > CLASS_SWITCH * cur {
            self.cls = CLASSES[self.class_score.iter().position(|&s| s == best).unwrap()];
        }
        self.hits += 1;
        if self.hits == MIN_HITS || t - self.last_seen > SHOW_S {
            self.shown_from = t;
        }
        (self.last_seen, self.score, self.unseen) = (t, m.score, 0.0);
        self.remember();
    }

    fn remember(&mut self) {
        let s = Sample { t: self.t, p: self.p, v: self.v, w: self.w, h: self.h };
        if self.history.back().is_none_or(|b| s.t > b.t) {
            self.history.push_back(s);
        }
        while self.history.len() > HISTORY {
            self.history.pop_front();
        }
    }
}

#[cfg(test)]
impl Track {
    /// A tentative track found at time t, 2 m ahead, for other modules' tests.
    pub fn for_test(t: f64) -> Track {
        let m = Meas {
            p: v3(0.0, 1.0, -2.0),
            w: 0.5,
            h: 1.7,
            ray: v3(0.0, 0.0, -1.0),
            sigma_ray: 0.1,
            sigma_lat: 0.05,
            score: 0.9,
            cls: 0,
            eye: v3(0.0, 1.6, 0.0),
            clip_w: false,
            clip_h: false,
            weak: false,
        };
        Track::new(1, &m, t)
    }
}

/// Smoothing factor for a sample dt (s) after the previous one: 10% of a step is left after SETTLE_S.
fn alpha(dt: f64) -> f32 {
    (1.0 - 0.1f64.powf(dt.max(0.0) / SETTLE_S)) as f32
}

/// Detections of one object (a person boxed twice, a person and a dog box on one body), within each other's noise:
/// only the most confident (seen in both views first) is kept.
fn merge_detections(dets: &[Meas]) -> Vec<Meas> {
    let mut order: Vec<&Meas> = dets.iter().collect();
    order.sort_by(|a, b| a.weak.cmp(&b.weak).then(b.score.total_cmp(&a.score)));
    let mut keep: Vec<Meas> = Vec::new();
    for m in order {
        let close =
            |k: &Meas| mahalanobis(m.p, k.p, &add(&noise(m, 1.0), &noise(k, 1.0))).is_some_and(|(d2, _)| d2 < GATE);
        if !keep.iter().any(close) {
            keep.push(*m);
        }
    }
    keep
}

/// Minimum-cost assignment of rows to columns (Hungarian, O(n^2 m)); match[row] = column. Pairs costing more than
/// `gate` are left unmatched.
pub fn assign(cost: &[Vec<f32>], gate: f32) -> Vec<Option<usize>> {
    let n = cost.len();
    let m = cost.first().map_or(0, |r| r.len());
    let mut matched = vec![None; n];
    if n == 0 || m == 0 {
        return matched;
    }
    let flip = n > m; // the algorithm wants rows <= columns
    let (rows, cols) = if flip { (m, n) } else { (n, m) };
    // 1-based; over-gate (and NaN) pairs get a large finite cost so they're picked last; a NaN would never compare
    // below `delta` and the search below would loop forever
    let a = |i: usize, j: usize| {
        let c = if flip { cost[j - 1][i - 1] } else { cost[i - 1][j - 1] };
        if c <= gate { c } else { gate * 100.0 }
    };
    let (mut u, mut v) = (vec![0f32; rows + 1], vec![0f32; cols + 1]);
    let (mut p, mut way) = (vec![0usize; cols + 1], vec![0usize; cols + 1]);
    for i in 1..=rows {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![f32::INFINITY; cols + 1];
        let mut used = vec![false; cols + 1];
        loop {
            used[j0] = true;
            let (i0, mut j1, mut delta) = (p[j0], 0, f32::INFINITY);
            for j in 1..=cols {
                if !used[j] {
                    let cur = a(i0, j) - u[i0] - v[j];
                    if cur < minv[j] {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if minv[j] < delta {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for j in 0..=cols {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    for (j, &pj) in p.iter().enumerate().skip(1) {
        if pj == 0 {
            continue;
        }
        let (row, col) = if flip { (j - 1, pj - 1) } else { (pj - 1, j - 1) };
        if cost[row][col] <= gate {
            matched[row] = Some(col);
        }
    }
    matched
}

const COST_OFFSET: f32 = 30.0; // keeps d^2 + ln|S| (ln|S| > -25 for a few cm of noise) positive for assign()
const CLASS_COST: f32 = 2.0; // a detection of another class than the track's still matches, at this extra cost
const NO_MATCH: f32 = 1e3; // assign()'s gate: above every gated cost

#[derive(Default)]
pub struct Tracker {
    pub tracks: Vec<Track>,
    next_id: u64,
}

impl Tracker {
    /// One detector frame captured at time t (s). seen(p): whether something at p would have been detected then (in
    /// both views, in range): only then does missing it count against its track.
    pub fn update(&mut self, dets: &[Meas], t: f64, seen: impl Fn(V3) -> bool) {
        let dets = merge_detections(dets);
        let dts: Vec<f64> = self.tracks.iter().map(|tr| (t - tr.t).max(0.0)).collect();
        for tr in &mut self.tracks {
            tr.predict(t);
        }
        let (strong, weak): (Vec<usize>, Vec<usize>) =
            (0..dets.len()).filter(|&i| dets[i].score >= LOW).partition(|&i| !dets[i].weak && dets[i].score >= HIGH);
        let mut matched = vec![false; self.tracks.len()];
        let unused = self.match_dets(&dets, &strong, &mut matched, t, false);
        // Not tentative tracks (ByteTrack): a hand's box and a weak one 0.7 m below it confirmed a track that flew off.
        self.match_dets(&dets, &weak, &mut matched, t, true);
        for (i, tr) in self.tracks.iter_mut().enumerate() {
            if !matched[i] && seen(tr.p) {
                tr.unseen += dts[i];
            }
        }
        for i in unused {
            let d = &dets[i];
            // Within a track's gate (a second detection of it, or a depth jump the match left over): not a new object.
            if self.tracks.iter().any(|tr| tr.distance(d).is_some_and(|(d2, _)| d2 < BIRTH_GATE)) {
                continue;
            }
            self.next_id += 1;
            self.tracks.push(Track::new(self.next_id, d, t));
            matched.push(true);
        }
        let gone = self.indistinguishable();
        let mut i = 0;
        self.tracks.retain(|tr| {
            let keep = !gone[i]
                && (tr.hits >= MIN_HITS || matched[i])
                && tr.unseen <= MAX_UNSEEN
                && t - tr.last_seen <= MAX_AGE;
            i += 1;
            keep
        });
    }

    /// Tracks that are one object (one started on a depth jump, before its gate grew, and both kept up since): within
    /// each other's uncertainty and moving alike. Of each such pair the one matched last (then more often) stays; the other is marked.
    fn indistinguishable(&self) -> Vec<bool> {
        let n = self.tracks.len();
        let mut gone = vec![false; n];
        for i in 0..n {
            for j in i + 1..n {
                let (a, b) = (&self.tracks[i], &self.tracks[j]);
                let close = mahalanobis(a.p, b.p, &add(&a.pa, &b.pa)).is_some_and(|(d2, _)| d2 < MERGE_GATE);
                if gone[i] || gone[j] || !close || (a.v - b.v).len() > MERGE_SPEED {
                    continue;
                }
                let a_stays = (a.last_seen, a.hits) >= (b.last_seen, b.hits);
                gone[if a_stays { j } else { i }] = true;
            }
        }
        gone
    }

    /// Matches still-unmatched tracks to dets[which]; updates the matched tracks. Returns the unmatched detections.
    fn match_dets(
        &mut self,
        dets: &[Meas],
        which: &[usize],
        matched: &mut [bool],
        t: f64,
        confirmed_only: bool,
    ) -> Vec<usize> {
        let rows: Vec<usize> = (0..self.tracks.len())
            .filter(|&i| !matched[i] && (!confirmed_only || self.tracks[i].hits >= MIN_HITS))
            .collect();
        let cost: Vec<Vec<f32>> = rows
            .iter()
            .map(|&r| {
                let tr = &self.tracks[r];
                which
                    .iter()
                    .map(|&c| match tr.distance(&dets[c]) {
                        Some((d2, ln_det)) if d2 < GATE => {
                            let class = if dets[c].cls != tr.cls { CLASS_COST } else { 0.0 };
                            (d2 + ln_det + COST_OFFSET + class).max(0.0)
                        }
                        _ => f32::INFINITY,
                    })
                    .collect()
            })
            .collect();
        let m = assign(&cost, NO_MATCH);
        let mut used = vec![false; which.len()];
        for (r, col) in m.into_iter().enumerate() {
            let Some(col) = col else {
                continue;
            };
            self.tracks[rows[r]].update(&dets[which[col]], t);
            matched[rows[r]] = true;
            used[col] = true;
        }
        (0..which.len()).filter(|&c| !used[c]).map(|c| which[c]).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A detection at p seen from the origin, with stereo-like noise: sigma_ray grows like depth^2.
    fn meas(p: V3, score: f32) -> Meas {
        let r = p.len();
        Meas {
            p,
            w: 0.5,
            h: 1.7,
            ray: p * (1.0 / r),
            sigma_ray: 0.05 + 0.02 * r * r,
            sigma_lat: 0.03 + r / 300.0,
            score,
            cls: 0,
            eye: V3::default(),
            clip_w: false,
            clip_h: false,
            weak: false,
        }
    }

    fn always(_: V3) -> bool {
        true
    }

    /// Deterministic noise in [-1, 1].
    fn noise_fn(seed: u32) -> impl FnMut() -> f32 {
        let mut seed = seed;
        move || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            ((seed >> 9) as i32 % 2001 - 1000) as f32 / 1000.0
        }
    }

    #[test]
    fn assignment_is_optimal_not_greedy() {
        // Greedy takes the cheapest pair (0,0)=1 and is left with (1,1)=10; optimum is 2+2.
        assert_eq!(assign(&[vec![1.0, 2.0], vec![2.0, 10.0]], 100.0), [Some(1), Some(0)]);
    }

    #[test]
    fn assignment_rectangular_gated() {
        let m = assign(&[vec![0.1, 5.0], vec![5.0, 5.0], vec![5.0, 0.2]], 1.0); // more rows than columns
        assert_eq!(m, [Some(0), None, Some(1)]);
    }

    #[test]
    fn assignment_nan_cost_is_gated() {
        assert_eq!(assign(&[vec![f32::NAN]], 1.0), [None]); // used to hang
    }

    #[test]
    fn inverse() {
        let a = [[2.0, 0.5, 0.0], [0.5, 1.0, 0.2], [0.0, 0.2, 3.0]];
        let (ai, det) = inv(&a).unwrap();
        let i = mul(&a, &ai);
        assert!((0..3).all(|r| (0..3).all(|c| (i[r][c] - if r == c { 1.0 } else { 0.0 }).abs() < 1e-5)));
        assert!((det - (2.0 * (3.0 - 0.04) - 0.5 * 1.5)).abs() < 1e-5);
        assert!(inv(&diag(0.0)).is_none());
    }

    #[test]
    fn walking_person() {
        // At 1 m/s along x, 3 m away, detected at 4 Hz with stereo-like noise (along the ray): confirmed on the 2nd frame, velocity learned, the last
        // 4 states kept; then a weak detection extends the track but starts nothing on its own, and a 0.15 one does neither.
        let mut trk = Tracker::default();
        let mut n = noise_fn(1);
        for i in 0..12 {
            let t = i as f64 * 0.25;
            let p = v3(t as f32 - 1.5, 0.0, -3.0);
            let (along, across) = (p * (1.0 / p.len()), v3(-p.z, 0.0, p.x) * (1.0 / p.len()));
            trk.update(&[meas(p + along * (0.2 * n()) + across * (0.03 * n()), 0.8)], t, always);
            if i == 0 {
                assert!(trk.tracks.len() == 1 && trk.tracks[0].hits < MIN_HITS, "new track is tentative");
            }
            if i == 1 {
                assert!(trk.tracks.len() == 1 && trk.tracks[0].hits >= MIN_HITS, "second hit confirms it");
            }
        }
        let tr = &trk.tracks[0];
        // (+-0.3 m/s is this filter's velocity noise here: numpy's 6-state filter on the same data agrees to 1e-6)
        assert!(trk.tracks.len() == 1 && (tr.v.x - 1.0).abs() < 0.35 && tr.v.z.abs() < 0.35, "velocity {:?}", tr.v);
        assert!(tr.history.len() == 4 && tr.history[0].t == 8.0 * 0.25 && tr.history[3].t == 11.0 * 0.25);

        let far = Meas { weak: true, ..meas(v3(-3.0, 0.0, -3.0), 0.9) };
        trk.update(&[meas(v3(1.5, 0.0, -3.0), 0.3), far], 3.0, always);
        assert!(trk.tracks.len() == 1 && trk.tracks[0].last_seen == 3.0);
        trk.update(&[meas(v3(1.75, 0.0, -3.0), 0.15)], 3.25, always);
        assert!(trk.tracks.len() == 1 && trk.tracks[0].last_seen == 3.0, "below LOW: no match");
    }

    #[test]
    fn weak_detections_dont_confirm() {
        let mut trk = Tracker::default();
        trk.update(&[meas(v3(0.0, 0.0, -2.0), 0.8)], 0.0, always);
        trk.update(&[Meas { weak: true, ..meas(v3(0.0, -0.3, -2.0), 0.8) }], 0.25, always);
        assert!(trk.tracks.iter().all(|tr| tr.hits < MIN_HITS));
    }

    #[test]
    fn arms_out_dont_move_a_seated_person() {
        // 2.7 m away, still; after 2 s, now and then the box is twice as wide (arms out), its center 0.3 m to the side
        // (half the change) and 0.4 m farther. Up to 1.5 m/s without meas_noise's size term, 0.9 with it: what's left
        // is the constant-velocity model believing a jump (ACCEL).
        let mut trk = Tracker::default();
        for i in 0..30 {
            let arms = i >= 8 && i % 3 == 0;
            let m = meas(v3(if arms { 0.3 } else { 0.0 }, 0.0, if arms { -3.1 } else { -2.7 }), 0.8);
            trk.update(&[Meas { w: if arms { 1.2 } else { 0.6 }, ..m }], i as f64 * 0.25, always);
            let tr = &trk.tracks[0];
            assert!(trk.tracks.len() == 1 && (i < 8 || tr.v.len() < 1.1), "frame {i}: {:?}", tr.v);
        }
    }

    #[test]
    fn depth_jumps_along_the_ray_stay_one_track() {
        // A person standing 7 m ahead: stereo depth swings by +-1.2 m (about 1 sigma there), bearing is good. A fixed
        // 0.7 m gate started a second track on every such jump.
        let mut trk = Tracker::default();
        let mut n = noise_fn(2);
        for i in 0..40 {
            let z = -7.0 + if i % 2 == 0 { 1.2 } else { -1.2 } + 0.2 * n();
            trk.update(&[meas(v3(0.02 * n(), 0.0, z), 0.8)], i as f64 * 0.25, always);
            assert_eq!(trk.tracks.len(), 1, "frame {i}");
        }
        assert!((trk.tracks[0].p.z + 7.0).abs() < 0.8 && trk.tracks[0].p.x.abs() < 0.05);
    }

    #[test]
    fn side_by_side_at_range_stay_two() {
        // Two people 0.6 m apart, 6 m away: far less than the depth noise, far more than the bearing noise.
        let mut trk = Tracker::default();
        let mut n = noise_fn(3);
        for i in 0..20 {
            let dets = [meas(v3(-0.3, 0.0, -6.0 + 0.6 * n()), 0.8), meas(v3(0.3, 0.0, -6.0 + 0.6 * n()), 0.8)];
            trk.update(&dets, i as f64 * 0.25, always);
        }
        assert_eq!(trk.tracks.len(), 2);
        assert!(
            trk.tracks.iter().any(|t| (t.p.x + 0.3).abs() < 0.1)
                && trk.tracks.iter().any(|t| (t.p.x - 0.3).abs() < 0.1)
        );
    }

    #[test]
    fn crossing_tracks_stay_apart_then_expire() {
        // Two people crossing paths keep their own tracks (prediction + optimal assignment), then both expire.
        let mut two = Tracker::default();
        for i in 0..12 {
            let t = i as f32 * 0.25;
            let x = -1.5 + t; // A moves +x, B moves -x; they pass at t = 1.5
            two.update(&[meas(v3(x, 0.0, -2.0), 0.9), meas(v3(-x, 0.0, -2.4), 0.9)], t as f64, always);
        }
        assert!(two.tracks.len() == 2 && two.tracks[0].v.x > 0.5 && two.tracks[1].v.x < -0.5);
        two.update(&[], 2.75 + MAX_UNSEEN + 0.1, always);
        assert!(two.tracks.is_empty());
    }

    #[test]
    fn out_of_view_tracks_wait_to_be_found_again() {
        // Missing while out of view doesn't count; after MAX_AGE it's dropped anyway. Found again after a while, its
        // motion starts over from the new detection.
        let mut trk = Tracker::default();
        for i in 0..4 {
            trk.update(&[meas(v3(0.25 * i as f32, 0.0, -3.0), 0.9)], i as f64 * 0.25, always);
        }
        let never = |_: V3| false;
        trk.update(&[], 3.0, never);
        assert_eq!(trk.tracks.len(), 1, "kept while out of view");
        trk.update(&[meas(v3(1.0, 0.0, -3.0), 0.9)], 3.25, always);
        let tr = &trk.tracks[0];
        assert!(tr.v == V3::default() && tr.history.len() == 1 && tr.p == v3(1.0, 0.0, -3.0), "restarted");
        trk.update(&[], 3.25 + MAX_AGE + 0.1, never);
        assert!(trk.tracks.is_empty(), "dropped after MAX_AGE");
    }

    #[test]
    fn indistinguishable_tracks_merge() {
        let mut trk = Tracker::default();
        let m = meas(v3(0.0, 0.0, -5.0), 0.9);
        trk.tracks = vec![Track::new(1, &m, 0.0), Track::new(2, &meas(v3(0.0, 0.0, -5.5), 0.9), 0.0)];
        (trk.tracks[0].hits, trk.tracks[1].hits) = (5, 3);
        trk.update(&[], 0.1, always);
        assert!(trk.tracks.len() == 1 && trk.tracks[0].id == 1);
    }

    #[test]
    fn detections_of_one_object_merge() {
        // Along the ray at 6 m (within depth noise): one; side by side (outside bearing noise): two.
        let a = meas(v3(0.0, 0.0, -6.0), 0.9);
        assert_eq!(merge_detections(&[a, meas(v3(0.0, 0.0, -6.5), 0.5)]).len(), 1);
        assert_eq!(merge_detections(&[a, meas(v3(0.6, 0.0, -6.0), 0.5)]).len(), 2);
        assert_eq!(merge_detections(&[Meas { weak: true, ..a }, meas(v3(0.0, 0.0, -6.5), 0.5)])[0].p.z, -6.5);
    }

    #[test]
    fn size_from_the_filtered_distance() {
        // A depth jump scales the box's metric size with it (same angular size); the track's size follows the filtered
        // distance instead, so it barely changes.
        let mut trk = Tracker::default();
        for i in 0..8 {
            trk.update(&[meas(v3(0.0, 0.0, -4.0), 0.9)], i as f64 * 0.25, always);
        }
        let far = Meas { w: 0.5 * 5.0 / 4.0, h: 1.7 * 5.0 / 4.0, ..meas(v3(0.0, 0.0, -5.0), 0.9) };
        trk.update(&[far], 2.0, always);
        let (w, h) = (trk.tracks[0].w, trk.tracks[0].h);
        assert!((w - 0.5).abs() < 0.05 && (h - 1.7).abs() < 0.15, "{w} {h}");
        // Clipped sides aren't learned from.
        let cut = Meas { w: 0.1, clip_w: true, ..meas(v3(0.0, 0.0, -4.0), 0.9) };
        trk.update(&[cut], 2.25, always);
        assert!((trk.tracks[0].w - w).abs() < 1e-6);
    }

    #[test]
    fn class_needs_a_clear_majority_to_change() {
        let mut trk = Tracker::default();
        let dog = |m: Meas| Meas { cls: 16, ..m };
        for i in 0..8 {
            trk.update(&[meas(v3(0.0, 0.0, -3.0), 0.8)], i as f64 * 0.25, always);
        }
        trk.update(&[dog(meas(v3(0.0, 0.0, -3.0), 0.8))], 2.0, always);
        assert_eq!(trk.tracks[0].cls, 0, "one dog frame doesn't flip a person");
        for i in 0..8 {
            trk.update(&[dog(meas(v3(0.0, 0.0, -3.0), 0.8))], 2.25 + i as f64 * 0.25, always);
        }
        assert_eq!(trk.tracks[0].cls, 16, "a steady dog does");
    }

    #[test]
    fn smoothing_settles_by_time() {
        // 4 Hz settles 90% in 4 samples.
        assert!(((1.0 - alpha(0.25)).powi(4) - 0.1).abs() < 1e-5);
    }
}
