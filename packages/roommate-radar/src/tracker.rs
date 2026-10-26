//! ByteTrack-style multi-object tracker in 3D. Each track is a constant-velocity Kalman filter over [x, y, z, w, h]
//! (box center in the standing universe and box size, meters); axes are independent, so it's five 2-state filters
//! (position, velocity) rather than one 10-state one. Per detector frame:
//!   1. predict every track to the frame's capture time,
//!   2. match tracks to confident detections (Hungarian, cost = 3D center distance, gated),
//!   3. match the tracks left over to low-confidence detections (keeps a track through a weak frame),
//!   4. unmatched confident detections start tentative tracks; tentative tracks that miss are dropped, confirmed
//!      ones after max_age without a match.
//!
//! Each track also keeps its last few matched detections (history) for drawing, with w, h and the distance from the
//! eye alpha-smoothed; the filter's predictions are only used for matching.
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug)]
pub struct Meas {
    pub v: [f32; 5], // x, y, z, w, h (m)
    pub sigma: f32,  // position measurement std dev (m); stereo depth gets worse with distance
    pub score: f32,
    pub cls: usize,
    pub eye: [f32; 3], // head position at capture: distance is smoothed along the ray from it
}

#[derive(Clone, Debug)]
pub struct Params {
    pub high: f32,     // detections at least this confident can start tracks (step 2); the rest only extend (3)
    pub gate: f32,     // max 3D distance (m) between a track's prediction and a detection to match them
    pub max_age: f32,  // drop a track this long (s) after its last match
    pub min_hits: u32, // matches before a track is confirmed (shown)
    pub accel: f32,    // process noise: std dev of a person's acceleration (m/s^2)
    pub size_rate: f32, // process noise for box size (m/s^2 equivalent)
    pub size_sigma: f32, // box size measurement std dev (m)
    pub history: usize, // matched detections kept per track
    pub size_alpha: f32, // w, h in history are exponentially smoothed (1 = raw); the spline alone follows noise
    pub dist_alpha: f32, // same for the distance from the eye (stereo depth); the bearing stays raw
}

impl Default for Params {
    fn default() -> Params {
        Params {
            high: 0.5,
            gate: 0.7,
            max_age: 1.0,
            min_hits: 2,
            accel: 2.0,
            size_rate: 0.3,
            size_sigma: 0.1,
            history: 4,
            size_alpha: 0.2,
            dist_alpha: 0.4,
        }
    }
}

/// One axis: position p, velocity v, covariance pc.
#[derive(Clone, Copy, Debug, Default)]
pub struct Axis {
    pub p: f32,
    pub v: f32,
    pc: [[f32; 2]; 2],
}

impl Axis {
    fn new(z: f32, r: f32, v_var: f32) -> Axis {
        Axis { p: z, v: 0.0, pc: [[r * r, 0.0], [0.0, v_var]] }
    }

    /// Constant velocity with white-noise acceleration of std dev a.
    fn predict(&mut self, dt: f32, a: f32) {
        let pc = &mut self.pc;
        self.p += self.v * dt;
        let q = a * a;
        let p00 = pc[0][0] + dt * (pc[0][1] + pc[1][0]) + dt * dt * pc[1][1];
        let p01 = pc[0][1] + dt * pc[1][1];
        pc[0][0] = p00 + q * dt * dt * dt / 3.0;
        pc[0][1] = p01 + q * dt * dt / 2.0;
        pc[1][0] = pc[0][1];
        pc[1][1] += q * dt;
    }

    fn update(&mut self, z: f32, r: f32) {
        let pc = &mut self.pc;
        let s = pc[0][0] + r * r;
        let (k0, k1, y) = (pc[0][0] / s, pc[1][0] / s, z - self.p);
        self.p += k0 * y;
        self.v += k1 * y;
        let (p00, p01, p11) = ((1.0 - k0) * pc[0][0], (1.0 - k0) * pc[0][1], pc[1][1] - k1 * pc[0][1]);
        *pc = [[p00, p01], [p01, p11]];
    }
}

/// One matched detection: capture time (s) and x, y, z, w, h.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub v: [f32; 5],
}

#[derive(Clone, Debug)]
pub struct Track {
    pub axis: [Axis; 5],
    pub t: f64,         // time the filter state is at (s)
    pub last_seen: f64, // time of the last matched detection (s)
    pub hits: u32,
    pub cls: usize,
    pub score: f32,
    pub confirmed: bool,           // reached min_hits; stays set while the track lives
    pub history: VecDeque<Sample>, // last prm.history matched detections, oldest first
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

#[derive(Default)]
pub struct Tracker {
    pub prm: Params,
    pub tracks: Vec<Track>,
}

impl Tracker {
    /// One detector frame captured at time t (s).
    pub fn update(&mut self, dets: &[Meas], t: f64) {
        for tr in &mut self.tracks {
            let dt = (t - tr.t).max(0.0) as f32;
            for (k, ax) in tr.axis.iter_mut().enumerate() {
                ax.predict(dt, if k < 3 { self.prm.accel } else { self.prm.size_rate });
            }
            tr.t = tr.t.max(t);
        }
        let (high, low): (Vec<usize>, Vec<usize>) = (0..dets.len()).partition(|&i| dets[i].score >= self.prm.high);

        let mut matched = vec![false; self.tracks.len()];
        let unused_high = self.match_dets(dets, &high, &mut matched, t);
        self.match_dets(dets, &low, &mut matched, t);
        for i in unused_high {
            let d = &dets[i];
            let mut tr = Track {
                axis: std::array::from_fn(|k| {
                    Axis::new(d.v[k], if k < 3 { d.sigma } else { self.prm.size_sigma }, if k < 3 { 1.0 } else { 0.1 })
                }),
                t,
                last_seen: t,
                hits: 1,
                cls: d.cls,
                score: d.score,
                confirmed: self.prm.min_hits <= 1,
                history: VecDeque::new(),
            };
            remember(&self.prm, &mut tr, d, t);
            self.tracks.push(tr);
            matched.push(true);
        }
        let max_age = self.prm.max_age as f64;
        let mut i = 0;
        self.tracks.retain(|tr| {
            let keep = (tr.confirmed || matched[i]) && t - tr.last_seen <= max_age;
            i += 1;
            keep
        });
    }

    /// Matches still-unmatched tracks to dets[which]; updates the matched tracks. Returns the unmatched detections.
    fn match_dets(&mut self, dets: &[Meas], which: &[usize], matched: &mut [bool], t: f64) -> Vec<usize> {
        let rows: Vec<usize> = (0..self.tracks.len()).filter(|&i| !matched[i]).collect();
        let cost: Vec<Vec<f32>> = rows
            .iter()
            .map(|&r| {
                let a = &self.tracks[r].axis;
                which.iter().map(|&c| (0..3).map(|k| (a[k].p - dets[c].v[k]).powi(2)).sum::<f32>().sqrt()).collect()
            })
            .collect();
        let m = assign(&cost, self.prm.gate);
        let mut used = vec![false; which.len()];
        for (r, col) in m.into_iter().enumerate() {
            let Some(col) = col else {
                continue;
            };
            let d = &dets[which[col]];
            let tr = &mut self.tracks[rows[r]];
            for k in 0..5 {
                tr.axis[k].update(d.v[k], if k < 3 { d.sigma } else { self.prm.size_sigma });
            }
            (tr.last_seen, tr.cls, tr.score) = (t, d.cls, d.score);
            remember(&self.prm, tr, d, t);
            tr.hits += 1;
            if tr.hits >= self.prm.min_hits {
                tr.confirmed = true;
            }
            matched[rows[r]] = true;
            used[col] = true;
        }
        (0..which.len()).filter(|&c| !used[c]).map(|c| which[c]).collect()
    }
}

/// Adds detection d (time t) to the track's history.
fn remember(prm: &Params, tr: &mut Track, d: &Meas, t: f64) {
    let mut s = Sample { t, v: d.v };
    if let Some(prev) = tr.history.back() {
        for c in 3..5 {
            s.v[c] = prev.v[c] + prm.size_alpha * (s.v[c] - prev.v[c]);
        }
        // Previous distance measured from this eye, so the head moving isn't smoothed away.
        let dist = |v: &[f32; 5]| (0..3).map(|c| (v[c] - d.eye[c]).powi(2)).sum::<f32>().sqrt();
        let (r, rp) = (dist(&s.v), dist(&prev.v));
        if r > 0.0 {
            let k = (rp + prm.dist_alpha * (r - rp)) / r;
            for c in 0..3 {
                s.v[c] = d.eye[c] + k * (s.v[c] - d.eye[c]);
            }
        }
    }
    if tr.history.back().is_none_or(|b| t > b.t) {
        tr.history.push_back(s);
    }
    while tr.history.len() > prm.history {
        tr.history.pop_front();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meas(v: [f32; 5], score: f32) -> Meas {
        Meas { v, sigma: 0.05, score, cls: 0, eye: [0.0; 3] }
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
    fn walking_person() {
        // At 1 m/s along x, detected at 4 Hz with depth noise: confirmed on the 2nd frame, velocity learned, the last
        // 4 detections kept; then a weak detection extends the track but starts nothing on its own.
        let mut trk = Tracker::default();
        let mut seed = 1u32;
        let mut noise = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            ((seed >> 9) as i32 % 2001 - 1000) as f32 / 1000.0 * 0.05
        };
        for i in 0..12 {
            let t = i as f64 * 0.25;
            trk.update(&[meas([t as f32 + noise(), 1.0, -2.0 + noise(), 0.5, 1.7], 0.8)], t);
            if i == 0 {
                assert!(trk.tracks.len() == 1 && !trk.tracks[0].confirmed, "new track is tentative");
            }
            if i == 1 {
                assert!(trk.tracks.len() == 1 && trk.tracks[0].confirmed, "second hit confirms it");
            }
        }
        let tr = &trk.tracks[0];
        assert!(trk.tracks.len() == 1 && (tr.axis[0].v - 1.0).abs() < 0.25);
        assert!(tr.history.len() == 4 && tr.history[0].t == 8.0 * 0.25 && tr.history[3].t == 11.0 * 0.25);

        trk.update(&[meas([3.0, 1.0, -2.0, 0.5, 1.7], 0.2), meas([-3.0, 1.0, -2.0, 0.5, 1.7], 0.2)], 3.0);
        assert!(trk.tracks.len() == 1 && trk.tracks[0].last_seen == 3.0);
    }

    #[test]
    fn size_and_distance_alpha_smoothed() {
        // A 20 cm jump in height moves the stored height by size_alpha of it.
        let mut sz = Tracker::default();
        sz.update(&[meas([0.0, 1.0, -2.0, 0.5, 1.7], 0.8)], 0.0);
        sz.update(&[meas([0.0, 1.0, -2.0, 0.5, 1.9], 0.8)], 0.25);
        let last = *sz.tracks[0].history.back().unwrap();
        assert!((last.v[4] - (1.7 + sz.prm.size_alpha * 0.2)).abs() < 1e-5 && last.v[3] == 0.5);
        // Distance too, along the ray from the eye: 2 m then 2.5 m straight ahead stores 2 + dist_alpha * 0.5.
        sz.update(&[Meas { eye: [0.0, 1.0, 0.0], ..meas([0.0, 1.0, -2.5, 0.5, 1.7], 0.8) }], 0.5);
        let last = *sz.tracks[0].history.back().unwrap();
        assert!((last.v[2] + 2.0 + sz.prm.dist_alpha * 0.5).abs() < 1e-5 && (last.v[1] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn crossing_tracks_stay_apart_then_expire() {
        // Two people crossing paths keep their own tracks (prediction + optimal assignment), then both expire.
        let mut two = Tracker::default();
        for i in 0..12 {
            let t = i as f32 * 0.25;
            let x = -1.5 + t; // A moves +x, B moves -x; they pass at t = 1.5
            two.update(&[meas([x, 1.0, -2.0, 0.5, 1.7], 0.9), meas([-x, 1.0, -2.4, 0.5, 1.7], 0.9)], t as f64);
        }
        assert!(two.tracks.len() == 2 && two.tracks[0].axis[0].v > 0.5 && two.tracks[1].axis[0].v < -0.5);
        two.update(&[], 2.75 + 1.1);
        assert!(two.tracks.is_empty());
    }
}
