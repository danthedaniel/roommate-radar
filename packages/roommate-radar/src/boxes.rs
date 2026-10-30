//! The tracks as OpenVR overlays: per confirmed track a box (rounded outline) and a label (class; confidence in
//! developer builds), upright sprites turned to face you every compositor frame, in the standing universe. Each box
//! follows its track's filter states (position and velocity, tracker.rs) played back behind the newest one, so there
//! is always a segment to be in, and fades in and out rather than popping.
use std::collections::VecDeque;

use crate::draw;
use crate::measure::{name, position};
use crate::openvr::{HmdMatrix34_t, VROverlayHandle_t, vr};
use crate::textures::Textures;
use crate::tracker::{MIN_HITS, SHOW_S, Sample, Track, Tracker};
use crate::v3::V3;

const MAX: usize = 8; // overlays of each kind
const LABEL_H: f32 = 0.07; // label height (m), just above the box

/// Upright quad at c facing the head position h (overlay +z points at the viewer).
fn facing(c: V3, h: V3) -> HmdMatrix34_t {
    let (mut zx, mut zz) = (h.x - c.x, h.z - c.z);
    let n = zx.hypot(zz);
    (zx, zz) = if n < 1e-4 { (0.0, 1.0) } else { (zx / n, zz / n) };
    HmdMatrix34_t { m: [[zz, 0.0, zx, c.x], [0.0, 1.0, 0.0, c.y], [-zx, 0.0, zz, c.z]] }
}

/// Coasting past the newest state (a missed detection), the velocity fades out over about this long (s), so a box
/// drifts at most this times its speed beyond the last measurement.
const COAST_S: f32 = 0.15;
const FADE_IN_S: f64 = 0.25; // a box fades in over this long (s) when it's confirmed or found again
const FADE_OUT_S: f64 = 0.4; // and out over this long once its track's last match is SHOW_S old

/// The track's box (center, width, height) at time t (s): a cubic Hermite between the two filter states around t,
/// their velocities as tangents (smooth, and exact for steady motion, Gaffer On Games' snapshot interpolation). Before
/// the first state, that state; past the last, coasting on with its velocity fading out over COAST_S.
fn sample_at(s: &VecDeque<Sample>, t: f64) -> (V3, f32, f32) {
    let (first, last) = (s[0], s[s.len() - 1]);
    if t <= first.t {
        return (first.p, first.w, first.h);
    }
    if t >= last.t {
        let k = COAST_S * (1.0 - (-(t - last.t) as f32 / COAST_S).exp());
        return (last.p + last.v * k, last.w, last.h);
    }
    let i = (0..s.len() - 1).find(|&i| s[i + 1].t > t).unwrap();
    let (a, b) = (s[i], s[i + 1]);
    let dt = (b.t - a.t) as f32;
    let u = (t - a.t) as f32 / dt;
    let (u2, u3) = (u * u, u * u * u);
    let p = a.p * (2.0 * u3 - 3.0 * u2 + 1.0)
        + a.v * ((u3 - 2.0 * u2 + u) * dt)
        + b.p * (-2.0 * u3 + 3.0 * u2)
        + b.v * ((u3 - u2) * dt);
    (p, a.w + u * (b.w - a.w), a.h + u * (b.h - a.h))
}

/// How visible a track's box is at time now (s): fading in from when it was confirmed (or found again), out once its
/// last match is SHOW_S old. 0 for tentative tracks.
fn opacity(t: &Track, now: f64) -> f32 {
    if t.hits < MIN_HITS {
        return 0.0;
    }
    let fade_in = (now - t.shown_from) / FADE_IN_S;
    let fade_out = 1.0 - (now - t.last_seen - SHOW_S) / FADE_OUT_S;
    fade_in.min(fade_out).clamp(0.0, 1.0) as f32
}

pub struct Boxes {
    outlines: [VROverlayHandle_t; MAX],
    labels: [VROverlayHandle_t; MAX],
    made: [Option<(usize, usize)>; MAX], // what each box overlay's texture was made for: (height, class)
    label_text: [String; MAX],
    shown: usize,
    last_log_ns: u64,
    pub drawn: Vec<V3>, // where the boxes shown now are (standing universe)
}

impl Boxes {
    pub fn create() -> Result<Boxes, String> {
        let create = |kind: &str| -> Result<[VROverlayHandle_t; MAX], String> {
            let mut ovs = [0; MAX];
            for (i, ov) in ovs.iter_mut().enumerate() {
                let key = format!("steamframe.roommate-radar.{kind}{i}");
                *ov = vr().create_overlay(&key, "roommate-radar").map_err(|e| format!("CreateOverlay {key}: {e}"))?;
            }
            Ok(ovs)
        };
        Ok(Boxes {
            outlines: create("box")?,
            labels: create("label")?,
            made: [None; MAX],
            label_text: Default::default(),
            shown: 0,
            last_log_ns: 0,
            drawn: Vec::new(),
        })
    }

    /// Shows the tracker's confirmed tracks at playback time t_play (s) to the head at `head`.
    pub fn show(&mut self, tracker: &Tracker, head: &HmdMatrix34_t, now: u64, t_play: f64, textures: &mut Textures) {
        let h = position(head);
        let now_s = now as f64 / 1e9;
        let mut log = String::new(); // each shown box relative to the head
        // In the order they were found: a new track takes the next overlay, and the later ones move up one when one
        // goes.
        let mut order: Vec<&Track> = tracker.tracks.iter().filter(|t| opacity(t, now_s) > 0.0).collect();
        order.sort_by_key(|t| t.id);
        let mut n = 0;
        self.drawn.clear();
        for t in order.into_iter().take(MAX) {
            let alpha = opacity(t, now_s);
            let (c, w, ht) = sample_at(&t.history, t_play);
            let (w, ht) = (w.max(0.05), ht.max(0.05));
            self.drawn.push(c);
            // Bearing from the head's horizontal forward (-z), + = right; height relative to the eyes.
            let (vx, vz, fx, fz) = (c.x - h.x, c.z - h.z, -head.m[0][2], -head.m[2][2]);
            log += &format!(
                " | {} {:.0}% {:.1}m {:+.0}deg {:+.1}m",
                name(t.cls),
                t.score * 100.0,
                vx.hypot(vz),
                (fx * vz - fz * vx).atan2(fx * vx + fz * vz).to_degrees(),
                c.y - h.y
            );
            // 256 texels across; height in steps of 8 so the texture is only redrawn when the aspect really changes
            let pw = 256;
            let ph = ((pw as f32 * ht / w / 8.0).round() as usize * 8).clamp(32, 1024);
            if self.made[n] != Some((ph, t.cls)) {
                let img = draw::outline(pw, ph, t.cls, (pw as f32 * 0.008 / w).clamp(1.5, 8.0)); // ~8 mm line
                textures.set(self.outlines[n], &img);
                self.made[n] = Some((ph, t.cls));
            }
            vr().set_width(self.outlines[n], w);
            vr().set_alpha(self.outlines[n], alpha);
            vr().set_alpha(self.labels[n], alpha);
            let mut m = facing(c, h);
            vr().set_transform(self.outlines[n], &m);

            let pct = cfg!(feature = "developer").then(|| (t.score * 100.0).round() as i32); // developer builds only
            let text = format!("{} {pct:?}", name(t.cls));
            if self.label_text[n] != text {
                let img = draw::label(name(t.cls), pct, t.cls);
                textures.set(self.labels[n], &img);
                vr().set_width(self.labels[n], LABEL_H * img.w as f32 / img.h as f32);
                self.label_text[n] = text;
            }
            m.m[1][3] += ht / 2.0 + LABEL_H / 2.0 + 0.015; // same sprite plane, just above the box
            vr().set_transform(self.labels[n], &m);
            if n >= self.shown {
                vr().show(self.outlines[n]);
                vr().show(self.labels[n]);
            }
            n += 1;
        }
        for i in n..self.shown {
            vr().hide(self.outlines[i]);
            vr().hide(self.labels[i]);
        }
        if n != self.shown || (n > 0 && now - self.last_log_ns > 2_000_000_000) {
            println!("roommate-radar: showing {n}{log}");
            self.last_log_ns = now;
        }
        self.shown = n;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v3::v3;

    #[test]
    fn quads_face_the_head() {
        // Box 2 m ahead of the head: a quad facing the head has +z toward it, and x right.
        let m = facing(v3(0.0, 1.0, -2.0), v3(0.0, 1.6, 0.0)).m;
        assert!((m[2][2] - 1.0).abs() < 1e-5 && (m[0][0] - 1.0).abs() < 1e-5);
        let m = facing(v3(2.0, 1.0, 0.0), v3(0.0, 1.6, 0.0)).m; // box to the right: +z points to -x
        assert!((m[0][2] + 1.0).abs() < 1e-5 && (m[1][1] - 1.0).abs() < 1e-5);
    }

    fn sample(t: f64, x: f32, vx: f32) -> Sample {
        Sample { t, p: v3(x, 1.0, -2.0), v: v3(vx, 0.0, 0.0), w: 0.5, h: 1.7 }
    }

    #[test]
    fn playback() {
        // Passes through the states, exact for steady motion even with uneven spacing; coasts a little past the last.
        let s: VecDeque<Sample> = [0.0, 0.25, 0.45, 0.75].map(|t| sample(t, t as f32, 1.0)).into();
        assert!((sample_at(&s, 0.45).0.x - 0.45).abs() < 1e-5);
        let worst =
            (0..=75).map(|i| (sample_at(&s, i as f64 / 100.0).0.x - i as f32 / 100.0).abs()).fold(0.0, f32::max);
        assert!(worst < 1e-4);
        assert!(sample_at(&s, -1.0).0.x == 0.0);
        let coast = |t: f64| sample_at(&s, t).0.x - 0.75;
        assert!(coast(0.8) > 0.03 && coast(0.8) < 0.05, "keeps moving at first");
        assert!((coast(5.0) - COAST_S).abs() < 1e-4, "then stops, COAST_S * speed on");
        // Stopping: no overshoot past the stop between states that say so.
        let stop: VecDeque<Sample> = [sample(0.0, 0.0, 1.0), sample(0.25, 0.25, 0.0)].into();
        assert!((0..=25).all(|i| sample_at(&stop, i as f64 / 100.0).0.x <= 0.25 + 1e-6));
    }

    #[test]
    fn fades() {
        let mut t = Track::for_test(10.0);
        assert_eq!(opacity(&t, 10.0), 0.0, "tentative");
        t.hits = MIN_HITS;
        let at = |dt: f64| opacity(&t, 10.0 + dt);
        assert!(at(0.0) == 0.0 && (at(FADE_IN_S / 2.0) - 0.5).abs() < 1e-5 && at(0.5) == 1.0, "fades in");
        assert!(at(SHOW_S) == 1.0 && at(SHOW_S + FADE_OUT_S / 2.0) < 0.51 && at(SHOW_S + FADE_OUT_S) < 1e-5, "and out");
    }
}
