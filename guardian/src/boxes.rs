//! The tracks as OpenVR overlays: per confirmed track a box (rounded outline) and a label (class, confidence), upright
//! sprites turned to face you every compositor frame, in the standing universe. Each box's center and size follow a
//! cubic spline (time-based Catmull-Rom) through its track's last detections, played back behind the newest one so
//! there is always a segment to be in.
use std::collections::VecDeque;

use crate::detector::{COCO, Options};
use crate::draw;
use crate::measure::position;
use crate::openvr::{HmdMatrix34_t, VROverlayHandle_t, vr};
use crate::stereo::Det;
use crate::textures::Textures;
use crate::tracker::{Sample, Tracker};
use crate::v3::{V3, v3};

const MAX: usize = 8; // overlays of each kind
const LABEL_H: f32 = 0.07; // label height (m), just above the box

/// A box (center p, width w, height h; standing universe) as seen from the head position: azimuth/elevation (rad) of
/// its edges, so duplicate() can tell two tracks of one person apart from two people.
fn angular_box(p: V3, w: f32, h: f32, head: V3) -> Det {
    let d = (p.x - head.x).hypot(p.z - head.z);
    let (az, el) = ((p.x - head.x).atan2(head.z - p.z), (p.y - head.y).atan2(d));
    let (hw, hh) = ((w / 2.0).atan2(d), (h / 2.0).atan2(d));
    Det { x0: az - hw, y0: el - hh, x1: az + hw, y1: el + hh, score: 0.0, cls: 0 }
}

/// Upright quad at c facing the head position h (overlay +z points at the viewer).
fn facing(c: V3, h: V3) -> HmdMatrix34_t {
    let (mut zx, mut zz) = (h.x - c.x, h.z - c.z);
    let n = zx.hypot(zz);
    (zx, zz) = if n < 1e-4 { (0.0, 1.0) } else { (zx / n, zz / n) };
    HmdMatrix34_t { m: [[zz, 0.0, zx, c.x], [0.0, 1.0, 0.0, c.y], [-zx, 0.0, zz, c.z]] }
}

/// Cubic Hermite through the samples with Catmull-Rom tangents scaled by the real sample spacing (one-sided at the
/// ends), so uneven detector timing doesn't kink the path. Clamps to the first/last sample outside their span.
fn spline_at(s: &VecDeque<Sample>, t: f64) -> Sample {
    let (first, last) = (s[0], s[s.len() - 1]);
    if t <= first.t {
        return first;
    }
    if t >= last.t {
        return last;
    }
    let i = (0..s.len() - 1).find(|&i| s[i + 1].t > t).unwrap();
    let tangent = |k: usize, c: usize| {
        // per second
        let (a, b) = (k.saturating_sub(1), (k + 1).min(s.len() - 1));
        (s[b].v[c] - s[a].v[c]) / (s[b].t - s[a].t) as f32
    };
    let dt = (s[i + 1].t - s[i].t) as f32;
    let u = (t - s[i].t) as f32 / dt;
    let (u2, u3) = (u * u, u * u * u);
    let mut r = Sample { t, v: [0.0; 5] };
    for c in 0..5 {
        r.v[c] = (2.0 * u3 - 3.0 * u2 + 1.0) * s[i].v[c]
            + (u3 - 2.0 * u2 + u) * dt * tangent(i, c)
            + (-2.0 * u3 + 3.0 * u2) * s[i + 1].v[c]
            + (u3 - u2) * dt * tangent(i + 1, c);
    }
    r
}

pub struct Boxes {
    outlines: [VROverlayHandle_t; MAX],
    labels: [VROverlayHandle_t; MAX],
    made: [Option<(usize, usize)>; MAX], // what each box overlay's texture was made for: (height, class)
    label_text: [String; MAX],
    shown: usize,
    last_log_ns: u64,
}

impl Boxes {
    pub fn create() -> Result<Boxes, String> {
        let create = |kind: &str| -> Result<[VROverlayHandle_t; MAX], String> {
            let mut ovs = [0; MAX];
            for (i, ov) in ovs.iter_mut().enumerate() {
                let key = format!("steamframe.guardian.{kind}{i}");
                *ov = vr().create_overlay(&key, "guardian").map_err(|e| format!("CreateOverlay {key}: {e}"))?;
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
        })
    }

    /// Shows the tracker's confirmed tracks at playback time t_play (s) to the head at `head`; returns how many.
    pub fn show(
        &mut self,
        tracker: &Tracker,
        head: &HmdMatrix34_t,
        now: u64,
        t_play: f64,
        textures: &mut Textures,
        opts: &Options,
    ) -> usize {
        let h = position(head);
        let mut log = String::new(); // each shown box relative to the head
        // Most recently matched first: when a depth jump misses the tracker's gate, the new track takes over and the
        // old one, still coasting, would be drawn as a second box just in front of or behind it.
        let mut order: Vec<_> = tracker.tracks.iter().collect();
        order.sort_by(|x, y| y.last_seen.total_cmp(&x.last_seen).then(y.score.total_cmp(&x.score)));
        let mut drawn: Vec<Det> = Vec::new(); // angular boxes (rad) seen from the head, for duplicate()
        let mut n = 0;
        for t in order {
            // Confirmed and matched recently (the tracker drops stale tracks only when the next detections come).
            if !t.confirmed || now as f64 / 1e9 - t.last_seen > tracker.prm.max_age as f64 || n == MAX {
                continue;
            }
            let p = spline_at(&t.history, t_play).v;
            let (c, w, ht) = (v3(p[0], p[1], p[2]), p[3].max(0.05), p[4].max(0.05));
            let a = angular_box(c, w, ht, h);
            if drawn.iter().any(|b| opts.duplicate(&a, b)) {
                continue;
            }
            drawn.push(a);
            // Bearing from the head's horizontal forward (-z), + = right; height relative to the eyes.
            let (vx, vz, fx, fz) = (c.x - h.x, c.z - h.z, -head.m[0][2], -head.m[2][2]);
            log += &format!(
                " | {} {:.0}% {:.1}m {:+.0}deg {:+.1}m",
                COCO[t.cls],
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
            let mut m = facing(c, h);
            vr().set_transform(self.outlines[n], &m);

            let pct = (t.score * 100.0).round() as i32;
            let text = format!("{} {pct}", COCO[t.cls]);
            if self.label_text[n] != text {
                let img = draw::label(COCO[t.cls], pct, t.cls);
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
            println!("guardian: showing {n}{log}");
            self.last_log_ns = now;
        }
        self.shown = n;
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quads_face_the_head() {
        // Box 2 m ahead of the head: a quad facing the head has +z toward it, and x right.
        let m = facing(v3(0.0, 1.0, -2.0), v3(0.0, 1.6, 0.0)).m;
        assert!((m[2][2] - 1.0).abs() < 1e-5 && (m[0][0] - 1.0).abs() < 1e-5);
        let m = facing(v3(2.0, 1.0, 0.0), v3(0.0, 1.6, 0.0)).m; // box to the right: +z points to -x
        assert!((m[0][2] + 1.0).abs() < 1e-5 && (m[1][1] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn spline() {
        // Passes through the samples, exact for steady motion even with uneven spacing, clamps at the ends.
        let s: VecDeque<Sample> =
            [0.0, 0.25, 0.45, 0.75].map(|t| Sample { t, v: [t as f32, 1.0, -2.0, 0.5, 1.7] }).into();
        assert!((spline_at(&s, 0.45).v[0] - 0.45).abs() < 1e-5);
        let worst =
            (0..=75).map(|i| (spline_at(&s, i as f64 / 100.0).v[0] - i as f32 / 100.0).abs()).fold(0.0, f32::max);
        assert!(worst < 1e-4);
        assert!(spline_at(&s, 0.9).v[0] == 0.75 && spline_at(&s, 0.9).v[4] == 1.7);
    }

    #[test]
    fn one_person_at_two_depths_is_a_duplicate() {
        let (opts, head) = (Options::default(), v3(0.0, 1.6, 0.0));
        let a = angular_box(v3(0.0, 1.2, -2.0), 0.5, 1.6, head);
        assert!(opts.duplicate(&a, &angular_box(v3(0.0, 1.2, -2.5), 0.5, 1.6, head)));
        assert!(!opts.duplicate(&a, &angular_box(v3(0.8, 1.2, -2.0), 0.5, 1.6, head)), "two side by side aren't");
    }
}
