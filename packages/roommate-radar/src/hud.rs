//! Chevrons at the left and right of your view, head-locked like a HUD: people and pets beside you, seen by a
//! side camera or only at the edge of the SLAM views, where there's no box to show (main.rs). One per side, for the
//! most confident of what's seen there, in its class's color (draw::class_rgb), as for the boxes. None while there's a
//! box on that side, or a track about to be one, nor for LINGER_S after (a side camera's range, from the floor alone, is
//! too rough to tell whether it's the same object; and boxes come and go). It sits at the height of its elevation from your head: the angle above or below its horizontal plane, every frame from where it was seen in
//! the room, so it follows head roll (which moves someone beside you up or down) and not pitch (which doesn't).
//! Sightings come 2-4 times a second, so the chevron follows a smoothed position (TAU_S).
use crate::draw;
use crate::measure::{name, to_head};
use crate::openvr::{HmdMatrix34_t, VROverlayHandle_t, vr};
use crate::textures::Textures;
use crate::v3::{V3, v3};

const HOLD_S: f64 = 1.0; // shown this long after the last sighting: the side cameras run at half the detector's rate
const FADE_S: f64 = 0.3; // then fades out over this long
const TAU_S: f32 = 0.2; // a chevron's position follows its object's sightings with this time constant (s)
const DIST: f32 = 0.6; // in front of the eyes (m)
const ANGLE_DEG: f32 = 35.0; // out from straight ahead: in view (SteamVR's view and lens outline both put it past 54)
const MAX_ELEVATION_DEG: f32 = 30.0; // up or down at most, from straight ahead (~25 deg of elevation at ANGLE_DEG out)
const WIDTH: f32 = 0.05; // m; the texture is 2:3
const LINGER_S: f64 = 0.5; // no chevron on a side for this long after a box was there
const SAME_M: f32 = 1.0; // a sighting of the same class this close (horizontally) to an earlier one is the same object

/// Something seen beside you.
struct Mark {
    side: usize, // 0 left, 1 right
    cls: usize,
    score: f32,  // its last sighting's confidence
    p: V3,       // and where (standing universe)
    shown_p: V3, // where its chevron is for, following p
    last: f64,   // when it was last seen (s)
}

pub struct Hud {
    ovs: [VROverlayHandle_t; 2], // left, right
    made: [Option<usize>; 2],    // the class each overlay's texture is for
    shown: [bool; 2],
    marks: Vec<Mark>,
    base: [V3; 2],      // each side's chevron at eye level (head frame)
    max_tan: f32,       // how far down and up (tangent) the chevrons can go and stay in view
    last_show: f64,     // s
    boxed_at: [f64; 2], // when each side last had a box (s)
}

impl Hud {
    pub fn create() -> Result<Hud, String> {
        let mut ovs = [0; 2];
        for (k, ov) in ovs.iter_mut().enumerate() {
            let key = format!("steamframe.roommate-radar.chevron{k}");
            *ov = vr().create_overlay(&key, "roommate-radar").map_err(|e| format!("CreateOverlay {key}: {e}"))?;
            vr().set_width(*ov, WIDTH);
        }
        let x = ANGLE_DEG.to_radians().tan() * DIST;
        Ok(Hud::new(ovs, [v3(-x, 0.0, -DIST), v3(x, 0.0, -DIST)], MAX_ELEVATION_DEG.to_radians().tan()))
    }

    fn new(ovs: [VROverlayHandle_t; 2], base: [V3; 2], max_tan: f32) -> Hud {
        let boxed_at = [f64::NEG_INFINITY; 2];
        Hud { ovs, made: [None; 2], shown: [false; 2], marks: Vec::new(), base, max_tan, last_show: 0.0, boxed_at }
    }

    /// Something of class cls on side k (0 left, 1 right), with confidence score, at p (standing universe) at time t
    /// (s); `why` for the log.
    pub fn saw(&mut self, k: usize, cls: usize, score: f32, p: V3, t: f64, why: &str) {
        let same = |m: &&mut Mark| m.side == k && m.cls == cls && (m.p.x - p.x).hypot(m.p.z - p.z) < SAME_M;
        if let Some(m) = self.marks.iter_mut().find(same) {
            if t >= m.last {
                (m.p, m.score, m.last) = (p, score, t);
            }
            return;
        }
        println!("roommate-radar: seen {} {} {:.0}% ({why})", ["left", "right"][k], name(cls), score * 100.0);
        self.marks.push(Mark { side: k, cls, score, p, shown_p: p, last: t });
    }

    /// The mark side k's chevron is for: the most confident there.
    fn best(&self, k: usize) -> Option<&Mark> {
        self.marks.iter().filter(|m| m.side == k).max_by(|a, b| a.score.total_cmp(&b.score))
    }

    /// The chevron's height (m, head frame) on side k for something at q (head frame): seen from the eyes, at its
    /// elevation, but in view (max_tan: up and down from straight ahead).
    fn height(&self, k: usize, q: V3) -> f32 {
        let b = self.base[k];
        let y = q.y / q.x.hypot(q.z) * b.x.hypot(b.z);
        let lim = self.max_tan * -b.z;
        y.clamp(-lim, lim)
    }

    /// The chevrons at time now (s), for the head pose now; none on a side with a box (`boxes`: the boxes drawn now, and
    /// the tracks that may soon be, standing universe).
    pub fn show(&mut self, now: f64, head: &HmdMatrix34_t, boxes: &[V3], textures: &mut Textures) {
        let a = 1.0 - (-((now - self.last_show).clamp(0.0, 0.1) as f32) / TAU_S).exp();
        self.last_show = now;
        self.marks.retain(|m| now - m.last < HOLD_S + FADE_S);
        for m in &mut self.marks {
            m.shown_p = m.shown_p + (m.p - m.shown_p) * a;
        }
        for &b in boxes {
            self.boxed_at[(to_head(head, b).x > 0.0) as usize] = now;
        }
        for (k, boxed) in self.boxed_at.map(|t| now - t < LINGER_S).into_iter().enumerate() {
            let ov = self.ovs[k];
            let best =
                self.best(k).filter(|_| !boxed).map(|m| (m.cls, m.last, self.height(k, to_head(head, m.shown_p))));
            if let Some((cls, last, y)) = best {
                if self.made[k] != Some(cls) {
                    textures.set(ov, &draw::chevron(128, 192, k == 0, cls));
                    self.made[k] = Some(cls);
                }
                vr().set_alpha(ov, (1.0 - (now - last - HOLD_S) / FADE_S).clamp(0.0, 1.0) as f32);
                let b = self.base[k];
                let t = HmdMatrix34_t { m: [[1.0, 0.0, 0.0, b.x], [0.0, 1.0, 0.0, b.y + y], [0.0, 0.0, 1.0, b.z]] };
                vr().set_transform_hmd(ov, &t);
            }
            let on = best.is_some();
            if on != self.shown[k] {
                if on {
                    vr().show(ov)
                } else {
                    vr().hide(ov)
                }
                self.shown[k] = on;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hud() -> Hud {
        // Chevrons ~50 deg out, going 45 deg up and down from straight ahead (33 deg of elevation)
        Hud::new([0; 2], [v3(-0.7, 0.0, -DIST), v3(0.7, 0.0, -DIST)], 1.0)
    }

    #[test]
    fn chevron_at_their_elevation() {
        // Someone 2 m to your right, their middle 0.5 m below your eyes: the chevron is seen at the same ~14 deg down.
        // Level with you, it's level; far above or below, it stays in view.
        let h = hud();
        let elev = |y: f32| y.atan2(0.7f32.hypot(DIST));
        assert!((elev(h.height(1, v3(2.0, -0.5, 0.0))) - (-0.5f32).atan2(2.0)).abs() < 1e-5);
        assert!(h.height(0, v3(-2.0, 0.0, -1.0)).abs() < 1e-6);
        assert!((h.height(1, v3(0.5, -3.0, 0.0)) + DIST).abs() < 1e-5);
    }

    #[test]
    fn most_confident() {
        // On the right: a person seen twice (one object, its newest confidence), a more confident dog beside them,
        // someone 3 m away. The chevron is the dog's. The left has its own.
        let mut h = hud();
        h.saw(1, 0, 0.6, v3(2.0, 1.0, 0.0), 1.0, "test");
        h.saw(1, 0, 0.7, v3(2.3, 1.0, 0.2), 1.5, "test");
        h.saw(1, 16, 0.8, v3(2.0, 0.3, 0.5), 1.5, "test");
        h.saw(1, 0, 0.5, v3(2.0, 1.0, 3.0), 1.5, "test");
        h.saw(0, 0, 0.55, v3(-2.0, 1.0, 0.0), 1.5, "test");
        assert_eq!(h.marks.len(), 4);
        assert!(h.marks[0].score == 0.7 && (h.marks[0].p.x - 2.3).abs() < 1e-6);
        assert_eq!(h.best(1).map(|m| m.cls), Some(16));
        assert_eq!(h.best(0).map(|m| m.score), Some(0.55));
    }
}
