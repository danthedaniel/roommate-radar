//! The detector's objects (head frame at capture) as the tracker's measurements (standing universe), keeping only what
//! roommate-radar shows: a wanted class, in range, not your own arms or hands. Depth from stereo, and from where the
//! box meets the floor (the floor and the head pose are known exactly) where the two agree; boxes seen in one view only
//! get theirs from the floor alone.
use crate::detector::{HFOV_DEG, View};
use crate::openvr::HmdMatrix34_t;
use crate::settings::Settings;
use crate::stereo::Obj;
use crate::tracker::Meas;
use crate::v3::{V3, v3};

/// person, cat, dog
pub fn wanted(cls: usize, set: &Settings) -> bool {
    if cls == 0 { set.people } else { (cls == 15 || cls == 16) && set.pets }
}

/// A wanted class's name.
pub fn name(cls: usize) -> &'static str {
    match cls {
        0 => "person",
        15 => "cat",
        _ => "dog",
    }
}

pub fn apply(m: &HmdMatrix34_t, p: V3) -> V3 {
    let r = |i: usize| m.m[i][0] * p.x + m.m[i][1] * p.y + m.m[i][2] * p.z + m.m[i][3];
    v3(r(0), r(1), r(2))
}

pub fn position(m: &HmdMatrix34_t) -> V3 {
    v3(m.m[0][3], m.m[1][3], m.m[2][3])
}

/// Your own arms and hands (YOLO often calls them "person"; it doesn't know the controller, so an arm box can stop at
/// the wrist and leave it out): a point on the forearm, from 40% of the way from the shoulder to a controller up to the
/// controller, inside the box: within its rectangle scaled to that point's depth (seen from the camera) and grown by
/// 15 cm, and from 40 cm behind its depth to 80 cm in front of it (arm boxes measure too far: cut off by the image edge
/// differently in each view, or merged with what's behind the arm). Box and arms (shoulder, controller) are in the
/// view's frame (measurements()) at capture time.
fn holds_controller(o: &Obj, arms: &[(V3, V3)]) -> bool {
    let m = 0.15;
    let (c, b) = (o.origin, &o.corners);
    arms.iter().any(|&(shoulder, hand)| {
        (4..=10).any(|k| {
            let p = shoulder + (hand - shoulder) * (k as f32 / 10.0);
            let s = (p.z - c.z) / (b[0].z - c.z);
            s > 0.0
                && p.x > c.x + (b[0].x - c.x) * s - m
                && p.x < c.x + (b[1].x - c.x) * s + m
                && p.y > c.y + (b[3].y - c.y) * s - m
                && p.y < c.y + (b[0].y - c.y) * s + m
                && p.z - b[0].z > -0.4
                && p.z - b[0].z < 0.8
        })
    })
}

/// Your own arm or lap, controller or not: the box reaches the bottom 10% of the view, is within ARM_DIST, and its top is
/// below eye level (anyone else that close shows their head). Head frame, like holds_controller.
fn own_arm(o: &Obj) -> bool {
    let b = &o.corners;
    let z = o.origin.z - b[3].z;
    let edge = (HFOV_DEG.to_radians() / 2.0).tan();
    z > 0.0 && z < ARM_DIST && (o.origin.y - b[3].y) / z > 0.8 * edge && b[0].y < o.origin.y
}

/// How close your own arm or lap is, at most (m).
const ARM_DIST: f32 = 1.2;

/// Boxes entirely higher than this above the floor (standing universe y = 0) are dropped: nobody's up there (m).
const MAX_BOTTOM: f32 = 2.0;

/// How far outside the play area's boundary detections are still kept (m).
pub const PLAY_AREA_MARGIN: f32 = 1.0;

/// [x, z] inside the boundary (`walls`, Vr::play_area_walls) or within PLAY_AREA_MARGIN of it; no walls: anywhere.
fn near_play_area(walls: &[[[f32; 2]; 2]], x: f32, z: f32) -> bool {
    let mut inside = walls.is_empty();
    for &[[x0, z0], [x1, z1]] in walls {
        // even-odd: walls crossing the +x ray from the point
        if (z0 > z) != (z1 > z) && x < x0 + (z - z0) / (z1 - z0) * (x1 - x0) {
            inside = !inside;
        }
        let (dx, dz) = (x1 - x0, z1 - z0);
        let s = (((x - x0) * dx + (z - z0) * dz) / (dx * dx + dz * dz).max(1e-9)).clamp(0.0, 1.0);
        if (x - x0 - s * dx).hypot(z - z0 - s * dz) < PLAY_AREA_MARGIN {
            return true;
        }
    }
    inside
}

/// p turned by yaw (rad) about the vertical: + turns -z (ahead) toward -x (left).
pub fn yawed(p: V3, yaw: f32) -> V3 {
    let (s, c) = yaw.sin_cos();
    v3(c * p.x + s * p.z, p.y, c * p.z - s * p.x)
}

/// Pose m turned by yaw about its vertical: its -z is m's yawed(-z, yaw).
fn turned(m: &HmdMatrix34_t, yaw: f32) -> HmdMatrix34_t {
    let (s, c) = yaw.sin_cos();
    let mut t = *m;
    for r in &mut t.m {
        (r[0], r[2]) = (c * r[0] - s * r[2], s * r[0] + c * r[2]);
    }
    t
}

/// The standing universe's p in the head frame of pose m: R^T (p - t).
pub fn to_head(m: &HmdMatrix34_t, p: V3) -> V3 {
    let d = p - position(m);
    let col = |j: usize| m.m[0][j] * d.x + m.m[1][j] * d.y + m.m[2][j] * d.z;
    v3(col(0), col(1), col(2))
}

/// Box center noise across the viewing ray (m): this much, plus CENTER_REL of the box's width (a box's center isn't a
/// fixed point on a body: arms, legs, a turn), plus a pixel at its distance.
const CENTER_M: f32 = 0.02;
const CENTER_REL: f32 = 0.03;
/// A box's bottom edge's noise (px), plus BOTTOM_REL of its height, for floor_depth().
const BOTTOM_PX: f32 = 1.5;
const BOTTOM_REL: f32 = 0.02;
/// How short the box's bottom edge reads the floor contact, as a fraction of the distance (the box's bottom is below
/// the feet or paws), and how far that varies: fitted, 3-9% short beyond 3 m, 1-6 cm scatter.
const CONTACT_REL: f32 = 0.06;
const CONTACT_SPREAD: f32 = 0.03;
/// Floor contact and stereo further apart than this many std devs: not standing on the floor (sitting, behind
/// furniture, cut off at the view's edge), and floor contact is left out.
const CONTACT_GATE: f32 = 2.5;
/// Only boxes whose bottom is at least this far below the horizon (sine of the angle) get a floor-contact depth.
const MIN_DOWN: f32 = 0.05;
/// What the tracker counts as in view (observable()) is this fraction of the view's size in from its edges.
const VIEW_MARGIN: f32 = 0.04;

/// Depth (head frame) at which the box's bottom edge meets the floor (standing universe y = 0), and its std dev. For
/// something standing on the floor with its feet in view, a far better range at a distance than stereo's: the
/// camera's height is a 1.6 m baseline, stereo's is 10.6 cm. Corrected for CONTACT_REL. None if the bottom edge is cut
/// off by the view, or isn't looking down onto the floor.
fn floor_depth(o: &Obj, head: &HmdMatrix34_t) -> Option<(f32, f32)> {
    let cam = apply(head, o.origin);
    let dir = apply(head, (o.corners[2] + o.corners[3]) * 0.5) - cam;
    let down = -dir.y / dir.len();
    if o.clip[3] || cam.y < 0.5 || down < MIN_DOWN {
        return None;
    }
    let z = o.dist * cam.y / -dir.y * (1.0 + CONTACT_REL);
    let h_px = (o.corners[0].y - o.corners[3].y) / o.dist * o.f;
    let sigma_angle = (BOTTOM_PX + BOTTOM_REL * h_px) / o.f;
    // The range is H / tan(a), a the angle below the horizon: dR/da = H / sin^2 a
    Some((z, cam.y / (down * down) * sigma_angle + CONTACT_SPREAD * z))
}

/// The object's depth (head frame), its std dev, and whether it's weak (seen in one view only, depth from the floor):
/// stereo, fused with floor contact where the two agree. None for one-view boxes off the floor.
fn depth(o: &Obj, head: &HmdMatrix34_t) -> Option<(f32, f32, bool)> {
    let floor = floor_depth(o, head);
    match floor {
        Some((zf, sf)) if !o.stereo => Some((zf, sf, true)),
        Some((zf, sf)) if (zf - o.dist).abs() < CONTACT_GATE * sf.hypot(o.sigma_z) => {
            let (wf, ws) = (1.0 / (sf * sf), 1.0 / (o.sigma_z * o.sigma_z));
            Some(((wf * zf + ws * o.dist) / (wf + ws), (wf + ws).sqrt().recip(), false))
        }
        _ if o.stereo => Some((o.dist, o.sigma_z, false)),
        _ => None,
    }
}

/// Detections this frame that roommate-radar cares about, in the standing universe (head pose at capture): with a
/// depth (stereo, or for one-view boxes where they meet the floor), a wanted class, not around a controller (`ctrl`,
/// standing universe, at capture), within range, not entirely above MAX_BOTTOM, and near the play area (`walls`: its
/// boundary when that setting is on, else empty). Each with its noise: stereo's along the viewing ray, far larger than
/// across it. objs are in the head frame turned by yaw (rad, measure::yawed): 0 for the SLAM pair, +-90 deg for the side
/// cameras' views (detector.rs), which look along their frame's -z just as the SLAM views do along the head's.
pub fn measurements(
    objs: &[Obj],
    head: &HmdMatrix34_t,
    yaw: f32,
    ctrl: &[V3],
    walls: &[[[f32; 2]; 2]],
    set: &Settings,
) -> Vec<Meas> {
    let h = position(head);
    let arms: Vec<(V3, V3)> = if set.hands { ctrl } else { &[] }
        .iter()
        .map(|&p| {
            let hand = to_head(head, p);
            let shoulder = v3(0.18f32.copysign(hand.x), -0.25, 0.0); // one body size for everyone
            (yawed(shoulder, -yaw), yawed(hand, -yaw))
        })
        .collect();
    let head = &turned(head, yaw);
    objs.iter()
        .filter(|o| wanted(o.cls, set))
        .filter_map(|o| {
            let (z, sigma_z, weak) = depth(o, head)?;
            let corners = o.corners.map(|c| o.origin + (c - o.origin) * (z / o.dist));
            let o = &Obj { dist: z, sigma_z, corners, ..*o };
            if holds_controller(o, &arms) || (set.hands && own_arm(o)) {
                return None;
            }
            let w = o.corners.map(|c| apply(head, c));
            let c = (w[0] + w[1] + w[2] + w[3]) * 0.25;
            let dist = (c.x - h.x).hypot(c.z - h.z);
            if dist > set.max_dist || (o.cls == 0 && dist < 0.5) || !near_play_area(walls, c.x, c.z) {
                return None; // a "person" closer than 50 cm is you
            }
            if w.iter().all(|p| p.y > MAX_BOTTOM) {
                return None;
            }
            let cam = apply(head, o.origin);
            let r = (c - cam).len();
            let (bw, bh) = ((w[1] - w[0]).len(), (w[3] - w[0]).len());
            let sigma_lat = CENTER_M + CENTER_REL * bw + r / o.f;
            Some(Meas {
                p: c,
                w: bw,
                h: bh,
                ray: (c - cam) * (1.0 / r),
                sigma_ray: (sigma_z * r / z).max(sigma_lat), // depth's std dev, along the ray
                sigma_lat,
                score: o.score,
                cls: o.cls,
                eye: h,
                clip_w: o.clip[0] || o.clip[2],
                clip_h: o.clip[1] || o.clip[3],
                weak,
            })
        })
        .collect()
}

/// Whether m's box is big enough, as an angle, to be within the range setting: for the chevrons (hud.rs), where the
/// range from one camera (from the floor alone) is too rough to go by. Its height, or its width where the view cuts its
/// height off, against the least its class shows (m) at set.max_dist.
pub fn near_by_size(m: &Meas, set: &Settings) -> bool {
    let (h, w) = match m.cls {
        0 => (1.2, 0.4),
        15 => (0.25, 0.3),
        _ => (0.4, 0.5),
    };
    let (size, least) = if m.clip_h { (m.w, w) } else { (m.h, h) };
    size / (m.p - m.eye).len() >= least / set.max_dist
}

/// Whether something at p (standing universe), with the head at `head`, would have been detected: inside both views'
/// image (VIEW_MARGIN in from the edges), within range, near the play area (`walls`, as for measurements()). Only there
/// does the tracker count missing it against its track.
pub fn observable(view: &View, head: &HmdMatrix34_t, p: V3, walls: &[[[f32; 2]; 2]], set: &Settings) -> bool {
    let h = position(head);
    if (p.x - h.x).hypot(p.z - h.z) > set.max_dist || !near_play_area(walls, p.x, p.z) {
        return false;
    }
    let q = to_head(head, p);
    let half = view.s as f32 / 2.0;
    (0..2).all(|i| {
        let d = q - view.origins[i];
        let (u, v) = (half + view.f * d.x / -d.z, half - view.f * d.y / -d.z);
        d.z < -0.3 && view.valid[i].inside(u, v, VIEW_MARGIN * view.s as f32)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::Valid;

    const HEAD: HmdMatrix34_t = HmdMatrix34_t { m: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 1.6], [0.0, 0.0, 1.0, 0.0]] };

    fn obj(cls: usize, z: f32, corners: [[f32; 3]; 4]) -> Obj {
        Obj {
            cls,
            score: 0.8,
            dist: z,
            sigma_z: 0.05,
            f: 134.0,
            corners: corners.map(|[x, y, z]| v3(x, y, z)),
            clip: [false; 4],
            stereo: true,
            origin: V3::default(),
            disp: crate::stereo::NO_DISPARITY,
        }
    }

    /// 60 cm wide, from 0.5 m above to 1 m below the eyes, z m ahead, x to the side.
    fn person(cls: usize, z: f32, x: f32) -> Obj {
        obj(cls, z, [[x - 0.3, 0.5, -z], [x + 0.3, 0.5, -z], [x + 0.3, -1.0, -z], [x - 0.3, -1.0, -z]])
    }

    fn hand() -> Obj {
        obj(0, 0.6, [[-0.1, -0.25, -0.6], [0.1, -0.25, -0.6], [0.1, -0.45, -0.6], [-0.1, -0.45, -0.6]])
    }

    fn meas(objs: &[Obj], ctrl: &[V3], set: &Settings) -> Vec<Meas> {
        measurements(objs, &HEAD, 0.0, ctrl, &[], set)
    }

    #[test]
    fn controllers_far_and_unwanted_dropped() {
        // A person 2 m ahead is kept, in the standing universe; a far one, a cup and a hand holding a controller are
        // not. A controller between you and the person doesn't hide them (different depth).
        let set = Settings::default();
        let ctrl = [v3(0.0, 1.25, -0.6), v3(0.2, 1.3, -0.6)]; // one in the hand box, one before the person
        let ms = meas(&[hand(), person(0, 2.0, 0.2), person(0, 5.0, 0.0), person(41, 1.0, 0.0)], &ctrl, &set);
        assert_eq!(ms.len(), 1);
        let m = ms[0];
        assert!((m.p.x - 0.2).abs() < 1e-5 && (m.p.y - 1.35).abs() < 1e-5, "head frame -> standing universe center");
        assert!((m.w - 0.6).abs() < 1e-5 && (m.h - 1.5).abs() < 1e-5, "and size");
        assert!(!m.weak && (m.ray.len() - 1.0).abs() < 1e-5 && m.ray.z < -0.98, "seen ahead");
        assert!(m.sigma_ray >= m.sigma_lat && m.sigma_lat < 0.1);
        assert_eq!(meas(&[hand()], &[], &set).len(), 1, "a hand without a controller is kept");
        assert_eq!(meas(&[hand()], &ctrl, &Settings { hands: false, ..set }).len(), 1, "unless ignoring hands is off");
    }

    #[test]
    fn small_boxes_are_far() {
        // A person 1.5 m tall 2 m away is within the 3 m range; a box 30 cm tall there is someone far beyond it.
        let set = Settings::default();
        assert!(meas(&[person(0, 2.0, 0.0)], &[], &set).iter().all(|m| near_by_size(m, &set)));
        let small = obj(0, 2.0, [[-0.1, -0.7, -2.0], [0.1, -0.7, -2.0], [0.1, -1.0, -2.0], [-0.1, -1.0, -2.0]]);
        let ms = meas(&[small], &[], &set);
        assert!(ms.len() == 1 && !near_by_size(&ms[0], &set));
    }

    #[test]
    fn side_views() {
        // The left side camera's view (yaw +90 deg): a person 2 m along its -z is 2 m to your left, and your left arm
        // reaching out to a controller there is dropped.
        let set = Settings::default();
        let left = std::f32::consts::FRAC_PI_2;
        let ms = measurements(&[person(0, 2.0, 0.0)], &HEAD, left, &[], &[], &set);
        assert!(ms.len() == 1 && (ms[0].p.x + 2.0).abs() < 1e-4 && ms[0].p.z.abs() < 1e-4);
        let arm = obj(0, 0.55, [[-0.06, -0.2, -0.55], [0.06, -0.2, -0.55], [0.06, -0.3, -0.55], [-0.06, -0.3, -0.55]]);
        assert!(measurements(&[arm], &HEAD, left, &[v3(-0.6, 1.35, 0.0)], &[], &set).is_empty());
        assert_eq!(measurements(&[arm], &HEAD, left, &[], &[], &set).len(), 1);
    }

    #[test]
    fn arms_dropped() {
        let set = Settings::default();
        // forearm reaching out to a controller 45 cm further, outside the box
        let arm = obj(0, 0.55, [[0.18, -0.2, -0.55], [0.3, -0.2, -0.55], [0.3, -0.27, -0.55], [0.18, -0.27, -0.55]]);
        assert!(meas(&[arm], &[v3(0.3, 1.3, -1.0)], &set).is_empty() && meas(&[arm], &[], &set).len() == 1);
        // the arm merged with the chair behind it: measured at 1.2 m, controller at 0.6 m
        assert!(meas(&[person(0, 1.2, 0.0)], &[v3(0.0, 1.3, -0.6)], &set).is_empty());
        // box from `top` down to the bottom edge of the view (100 deg): a bare arm unless there's a head above the eyes
        let low = |z: f32, top: f32| {
            let y1 = -1.2 * z;
            obj(0, z, [[-0.3, top, -z], [0.3, top, -z], [0.3, y1, -z], [-0.3, y1, -z]])
        };
        assert!(meas(&[low(0.8, -0.3)], &[], &set).is_empty());
        assert!(meas(&[low(0.8, 0.1)], &[], &set).len() == 1 && meas(&[low(2.0, -0.3)], &[], &set).len() == 1);
        assert!(meas(&[person(0, 0.4, 0.0)], &[], &set).is_empty(), "person closer than 50 cm dropped");
        // box 0.5 m tall whose bottom is `y` above the floor
        let high = |y: f32| {
            let y0 = y - HEAD.m[1][3];
            obj(0, 2.0, [[-0.3, y0 + 0.5, -2.0], [0.3, y0 + 0.5, -2.0], [0.3, y0, -2.0], [-0.3, y0, -2.0]])
        };
        assert!(meas(&[high(MAX_BOTTOM + 0.1)], &[], &set).is_empty(), "box entirely above {MAX_BOTTOM} m dropped");
        assert_eq!(meas(&[high(MAX_BOTTOM - 0.1)], &[], &set).len(), 1, "box reaching below {MAX_BOTTOM} m kept");
    }

    #[test]
    fn play_area() {
        // 2 x 2 m square around the origin; the margin reaches PLAY_AREA_MARGIN past it (e: its edge)
        let walls = [
            [[-1.0, -1.0], [1.0, -1.0]],
            [[1.0, -1.0], [1.0, 1.0]],
            [[1.0, 1.0], [-1.0, 1.0]],
            [[-1.0, 1.0], [-1.0, -1.0]],
        ];
        let (m, e) = (PLAY_AREA_MARGIN, 1.0 + PLAY_AREA_MARGIN);
        assert!(
            near_play_area(&walls, 0.0, 0.0)
                && near_play_area(&walls, e - 0.1, 0.0)
                && near_play_area(&walls, 0.0, 0.1 - e)
        );
        let c = 1.0 + 0.75 * m; // 1.06 margins from the corner
        assert!(
            !near_play_area(&walls, e + 0.1, 0.0) && !near_play_area(&walls, c, c) && near_play_area(&[], 9.0, 9.0)
        );
        let set = Settings::default();
        let ms = |z| measurements(&[person(0, z, 0.0)], &HEAD, 0.0, &[], &walls, &set).len();
        assert!(
            ms(e - 0.5) == 1 && ms(e + 0.5) == 0,
            "kept half a meter inside the margin, dropped half a meter past it"
        );
    }

    #[test]
    fn floor_contact() {
        let set = Settings { max_dist: 6.0, ..Settings::default() };
        // Box bottom on the floor 4 m ahead (1.6 m below the eyes), so the feet are CONTACT_REL farther: stereo reading
        // 4.6 m with a 1 m std dev is pulled to the floor contact's, and its noise shrinks.
        let standing = |z: f32| obj(0, z, [[-0.3, 0.1, -z], [0.3, 0.1, -z], [0.3, -1.6, -z], [-0.3, -1.6, -z]]);
        let feet = 4.0 * (1.0 + CONTACT_REL);
        let seen = Obj { sigma_z: 1.0, ..scale(standing(4.0), 4.6) };
        let m = meas(&[seen], &[], &set)[0];
        assert!((m.p.z + feet).abs() < 0.15 && m.sigma_ray < 0.5 && !m.weak, "{m:?}");
        // Seen in one view only (on the 1 m plane): the floor contact's depth alone, weak.
        let one = Obj { stereo: false, sigma_z: 0.0, ..scale(standing(4.0), 1.0) };
        let m = meas(&[one], &[], &set)[0];
        assert!((m.p.z + feet).abs() < 0.01 && m.weak);
        // Sitting (bottom edge 0.5 m up), stereo good: the floor contact (far behind) is left out.
        let sitting = obj(0, 3.0, [[-0.3, 0.1, -3.0], [0.3, 0.1, -3.0], [0.3, -1.1, -3.0], [-0.3, -1.1, -3.0]]);
        assert!((meas(&[sitting], &[], &set)[0].p.z + 3.0).abs() < 1e-4);
        // Its bottom cut off by the view's edge, seen in one view: no depth at all.
        let cut = Obj { clip: [false, false, false, true], ..one };
        assert!(meas(&[cut], &[], &set).is_empty());
    }

    /// o moved to depth z along its rays from the camera.
    fn scale(o: Obj, z: f32) -> Obj {
        Obj { dist: z, corners: o.corners.map(|c| o.origin + (c - o.origin) * (z / o.dist)), ..o }
    }

    #[test]
    fn what_the_views_see() {
        // Two views at f = 100 px, 200 px wide, the right half of the left one outside the fisheye.
        let mut left = Valid { rows: vec![(0.0, 100.0); 200], cols: vec![(0.0, 200.0); 200] };
        left.cols[100..].fill((200.0, 0.0));
        let all = Valid { rows: vec![(0.0, 200.0); 200], cols: vec![(0.0, 200.0); 200] };
        let view = View { s: 200, f: 100.0, origins: [V3::default(); 2], valid: [left, all] };
        let set = Settings { max_dist: 6.0, ..Settings::default() };
        let sees = |p: V3| observable(&view, &HEAD, p, &[], &set);
        assert!(sees(v3(-0.5, 1.6, -3.0)), "ahead, a little left");
        assert!(!sees(v3(0.5, 1.6, -3.0)), "outside the left view's image");
        assert!(!sees(v3(0.0, 1.6, 3.0)), "behind");
        assert!(!sees(v3(-0.5, 1.6, -7.0)), "beyond range");
    }

    #[test]
    fn classes() {
        let set = Settings { people: false, ..Settings::default() };
        assert!(!wanted(0, &set) && wanted(16, &set) && !wanted(41, &set));
    }
}
