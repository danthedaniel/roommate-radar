//! The detector's objects (head frame at capture) as the tracker's measurements (standing universe), keeping only what
//! roommate-radar shows: a wanted class, in range, not your own arms or hands. Depth from stereo, and from where the
//! box meets the floor (the floor and the head pose are known exactly) where the two agree; boxes seen in one view only
//! get theirs from the floor alone.
use crate::detector::{HFOV_DEG, View};
use crate::openvr::HmdMatrix34_t;
use crate::settings::Settings;
use crate::stereo::Obj;
use crate::tracker::Meas;
use crate::v3::{V3, position, transform, v3};

/// person, cat, dog
fn wanted(cls: usize, settings: &Settings) -> bool {
    if cls == 0 { settings.people } else { (cls == 15 || cls == 16) && settings.pets }
}

/// A wanted class's name.
pub fn class_name(cls: usize) -> &'static str {
    match cls {
        0 => "person",
        15 => "cat",
        _ => "dog",
    }
}

/// Your own arms and hands (YOLO often calls them "person"; it doesn't know the controller, so an arm box can stop at
/// the wrist and leave it out): a point on the forearm, from 40% of the way from the shoulder to a controller up to the
/// controller, inside the box: within its rectangle scaled to that point's depth (seen from the camera) and grown by
/// 15 cm, and from 40 cm behind its depth to 80 cm in front of it (arm boxes measure too far: cut off by the image edge
/// differently in each view, or merged with what's behind the arm). Box and arms (shoulder, controller) are in the
/// head frame at capture time.
fn holds_controller(o: &Obj, arms: &[(V3, V3)]) -> bool {
    let margin_m = 0.15;
    let (cam, b) = (o.origin, &o.corners);
    arms.iter().any(|&(shoulder, hand)| {
        (4..=10).any(|k| {
            let p = shoulder + (hand - shoulder) * (k as f32 / 10.0);
            let depth_scale = (p.z - cam.z) / (b[0].z - cam.z);
            depth_scale > 0.0
                && p.x > cam.x + (b[0].x - cam.x) * depth_scale - margin_m
                && p.x < cam.x + (b[1].x - cam.x) * depth_scale + margin_m
                && p.y > cam.y + (b[3].y - cam.y) * depth_scale - margin_m
                && p.y < cam.y + (b[0].y - cam.y) * depth_scale + margin_m
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
    let tan_half_fov = (HFOV_DEG.to_radians() / 2.0).tan();
    z > 0.0 && z < ARM_DIST && (o.origin.y - b[3].y) / z > 0.8 * tan_half_fov && b[0].y < o.origin.y
}

/// How close your own arm or lap is, at most (m).
const ARM_DIST: f32 = 1.2;

/// Boxes entirely higher than this above the floor (standing universe y = 0) are dropped: nobody's up there (m).
const MAX_BOTTOM_Y: f32 = 2.0;

/// How far outside the play area's boundary detections are still kept (m).
pub const PLAY_AREA_MARGIN: f32 = 1.0;

/// [x, z] inside the boundary (`walls`, Vr::boundary_walls) or within PLAY_AREA_MARGIN of it; no walls: anywhere.
fn near_play_area(walls: &[[[f32; 2]; 2]], x: f32, z: f32) -> bool {
    let mut inside = walls.is_empty();
    for &[[x0, z0], [x1, z1]] in walls {
        // even-odd: walls crossing the +x ray from the point
        if (z0 > z) != (z1 > z) && x < x0 + (z - z0) / (z1 - z0) * (x1 - x0) {
            inside = !inside;
        }
        let (dx, dz) = (x1 - x0, z1 - z0);
        let along_wall = (((x - x0) * dx + (z - z0) * dz) / (dx * dx + dz * dz).max(1e-9)).clamp(0.0, 1.0);
        if (x - x0 - along_wall * dx).hypot(z - z0 - along_wall * dz) < PLAY_AREA_MARGIN {
            return true;
        }
    }
    inside
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
const MIN_DOWN_SIN: f32 = 0.05;
/// What the tracker counts as in view (observable()) is this fraction of the view's size in from its edges.
const VIEW_MARGIN: f32 = 0.04;

/// Depth (head frame) at which the box's bottom edge meets the floor (standing universe y = 0), and its std dev. For
/// something standing on the floor with its feet in view, a far better range at a distance than stereo's: the
/// camera's height is a 1.6 m baseline, stereo's is 10.6 cm. Corrected for CONTACT_REL. None if the bottom edge is cut
/// off by the view, or isn't looking down onto the floor.
fn floor_depth(o: &Obj, head: &HmdMatrix34_t) -> Option<(f32, f32)> {
    let cam = transform(head, o.origin);
    let dir = transform(head, (o.corners[2] + o.corners[3]) * 0.5) - cam;
    let down_sin = -dir.y / dir.len();
    if o.clip[3] || cam.y < 0.5 || down_sin < MIN_DOWN_SIN {
        return None;
    }
    let z = o.depth * cam.y / -dir.y * (1.0 + CONTACT_REL);
    let h_px = (o.corners[0].y - o.corners[3].y) / o.depth * o.focal_px;
    let sigma_angle = (BOTTOM_PX + BOTTOM_REL * h_px) / o.focal_px;
    // The range is H / tan(a), a the angle below the horizon: dR/da = H / sin^2 a
    Some((z, cam.y / (down_sin * down_sin) * sigma_angle + CONTACT_SPREAD * z))
}

/// The object's depth (head frame), its std dev, and whether it's weak (seen in one view only, depth from the floor):
/// stereo, fused with floor contact where the two agree. None for one-view boxes off the floor.
fn depth(o: &Obj, head: &HmdMatrix34_t) -> Option<(f32, f32, bool)> {
    let floor = floor_depth(o, head);
    match floor {
        Some((floor_z, floor_sigma)) if !o.stereo => Some((floor_z, floor_sigma, true)),
        Some((floor_z, floor_sigma)) if (floor_z - o.depth).abs() < CONTACT_GATE * floor_sigma.hypot(o.sigma_z) => {
            let (floor_weight, stereo_weight) = (1.0 / (floor_sigma * floor_sigma), 1.0 / (o.sigma_z * o.sigma_z));
            Some((
                (floor_weight * floor_z + stereo_weight * o.depth) / (floor_weight + stereo_weight),
                (floor_weight + stereo_weight).sqrt().recip(),
                false,
            ))
        }
        _ if o.stereo => Some((o.depth, o.sigma_z, false)),
        _ => None,
    }
}

/// Detections this frame that roommate-radar cares about, in the standing universe (head pose at capture): with a
/// depth (stereo, or for one-view boxes where they meet the floor), a wanted class, not around a controller (`ctrl`,
/// standing universe, at capture), within range, not entirely above MAX_BOTTOM_Y, and near the play area (`walls`: its
/// boundary when that setting is on, else empty). Each with its noise: stereo's along the viewing ray, far larger than
/// across it. objs are in the head frame.
pub fn measurements(
    objs: &[Obj],
    head: &HmdMatrix34_t,
    ctrl: &[V3],
    walls: &[[[f32; 2]; 2]],
    settings: &Settings,
) -> Vec<Meas> {
    let head_pos = position(head);
    let arms: Vec<(V3, V3)> = if settings.ignore_hands { ctrl } else { &[] }
        .iter()
        .map(|&p| {
            let hand = to_head(head, p);
            let shoulder = v3(0.18f32.copysign(hand.x), -0.25, 0.0); // one body size for everyone
            (shoulder, hand)
        })
        .collect();
    objs.iter()
        .filter(|o| wanted(o.cls, settings))
        .filter_map(|o| {
            // Not finite: stereo's sigma_z is 0 at its least disparity, which fusing with the floor's makes NaN
            let (z, sigma_z, weak) = depth(o, head).filter(|d| d.0.is_finite() && d.1.is_finite())?;
            let corners = o.corners.map(|corner| o.origin + (corner - o.origin) * (z / o.depth));
            let o = &Obj { depth: z, sigma_z, corners, ..*o };
            if holds_controller(o, &arms) || (settings.ignore_hands && own_arm(o)) {
                return None;
            }
            let world = o.corners.map(|c| transform(head, c));
            let center = (world[0] + world[1] + world[2] + world[3]) * 0.25;
            let horiz_dist = (center.x - head_pos.x).hypot(center.z - head_pos.z);
            if horiz_dist > settings.max_dist
                || (o.cls == 0 && horiz_dist < 0.5)
                || !near_play_area(walls, center.x, center.z)
            {
                return None; // a "person" closer than 50 cm is you
            }
            if world.iter().all(|p| p.y > MAX_BOTTOM_Y) {
                return None;
            }
            let cam = transform(head, o.origin);
            let r = (center - cam).len();
            let (bw, bh) = ((world[1] - world[0]).len(), (world[3] - world[0]).len());
            let sigma_lat = CENTER_M + CENTER_REL * bw + r / o.focal_px;
            Some(Meas {
                p: center,
                w: bw,
                h: bh,
                ray: (center - cam) * (1.0 / r),
                sigma_ray: (sigma_z * r / z).max(sigma_lat), // depth's std dev, along the ray
                sigma_lat,
                score: o.score,
                cls: o.cls,
                eye: head_pos,
                clip_w: o.clip[0] || o.clip[2],
                clip_h: o.clip[1] || o.clip[3],
                one_view: weak,
            })
        })
        .collect()
}

/// Whether something at p (standing universe), with the head at `head`, would have been detected: inside both views'
/// image (VIEW_MARGIN in from the edges), within range, near the play area (`walls`, as for measurements()). Only there
/// does the tracker count missing it against its track.
pub fn observable(view: &View, head: &HmdMatrix34_t, p: V3, walls: &[[[f32; 2]; 2]], settings: &Settings) -> bool {
    let head_pos = position(head);
    if (p.x - head_pos.x).hypot(p.z - head_pos.z) > settings.max_dist || !near_play_area(walls, p.x, p.z) {
        return false;
    }
    let q = to_head(head, p);
    let half = view.size_px as f32 / 2.0;
    (0..2).all(|i| {
        let d = q - view.origins[i];
        let (u, v) = (half + view.focal_px * d.x / -d.z, half - view.focal_px * d.y / -d.z);
        d.z < -0.3 && view.valid[i].inside(u, v, VIEW_MARGIN * view.size_px as f32)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rectify::ValidRegion;

    const HEAD: HmdMatrix34_t = HmdMatrix34_t { m: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 1.6], [0.0, 0.0, 1.0, 0.0]] };

    fn obj(cls: usize, z: f32, corners: [[f32; 3]; 4]) -> Obj {
        Obj {
            cls,
            score: 0.8,
            depth: z,
            sigma_z: 0.05,
            focal_px: 134.0,
            corners: corners.map(|[x, y, z]| v3(x, y, z)),
            clip: [false; 4],
            stereo: true,
            origin: V3::default(),
            disp: crate::stereo::NO_DISPARITY,
        }
    }

    /// 60 cm wide, from 0.5 m above to 1 m below the eyes, z m ahead, x to the side.
    fn upright_box(cls: usize, z: f32, x: f32) -> Obj {
        obj(cls, z, [[x - 0.3, 0.5, -z], [x + 0.3, 0.5, -z], [x + 0.3, -1.0, -z], [x - 0.3, -1.0, -z]])
    }

    fn hand() -> Obj {
        obj(0, 0.6, [[-0.1, -0.25, -0.6], [0.1, -0.25, -0.6], [0.1, -0.45, -0.6], [-0.1, -0.45, -0.6]])
    }

    fn meas(objs: &[Obj], ctrl: &[V3], settings: &Settings) -> Vec<Meas> {
        measurements(objs, &HEAD, ctrl, &[], settings)
    }

    #[test]
    fn controllers_far_and_unwanted_dropped() {
        // A person 2 m ahead is kept, in the standing universe; a far one, a cup and a hand holding a controller are
        // not. A controller between you and the person doesn't hide them (different depth).
        let settings = Settings::default();
        let ctrl = [v3(0.0, 1.25, -0.6), v3(0.2, 1.3, -0.6)]; // one in the hand box, one before the person
        let ms = meas(
            &[hand(), upright_box(0, 2.0, 0.2), upright_box(0, 5.0, 0.0), upright_box(41, 1.0, 0.0)],
            &ctrl,
            &settings,
        );
        assert_eq!(ms.len(), 1);
        let m = ms[0];
        assert!((m.p.x - 0.2).abs() < 1e-5 && (m.p.y - 1.35).abs() < 1e-5, "head frame -> standing universe center");
        assert!((m.w - 0.6).abs() < 1e-5 && (m.h - 1.5).abs() < 1e-5, "and size");
        assert!(!m.one_view && (m.ray.len() - 1.0).abs() < 1e-5 && m.ray.z < -0.98, "seen ahead");
        assert!(m.sigma_ray >= m.sigma_lat && m.sigma_lat < 0.1);
        assert_eq!(meas(&[hand()], &[], &settings).len(), 1, "a hand without a controller is kept");
        assert_eq!(
            meas(&[hand()], &ctrl, &Settings { ignore_hands: false, ..settings }).len(),
            1,
            "unless ignoring hands is off"
        );
    }

    #[test]
    fn arms_dropped() {
        let settings = Settings::default();
        // forearm reaching out to a controller 45 cm further, outside the box
        let arm = obj(0, 0.55, [[0.18, -0.2, -0.55], [0.3, -0.2, -0.55], [0.3, -0.27, -0.55], [0.18, -0.27, -0.55]]);
        assert!(meas(&[arm], &[v3(0.3, 1.3, -1.0)], &settings).is_empty() && meas(&[arm], &[], &settings).len() == 1);
        // the arm merged with the chair behind it: measured at 1.2 m, controller at 0.6 m
        assert!(meas(&[upright_box(0, 1.2, 0.0)], &[v3(0.0, 1.3, -0.6)], &settings).is_empty());
        // box from `top` down to the bottom edge of the view (100 deg): a bare arm unless there's a head above the eyes
        let low = |z: f32, top: f32| {
            let y1 = -1.2 * z;
            obj(0, z, [[-0.3, top, -z], [0.3, top, -z], [0.3, y1, -z], [-0.3, y1, -z]])
        };
        assert!(meas(&[low(0.8, -0.3)], &[], &settings).is_empty());
        assert!(
            meas(&[low(0.8, 0.1)], &[], &settings).len() == 1 && meas(&[low(2.0, -0.3)], &[], &settings).len() == 1
        );
        assert!(meas(&[upright_box(0, 0.4, 0.0)], &[], &settings).is_empty(), "person closer than 50 cm dropped");
        // box 0.5 m tall whose bottom is `y` above the floor
        let high = |y: f32| {
            let y0 = y - HEAD.m[1][3];
            obj(0, 2.0, [[-0.3, y0 + 0.5, -2.0], [0.3, y0 + 0.5, -2.0], [0.3, y0, -2.0], [-0.3, y0, -2.0]])
        };
        assert!(
            meas(&[high(MAX_BOTTOM_Y + 0.1)], &[], &settings).is_empty(),
            "box entirely above {MAX_BOTTOM_Y} m dropped"
        );
        assert_eq!(
            meas(&[high(MAX_BOTTOM_Y - 0.1)], &[], &settings).len(),
            1,
            "box reaching below {MAX_BOTTOM_Y} m kept"
        );
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
        let settings = Settings::default();
        let kept = |z| measurements(&[upright_box(0, z, 0.0)], &HEAD, &[], &walls, &settings).len();
        assert!(
            kept(e - 0.5) == 1 && kept(e + 0.5) == 0,
            "kept half a meter inside the margin, dropped half a meter past it"
        );
    }

    #[test]
    fn floor_contact() {
        let settings = Settings { max_dist: 6.0, ..Settings::default() };
        // Box bottom on the floor 4 m ahead (1.6 m below the eyes), so the feet are CONTACT_REL farther: stereo reading
        // 4.6 m with a 1 m std dev is pulled to the floor contact's, and its noise shrinks.
        let standing = |z: f32| obj(0, z, [[-0.3, 0.1, -z], [0.3, 0.1, -z], [0.3, -1.6, -z], [-0.3, -1.6, -z]]);
        let feet = 4.0 * (1.0 + CONTACT_REL);
        let seen = Obj { sigma_z: 1.0, ..scale(standing(4.0), 4.6) };
        let m = meas(&[seen], &[], &settings)[0];
        assert!((m.p.z + feet).abs() < 0.15 && m.sigma_ray < 0.5 && !m.one_view, "{m:?}");
        // Seen in one view only (on the 1 m plane): the floor contact's depth alone, weak.
        let one = Obj { stereo: false, sigma_z: 0.0, ..scale(standing(4.0), 1.0) };
        let m = meas(&[one], &[], &settings)[0];
        assert!((m.p.z + feet).abs() < 0.01 && m.one_view);
        // Sitting (bottom edge 0.5 m up), stereo good: the floor contact (far behind) is left out.
        let sitting = obj(0, 3.0, [[-0.3, 0.1, -3.0], [0.3, 0.1, -3.0], [0.3, -1.1, -3.0], [-0.3, -1.1, -3.0]]);
        assert!((meas(&[sitting], &[], &settings)[0].p.z + 3.0).abs() < 1e-4);
        // Its bottom cut off by the view's edge, seen in one view: no depth at all.
        let cut = Obj { clip: [false, false, false, true], ..one };
        assert!(meas(&[cut], &[], &settings).is_empty());
    }

    /// o moved to depth z along its rays from the camera.
    fn scale(o: Obj, z: f32) -> Obj {
        Obj { depth: z, corners: o.corners.map(|c| o.origin + (c - o.origin) * (z / o.depth)), ..o }
    }

    #[test]
    fn what_the_views_see() {
        // Two views at f = 100 px, 200 px wide, the right half of the left one outside the fisheye.
        let mut left = ValidRegion { rows: vec![(0.0, 100.0); 200], cols: vec![(0.0, 200.0); 200] };
        left.cols[100..].fill((200.0, 0.0));
        let all = ValidRegion { rows: vec![(0.0, 200.0); 200], cols: vec![(0.0, 200.0); 200] };
        let view = View { size_px: 200, focal_px: 100.0, origins: [V3::default(); 2], valid: [left, all] };
        let settings = Settings { max_dist: 6.0, ..Settings::default() };
        let sees = |p: V3| observable(&view, &HEAD, p, &[], &settings);
        assert!(sees(v3(-0.5, 1.6, -3.0)), "ahead, a little left");
        assert!(!sees(v3(0.5, 1.6, -3.0)), "outside the left view's image");
        assert!(!sees(v3(0.0, 1.6, 3.0)), "behind");
        assert!(!sees(v3(-0.5, 1.6, -7.0)), "beyond range");
    }

    #[test]
    fn classes() {
        let settings = Settings { people: false, ..Settings::default() };
        assert!(!wanted(0, &settings) && wanted(16, &settings) && !wanted(41, &settings));
    }
}
