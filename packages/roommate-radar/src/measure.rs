//! The detector's objects (head frame at capture) as the tracker's measurements (standing universe), keeping only what
//! roommate-radar shows: stereo, a wanted class, in range, not your own arms or hands.
use crate::detector::HFOV_DEG;
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
/// differently in each view, or merged with what's behind the arm). Box and controllers are in the head frame at
/// capture time.
fn holds_controller(o: &Obj, controllers: &[V3]) -> bool {
    let m = 0.15;
    let (c, b) = (o.origin, &o.corners);
    controllers.iter().any(|&hand| {
        let shoulder = v3(0.18f32.copysign(hand.x), -0.25, 0.0); // ponytail: one body size for everyone
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

/// Your own arm or lap, controller or not: the box reaches the bottom 10% of the view, is within 1.2 m, and its top is
/// below eye level (anyone else that close shows their head). Head frame, like holds_controller.
fn own_arm(o: &Obj) -> bool {
    let b = &o.corners;
    let z = o.origin.z - b[3].z;
    let edge = (HFOV_DEG.to_radians() / 2.0).tan();
    z > 0.0 && z < 1.2 && (o.origin.y - b[3].y) / z > 0.8 * edge && b[0].y < o.origin.y
}

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

/// Detections this frame that roommate-radar cares about, in the standing universe (head pose at capture): stereo (has
/// depth), a wanted class, not around a controller (`ctrl`, standing universe, at capture), within range, not entirely
/// above MAX_BOTTOM, and near the play area (`walls`: its boundary when that setting is on, else empty).
pub fn measurements(
    objs: &[Obj],
    head: &HmdMatrix34_t,
    ctrl: &[V3],
    walls: &[[[f32; 2]; 2]],
    set: &Settings,
) -> Vec<Meas> {
    let h = position(head);
    let m = &head.m;
    // head_from_world = R^T (p - t)
    let ctrl_head: Vec<V3> = if set.hands { ctrl } else { &[] }
        .iter()
        .map(|&p| {
            let d = p - h;
            let col = |j: usize| m[0][j] * d.x + m[1][j] * d.y + m[2][j] * d.z;
            v3(col(0), col(1), col(2))
        })
        .collect();
    objs.iter()
        .filter(|o| o.stereo && wanted(o.cls, set) && !holds_controller(o, &ctrl_head) && !(set.hands && own_arm(o)))
        .filter_map(|o| {
            let w = o.corners.map(|c| apply(head, c));
            let c = (w[0] + w[1] + w[2] + w[3]) * 0.25;
            let dist = (c.x - h.x).hypot(c.z - h.z);
            if dist > set.max_dist || (o.cls == 0 && dist < 0.5) || !near_play_area(walls, c.x, c.z) {
                return None; // a "person" closer than 50 cm is you
            }
            if w.iter().all(|p| p.y > MAX_BOTTOM) {
                return None;
            }
            // ponytail: isotropic position noise growing like stereo depth error (~Z^2); per-axis noise along the view
            // ray would track lateral motion tighter
            let sigma = 0.05 + 0.02 * o.dist * o.dist;
            Some(Meas {
                v: [c.x, c.y, c.z, (w[1] - w[0]).len(), (w[3] - w[0]).len()],
                sigma,
                score: o.score,
                cls: o.cls,
                eye: [h.x, h.y, h.z],
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: HmdMatrix34_t = HmdMatrix34_t { m: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 1.6], [0.0, 0.0, 1.0, 0.0]] };

    fn obj(cls: usize, z: f32, corners: [[f32; 3]; 4]) -> Obj {
        Obj {
            cls,
            score: 0.8,
            dist: z,
            corners: corners.map(|[x, y, z]| v3(x, y, z)),
            stereo: true,
            origin: V3::default(),
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
        measurements(objs, &HEAD, ctrl, &[], set)
    }

    #[test]
    fn controllers_far_and_unwanted_dropped() {
        // A person 2 m ahead is kept, in the standing universe; a far one, a cup and a hand holding a controller are
        // not. A controller between you and the person doesn't hide them (different depth).
        let set = Settings::default();
        let ctrl = [v3(0.0, 1.25, -0.6), v3(0.2, 1.3, -0.6)]; // one in the hand box, one before the person
        let ms = meas(&[hand(), person(0, 2.0, 0.2), person(0, 5.0, 0.0), person(41, 1.0, 0.0)], &ctrl, &set);
        assert_eq!(ms.len(), 1);
        let v = ms[0].v;
        assert!((v[0] - 0.2).abs() < 1e-5 && (v[1] - 1.35).abs() < 1e-5, "head frame -> standing universe center");
        assert!((v[3] - 0.6).abs() < 1e-5 && (v[4] - 1.5).abs() < 1e-5, "and size");
        assert_eq!(meas(&[hand()], &[], &set).len(), 1, "a hand without a controller is kept");
        assert_eq!(meas(&[hand()], &ctrl, &Settings { hands: false, ..set }).len(), 1, "unless ignoring hands is off");
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
        let high =
            |y0: f32| obj(0, 2.0, [[-0.3, y0 + 0.5, -2.0], [0.3, y0 + 0.5, -2.0], [0.3, y0, -2.0], [-0.3, y0, -2.0]]);
        assert!(meas(&[high(0.5)], &[], &set).is_empty(), "box entirely above 2 m dropped");
        assert_eq!(meas(&[high(0.3)], &[], &set).len(), 1, "box reaching below 2 m kept");
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
        let ms = |z| measurements(&[person(0, z, 0.0)], &HEAD, &[], &walls, &set).len();
        assert!(
            ms(e - 0.5) == 1 && ms(e + 0.5) == 0,
            "kept half a meter inside the margin, dropped half a meter past it"
        );
    }

    #[test]
    fn classes() {
        let set = Settings { people: false, ..Settings::default() };
        assert!(!wanted(0, &set) && wanted(16, &set) && !wanted(41, &set));
    }
}
