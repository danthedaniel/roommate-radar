//! Stereo for the detector: matches YOLO's boxes in the two rectified views (a rectified pair, horizontal epipolar
//! lines) and triangulates their depth from disparity, into head-frame objects.
use crate::v3::{V3, v3};

/// A box in one view (px), or seen from the head (rad, boxes.rs).
#[derive(Clone, Copy, Debug, Default)]
pub struct Det {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
    pub score: f32,
    pub cls: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct Obj {
    pub cls: usize,
    pub score: f32,
    pub dist: f32,
    pub corners: [V3; 4], // head frame at capture time: TL, TR, BR, BL
    pub stereo: bool,     // false: seen in the left view only, corners are on the 1 m plane
    pub origin: V3,       // camera position (head frame) the corners were projected from
}

/// Sub-pixel disparity refinement by NCC of the left box interior along the (horizontal) epipolar line.
fn refine_disparity(l_img: &[u8], r_img: &[u8], s: usize, l: &Det, d0: f32) -> f32 {
    let (w, h) = (l.x1 - l.x0, l.y1 - l.y0);
    let (x0, x1) = ((l.x0 + 0.2 * w) as i32, (l.x1 - 0.2 * w) as i32);
    let (y0, y1) = ((l.y0 + 0.2 * h) as i32, (l.y1 - 0.2 * h) as i32);
    let step = ((x1 - x0).max(y1 - y0) / 24).max(1) as usize;
    if x1 - x0 < 6 || y1 - y0 < 6 {
        return d0;
    }
    const R: i32 = 6;
    let s_ = s as i32;
    let mut score = [0f32; 2 * R as usize + 1];
    for k in -R..=R {
        let d = d0.round() as i32 + k;
        let (mut sl, mut sr, mut sll, mut srr, mut slr) = (0f64, 0f64, 0f64, 0f64, 0f64);
        let mut n = 0;
        for y in (y0..y1).step_by(step) {
            for x in (x0..x1).step_by(step) {
                let xr = x - d;
                if x < 0 || y < 0 || x >= s_ || y >= s_ || xr < 0 || xr >= s_ {
                    continue;
                }
                let (a, b) = (l_img[(y * s_ + x) as usize] as f64, r_img[(y * s_ + xr) as usize] as f64);
                sl += a;
                sr += b;
                sll += a * a;
                srr += b * b;
                slr += a * b;
                n += 1;
            }
        }
        let nn = n.max(1) as f64;
        let cov = slr - sl * sr / nn;
        let var = (sll - sl * sl / nn) * (srr - sr * sr / nn);
        score[(k + R) as usize] = if n > 20 && var > 1e-3 { (cov / var.sqrt()) as f32 } else { -1.0 };
    }
    let best = (0..score.len()).max_by(|&a, &b| score[a].total_cmp(&score[b]).then(b.cmp(&a))).unwrap();
    if score[best] < 0.6 || best == 0 || best == 2 * R as usize {
        return d0;
    }
    let (a, b, c) = (score[best - 1], score[best], score[best + 1]);
    let sub = (a - c) / (2.0 * (a - 2.0 * b + c) + 1e-9);
    d0.round() + (best as i32 - R) as f32 + sub.clamp(-0.5, 0.5)
}

/// Matches the left view's boxes to the right view's: same rows, similar size, positive disparity. Class-agnostic like
/// nms() (the views can disagree on laptop/tv for one object), with a mild preference for the same class. l_img, r_img:
/// the S x S views; f (px) and baseline (m) of the pair; origin: the left camera in the head frame.
pub fn match_views(
    views: &[Vec<Det>; 2],
    l_img: &[u8],
    r_img: &[u8],
    s: usize,
    f: f32,
    baseline: f32,
    origin: V3,
) -> Vec<Obj> {
    let half = s as f32 / 2.0;
    let make_obj = |cls, score, z: f32, bx0: f32, by0: f32, bx1: f32, by1: f32, stereo| {
        let (us, vs) = ([bx0, bx1, bx1, bx0], [by0, by0, by1, by1]);
        let corner = |k: usize| origin + v3((us[k] - half) / f * z, -(vs[k] - half) / f * z, -z);
        Obj { cls, score, dist: z, corners: [corner(0), corner(1), corner(2), corner(3)], stereo, origin }
    };
    let mut objs = Vec::new();
    let mut used = vec![false; views[1].len()];
    for l in &views[0] {
        let (lh, lw) = (l.y1 - l.y0, l.x1 - l.x0);
        let mut best: Option<(usize, f32)> = None;
        for (j, r) in views[1].iter().enumerate() {
            if used[j] {
                continue;
            }
            let (rh, rw) = (r.y1 - r.y0, r.x1 - r.x0);
            let dy = ((l.y0 + l.y1) - (r.y0 + r.y1)).abs() / 2.0 / lh;
            let dsize = (lh - rh).abs() / lh.max(rh) + (lw - rw).abs() / lw.max(rw);
            let disp = ((l.x0 + l.x1) - (r.x0 + r.x1)) / 2.0;
            if dy > 0.15 || dsize > 0.5 || disp < 0.3 || disp > f * baseline / 0.2 {
                continue;
            }
            let cost = dy + dsize + if r.cls != l.cls { 0.2 } else { 0.0 };
            if best.is_none_or(|(_, c)| cost < c) {
                best = Some((j, cost));
            }
        }
        let Some((bi, _)) = best else {
            // Direction-only measurement: the tracker can still update an existing track with it.
            objs.push(make_obj(l.cls, l.score, 1.0, l.x0, l.y0, l.x1, l.y1, false));
            continue;
        };
        used[bi] = true;
        let r = &views[1][bi];
        let d0 = ((l.x0 - r.x0) + (l.x1 - r.x1)) / 2.0;
        let d = refine_disparity(l_img, r_img, s, l, d0);
        let z = f * baseline / d.max(0.3);
        // Average the two views' boxes (right box shifted by the disparity) in the left view.
        let (bx0, bx1) = ((l.x0 + r.x0 + d) / 2.0, (l.x1 + r.x1 + d) / 2.0);
        let (by0, by1) = ((l.y0 + r.y0) / 2.0, (l.y1 + r.y1) / 2.0);
        let top = if l.score >= r.score { l } else { r };
        objs.push(make_obj(top.cls, top.score, z, bx0, by0, bx1, by1, true));
    }
    objs
}
