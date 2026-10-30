//! Stereo for the detector: matches YOLO's boxes in the two rectified views (a rectified pair: horizontal epipolar
//! lines, so one object's box has the same rows in both) and measures their depth from disparity, into head-frame
//! objects.
//!
//! Matching (match_views): every left/right pair is scored by how well its box edges agree (the same rows; the same
//! disparity at both sides), whether that disparity is plausible for the box's height (a box h px tall on something H m
//! tall has d = h B / H, H within its class's range: rules out pairing two people side by side the wrong way round, and
//! people in small pictures), and how alike the two views look there (ZNCC); then the best overall assignment
//! (Hungarian, OC Stereo's recipe with geometry added). Box edges cut off by a view's edge (calib::Valid) aren't the
//! object's, and are left out of all of it.
//!
//! Depth (patch_disparity): a box is half background, so instead of one window over all of it, a grid of small patches
//! over the object's middle (a person's head and torso) each find their own disparity, and the nearest cluster holding
//! a good share of them (the object, in front of what's behind it) is taken; fused with the box edges' disparity (the
//! silhouette's, noisier but never the background's) when the two agree. Each depth comes with its std dev for the
//! tracker: stereo error grows like z^2 / (f B).
use crate::calib::Valid;
use crate::tracker::assign;
use crate::v3::{V3, v3};

/// A box in one view (px).
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
    pub dist: f32,        // depth (along head -z) of the box's plane (m)
    pub sigma_z: f32,     // its std dev (m); 0 when seen in one view only
    pub f: f32,           // the views' focal length (px)
    pub corners: [V3; 4], // head frame at capture time: TL, TR, BR, BL
    pub clip: [bool; 4],  // edges x0, y0, x1, y1 cut off by the view's edge (stereo: in both views): not the object's
    pub stereo: bool,     // false: seen in one view only, corners are on the 1 m plane
    pub origin: V3,       // camera position (head frame) the corners were projected from
    pub disp: Disparity,  // how dist was measured, for the trace
}

/// A stereo object's disparities (px): from its box edges and from the patches (NaN: too few matched), with their std
/// devs, and the one dist comes from. All NaN for one-view objects.
#[derive(Clone, Copy, Debug)]
pub struct Disparity {
    pub edges: f32,
    pub sigma_edges: f32,
    pub patches: f32,
    pub sigma_patches: f32,
    pub used: f32,
    pub patch_stats: PatchStats,
}

pub const NO_DISPARITY: Disparity = Disparity {
    edges: f32::NAN,
    sigma_edges: f32::NAN,
    patches: f32::NAN,
    sigma_patches: f32::NAN,
    used: f32::NAN,
    patch_stats: PatchStats { tried: 0, outside: 0, flat: 0, weak: 0, unclear: 0, matched: 0, contrast: 0.0 },
};

const EDGE: f32 = 2.0; // a box edge within this (px) of the view's valid region's edge is cut off by it
const DY_MAX: f32 = 0.12; // max row mismatch of a pair's top and bottom edges (fraction of box height)
const DX_MAX: f32 = 0.25; // max mismatch of the disparities at a pair's two sides (fraction of box height)
const NCC_MIN: f32 = 0.0; // pairs that look less alike than this at their disparity aren't one object
const CLASS_COST: f32 = 0.3; // the views disagreeing on the class (cat/dog) still pair, at this extra cost
const MATCH_GATE: f32 = 1.5; // max pair cost
const D_MIN: f32 = 0.3; // min disparity (px): ~50 m at 320
const PATCH_NCC: f32 = 0.7; // patches matching worse than this are left out
const PATCH_TEXTURE: f32 = 25.0; // nor are patches flatter than this (gray level variance)
// Fitted to calibration recordings (calibrate.rs) of a person standing still at 1.5-6 m, at 320, 480 and 640:
const EDGE_PX: f32 = 1.2; // a box edge's disparity error (px): 0.4-0.9 px scatter, biases to +-0.9 px, any box width
// What depth is really good to, as disparity: frame-to-frame scatter is a tenth of a pixel, but a pose's bias (which
// part of the body the patches find, the box's cut) is 0.1-0.35 px at 480, about the same angle at every resolution.
const SIGMA_D_MIN_RAD: f32 = 0.002;
const ALIGN_D: i32 = 40; // alignment()'s disparity search (px): ~0.5 m at 480
const ALIGN_DY: i32 = 6; // and row search
const PATCH_SIGMA_MIN: f32 = 0.12; // the patches' disparity is never taken as better than this (px): pixel locking

/// Plausible heights (m) of what's visible of each class in a box: low ends allow for occlusion (a head and shoulders
/// above a couch, a cat behind a chair).
fn heights(cls: usize) -> (f32, f32) {
    match cls {
        0 => (0.3, 2.2),
        15 => (0.1, 0.6),
        _ => (0.15, 1.2),
    }
}

/// Which edges (x0, y0, x1, y1) of d lie on the edge of the view's valid region.
pub fn clipped(d: &Det, v: &Valid) -> [bool; 4] {
    // Along the middle 80% of each edge: a box's corner can be outside the fisheye's circle while its edges are fine.
    let (my, mx) = (0.1 * (d.y1 - d.y0), 0.1 * (d.x1 - d.x0));
    [
        d.x0 <= v.left(d.y0 + my, d.y1 - my) + EDGE,
        d.y0 <= v.top(d.x0 + mx, d.x1 - mx) + EDGE,
        d.x1 >= v.right(d.y0 + my, d.y1 - my) - EDGE,
        d.y1 >= v.bottom(d.x0 + mx, d.x1 - mx) - EDGE,
    ]
}

/// ZNCC of the left view's region (x0..x1, y0..y1, every `step` px) with the right view's at disparity d (and dy rows
/// up: 0 in a rectified pair); None if the region is too flat or too little of it is inside both views.
fn zncc(
    l: &[u8],
    r: &[u8],
    s: usize,
    (x0, y0, x1, y1): (i32, i32, i32, i32),
    step: usize,
    d: i32,
    dy: i32,
) -> Option<f32> {
    let s_ = s as i32;
    let (mut sl, mut sr, mut sll, mut srr, mut slr, mut n) = (0f32, 0f32, 0f32, 0f32, 0f32, 0);
    for y in (y0.max(dy.max(0))..y1.min(s_ + dy.min(0)).min(s_)).step_by(step) {
        for x in (x0.max(d.max(0))..x1.min(s_ + d.min(0)).min(s_)).step_by(step) {
            let (a, b) = (l[(y * s_ + x) as usize] as f32, r[((y - dy) * s_ + x - d) as usize] as f32);
            (sl, sr, sll, srr, slr) = (sl + a, sr + b, sll + a * a, srr + b * b, slr + a * b);
            n += 1;
        }
    }
    let nn = n as f32;
    let (vl, vr) = (sll - sl * sl / nn, srr - sr * sr / nn);
    if n < 9 || vl < PATCH_TEXTURE * nn || vr < PATCH_TEXTURE * nn {
        return None;
    }
    Some((slr - sl * sr / nn) / (vl * vr).sqrt())
}

/// A candidate left/right pair: its cost and the disparity its edges give, with how many edges that's from.
struct Pair {
    cost: f32,
    d: f32,
    edges: usize,
    spread: f32, // how far apart the two sides' disparities are (px), 0 with one
}

/// Scores pairing left box l with right box r (clip: their cut-off edges); None if they can't be one object.
#[allow(clippy::too_many_arguments)]
fn pair(l: &Det, r: &Det, lc: &[bool; 4], rc: &[bool; 4], imgs: [&[u8]; 2], s: usize, baseline: f32) -> Option<Pair> {
    let h = (l.y1 - l.y0).max(r.y1 - r.y0).max(1.0);
    // Same rows: the top and bottom edges that are the object's in both views.
    let dys: Vec<f32> =
        [(l.y0 - r.y0, 1), (l.y1 - r.y1, 3)].iter().filter(|e| !lc[e.1] && !rc[e.1]).map(|e| e.0).collect();
    let dy = if dys.is_empty() {
        ((l.y0 + l.y1) - (r.y0 + r.y1)).abs() / 2.0 // cut off top and bottom: the centers, for lack of better
    } else {
        dys.iter().map(|d| d.abs()).sum::<f32>() / dys.len() as f32
    } / h;
    // The same disparity at both sides, from the sides that are the object's in both views.
    let dxs: Vec<f32> =
        [(l.x0 - r.x0, 0), (l.x1 - r.x1, 2)].iter().filter(|e| !lc[e.1] && !rc[e.1]).map(|e| e.0).collect();
    if dy > DY_MAX || dxs.is_empty() {
        return None;
    }
    let d = dxs.iter().sum::<f32>() / dxs.len() as f32;
    let dx = if dxs.len() == 2 { (dxs[0] - dxs[1]).abs() / h } else { 0.0 };
    // d = h B / H for what's visible of it, H within the class's range (either view's class).
    let ((lo0, hi0), (lo1, hi1)) = (heights(l.cls), heights(r.cls));
    let (hmin, hmax) = (lo0.min(lo1), hi0.max(hi1));
    if dx > DX_MAX || d < D_MIN.max(0.8 * h * baseline / hmax) || d > 1.25 * h * baseline / hmin {
        return None;
    }
    // Alike at that disparity: the box's middle, about 16 x 16 samples, the best of +-2 px.
    let (w, bh) = (l.x1 - l.x0, l.y1 - l.y0);
    let region = ((l.x0 + 0.2 * w) as i32, (l.y0 + 0.2 * bh) as i32, (l.x1 - 0.2 * w) as i32, (l.y1 - 0.2 * bh) as i32);
    let step = ((0.6 * w.max(bh)) as usize / 16).max(1);
    let ncc =
        (-2..=2).filter_map(|k| zncc(imgs[0], imgs[1], s, region, step, d.round() as i32 + k, 0)).reduce(f32::max);
    if ncc.is_some_and(|c| c < NCC_MIN) {
        return None;
    }
    let looks = 0.5 * (1.0 - ncc.unwrap_or(0.5)); // too flat to tell: neutral
    let class = if l.cls != r.cls { CLASS_COST } else { 0.0 };
    let spread = dx * h;
    Some(Pair { cost: 2.0 * dy + dx + looks + class, d, edges: dxs.len(), spread })
}

/// What became of patch_disparity()'s patches, for the trace: how many it tried, and of those how many were partly
/// outside a view, too flat (in either view), matched too weakly (best ZNCC under PATCH_NCC), had no clear peak (another
/// as good 2+ px away, or at the search's end), or matched; and the tried ones' mean gray level std dev in the left view.
#[derive(Clone, Copy, Debug, Default)]
pub struct PatchStats {
    pub tried: u16,
    pub outside: u16,
    pub flat: u16,
    pub weak: u16,
    pub unclear: u16,
    pub matched: u16,
    pub contrast: f32,
}

/// alignment()'s patches: tried, too flat, best ZNCC under 0.8, best at the row (dy) or disparity (d) search's end,
/// another as good 2+ px away, or found.
#[derive(Clone, Copy, Debug, Default)]
pub struct AlignStats {
    pub tried: u32,
    pub flat: u32,
    pub weak: u32,
    pub edge_dy: u32,
    pub edge_d: u32,
    pub unclear: u32,
    pub found: u32,
}

/// The gray level std dev of the region (x0..x1, y0..y1) of img.
fn std_dev(img: &[u8], s: usize, (x0, y0, x1, y1): (i32, i32, i32, i32)) -> f32 {
    let v: Vec<f32> = (y0.max(0)..y1.min(s as i32))
        .flat_map(|y| (x0.max(0)..x1.min(s as i32)).map(move |x| img[y as usize * s + x as usize] as f32))
        .collect();
    let m = v.iter().sum::<f32>() / v.len().max(1) as f32;
    (v.iter().map(|a| (a - m) * (a - m)).sum::<f32>() / v.len().max(1) as f32).sqrt()
}

/// The object's disparity in left box `l` near `d0` (within lo..hi), from a grid of small patches over its middle, each
/// matched along its row by ZNCC: the nearest cluster holding at least half as many patches as the largest one, its
/// median, and that median's std dev; None if too few patches matched. And what became of the patches.
fn patch_disparity(imgs: [&[u8]; 2], s: usize, l: &Det, d0: f32, lo: f32, hi: f32) -> (Option<(f32, f32)>, PatchStats) {
    let (w, h) = (l.x1 - l.x0, l.y1 - l.y0);
    // A person's head and torso (not the gaps by the arms, or the legs' background); a pet's middle.
    let (fx, fy) = if l.cls == 0 { ([0.3, 0.7], [0.08, 0.6]) } else { ([0.2, 0.8], [0.2, 0.8]) };
    let (rx0, rx1, ry0, ry1) = (l.x0 + fx[0] * w, l.x0 + fx[1] * w, l.y0 + fy[0] * h, l.y0 + fy[1] * h);
    let rad = (((rx1 - rx0).min(ry1 - ry0) / 4.0).round() as i32).clamp(2, 5);
    let side = (2 * rad + 1) as f32;
    let (nx, ny) = ((((rx1 - rx0) / side) as usize).clamp(1, 6), (((ry1 - ry0) / side) as usize).clamp(1, 8));
    let reach = (0.4 * d0).max(3.0);
    let (k0, k1) = ((d0 - reach).max(lo).floor() as i32, (d0 + reach).min(hi).ceil() as i32);
    let mut st = PatchStats::default();
    if k1 - k0 < 2 {
        return (None, st);
    }
    let mut ds = Vec::new();
    let mut score = vec![0f32; (k1 - k0 + 1) as usize];
    for gy in 0..ny {
        for gx in 0..nx {
            let cx = (rx0 + (gx as f32 + 0.5) * (rx1 - rx0) / nx as f32) as i32;
            let cy = (ry0 + (gy as f32 + 0.5) * (ry1 - ry0) / ny as f32) as i32;
            let patch = (cx - rad, cy - rad, cx + rad + 1, cy + rad + 1);
            st.tried += 1;
            st.contrast += std_dev(imgs[0], s, patch);
            // The whole patch, in both views, or not at all (a partial one would score on fewer pixels).
            if patch.0 - k1 < 0
                || patch.2 - k0 > s as i32
                || patch.0 < 0
                || patch.2 > s as i32
                || patch.1 < 0
                || patch.3 > s as i32
            {
                st.outside += 1;
                continue;
            }
            let mut flat = false;
            for (i, k) in (k0..=k1).enumerate() {
                match zncc(imgs[0], imgs[1], s, patch, 1, k, 0) {
                    Some(c) => score[i] = c,
                    None => {
                        flat = true;
                        break;
                    }
                }
            }
            if flat {
                st.flat += 1;
                continue;
            }
            let best = (0..score.len()).max_by(|&a, &b| score[a].total_cmp(&score[b])).unwrap();
            // A clear peak, inside the search range: nothing 2+ steps away nearly as good (repetitive texture)
            let unique = (0..score.len()).filter(|&i| i.abs_diff(best) >= 2).all(|i| score[i] < score[best] - 0.1);
            if score[best] < PATCH_NCC {
                st.weak += 1;
                continue;
            }
            if best == 0 || best == score.len() - 1 || !unique {
                st.unclear += 1;
                continue;
            }
            st.matched += 1;
            let (a, b, c) = (score[best - 1], score[best], score[best + 1]);
            let sub = ((a - c) / (2.0 * (a - 2.0 * b + c) - 1e-9)).clamp(-0.5, 0.5);
            ds.push((k0 + best as i32) as f32 + sub);
        }
    }
    st.contrast /= st.tried.max(1) as f32;
    (nearest_cluster(&mut ds), st)
}

/// For checking the rectification (the calibration app): small textured patches over the whole left view, each found in
/// the right view by ZNCC over disparities -2..ALIGN_D and rows -ALIGN_DY..ALIGN_DY. A rectified pair has no row offset
/// anywhere, and no disparity for anything far away, so what's left is how the views are misaligned. Per patch found
/// clearly: its center (px), disparity and row offset (px, sub-pixel) and ZNCC. And why the others weren't (for the log):
/// AlignStats.
pub fn alignment(l: &[u8], r: &[u8], s: usize) -> (Vec<[f32; 5]>, AlignStats) {
    const R: i32 = 4; // 9 x 9
    const STEP: usize = 24;
    let (ds, dys) = (-2..=ALIGN_D, -ALIGN_DY..=ALIGN_DY);
    let (mut out, mut st) = (Vec::new(), AlignStats::default());
    for cy in (STEP / 2..s - STEP / 2).step_by(STEP) {
        for cx in (STEP / 2..s - STEP / 2).step_by(STEP) {
            let (cx, cy) = (cx as i32, cy as i32);
            let patch = (cx - R, cy - R, cx + R + 1, cy + R + 1);
            st.tried += 1;
            if std_dev(l, s, patch) < 8.0 {
                st.flat += 1;
                continue; // flat: nothing to find
            }
            // score[dy][d]; None outside the right view or flat there
            let score: Vec<Vec<f32>> = dys
                .clone()
                .map(|dy| {
                    ds.clone()
                        .map(|d| {
                            let inside =
                                cx - R - d >= 0 && cx + R - d < s as i32 && cy - R - dy >= 0 && cy + R - dy < s as i32;
                            if inside { zncc(l, r, s, patch, 1, d, dy).unwrap_or(-1.0) } else { -1.0 }
                        })
                        .collect()
                })
                .collect();
            let (mut bi, mut bj) = (0, 0);
            for (i, row) in score.iter().enumerate() {
                for (j, &v) in row.iter().enumerate() {
                    if v > score[bi][bj] {
                        (bi, bj) = (i, j);
                    }
                }
            }
            let b = score[bi][bj];
            let clear = score.iter().enumerate().all(|(i, row)| {
                row.iter().enumerate().all(|(j, &v)| (i.abs_diff(bi) < 2 && j.abs_diff(bj) < 2) || v < b - 0.1)
            });
            let rejected = if b < 0.8 {
                Some(&mut st.weak)
            } else if bi == 0 || bi == score.len() - 1 {
                Some(&mut st.edge_dy)
            } else if bj == 0 || bj == score[0].len() - 1 {
                Some(&mut st.edge_d)
            } else if !clear {
                Some(&mut st.unclear)
            } else {
                None
            };
            if let Some(n) = rejected {
                *n += 1;
                continue;
            }
            st.found += 1;
            let sub = |a: f32, c: f32| ((a - c) / (2.0 * (a - 2.0 * b + c) - 1e-9)).clamp(-0.5, 0.5);
            let d = (bj as i32 - 2) as f32 + sub(score[bi][bj - 1], score[bi][bj + 1]);
            let dy = (bi as i32 - ALIGN_DY) as f32 + sub(score[bi - 1][bj], score[bi + 1][bj]);
            out.push([cx as f32, cy as f32, d, dy, b]);
        }
    }
    (out, st)
}

/// Of a set of disparities, the nearest (largest) cluster holding at least half as many as the largest cluster: its
/// median and that median's std dev (px). None with fewer than 3 in it.
fn nearest_cluster(ds: &mut [f32]) -> Option<(f32, f32)> {
    ds.sort_by(|a, b| b.total_cmp(a));
    // An object's own depth spans a few % of its distance: as disparity, a few % of it.
    let width = |c: f32| (0.08 * c).max(0.5);
    let near = |c: f32| ds.iter().filter(|&&d| (d - c).abs() <= width(c)).count();
    let most = ds.iter().map(|&d| near(d)).max()?;
    let first = *ds.iter().find(|&&d| near(d) * 2 >= most && near(d) >= 3)?;
    let median = |c: f32| {
        let m: Vec<f32> = ds.iter().copied().filter(|&d| (d - c).abs() <= width(c)).collect();
        (m[m.len() / 2], m)
    };
    let (c, _) = median(first);
    let (c, m) = median(c); // centered on the cluster
    if m.len() < 3 {
        return None;
    }
    let mut dev: Vec<f32> = m.iter().map(|d| (d - c).abs()).collect();
    dev.sort_by(f32::total_cmp);
    let sigma = (1.4826 * dev[dev.len() / 2]).max(0.25) / (m.len() as f32).sqrt();
    Some((c, sigma.max(PATCH_SIGMA_MIN)))
}

/// Matches the left view's boxes to the right view's and measures each pair's depth; boxes seen in one view only come
/// out on the 1 m plane from their camera, stereo false. imgs: the two S x S views; f (px) and baseline (m) of the pair;
/// origins: the cameras in the head frame; valid: where each view has image.
#[allow(clippy::too_many_arguments)]
pub fn match_views(
    views: &[Vec<Det>; 2],
    imgs: [&[u8]; 2],
    s: usize,
    f: f32,
    baseline: f32,
    origins: [V3; 2],
    valid: &[Valid; 2],
) -> Vec<Obj> {
    let half = s as f32 / 2.0;
    let make_obj = |o: V3, cls, score, z: f32, sigma_z, b: [f32; 4], clip, disp: Option<Disparity>| {
        let (us, vs) = ([b[0], b[2], b[2], b[0]], [b[1], b[1], b[3], b[3]]);
        let corner = |k: usize| o + v3((us[k] - half) / f * z, -(vs[k] - half) / f * z, -z);
        let corners = [corner(0), corner(1), corner(2), corner(3)];
        let (stereo, disp) = (disp.is_some(), disp.unwrap_or(NO_DISPARITY));
        Obj { cls, score, dist: z, sigma_z, f, corners, clip, stereo, origin: o, disp }
    };
    let clips: [Vec<[bool; 4]>; 2] = [0, 1].map(|v| views[v].iter().map(|d| clipped(d, &valid[v])).collect());
    let pairs: Vec<Vec<Option<Pair>>> = (0..views[0].len())
        .map(|i| {
            (0..views[1].len())
                .map(|j| pair(&views[0][i], &views[1][j], &clips[0][i], &clips[1][j], imgs, s, baseline))
                .collect()
        })
        .collect();
    let cost: Vec<Vec<f32>> =
        pairs.iter().map(|row| row.iter().map(|p| p.as_ref().map_or(f32::INFINITY, |p| p.cost)).collect()).collect();
    let assigned = assign(&cost, MATCH_GATE);

    let mut objs = Vec::new();
    let mut right_used = vec![false; views[1].len()];
    for (i, m) in assigned.iter().enumerate() {
        let (l, lc) = (&views[0][i], &clips[0][i]);
        let Some(j) = *m else {
            objs.push(make_obj(origins[0], l.cls, l.score, 1.0, 0.0, [l.x0, l.y0, l.x1, l.y1], *lc, None));
            continue;
        };
        right_used[j] = true;
        let (r, rc, p) = (&views[1][j], &clips[1][j], pairs[i][j].as_ref().unwrap());
        let ((lo0, hi0), (lo1, hi1)) = (heights(l.cls), heights(r.cls)); // either view's class, as in pair()
        let (hmin, hmax) = (lo0.min(lo1), hi0.max(hi1));
        let h = (l.y1 - l.y0).max(r.y1 - r.y0);
        // At least what the two sides disagree by: a seated person's sides gave 15 and 29 px when their patches all
        // failed (true: 10), and the edges alone put them at half their distance.
        let edge_sigma = (EDGE_PX / (p.edges as f32).sqrt()).max(p.spread / 2.0);
        let (lo, hi) = (D_MIN.max(0.8 * h * baseline / hmax), 1.25 * h * baseline / hmin);
        let (patches, patch_stats) = patch_disparity(imgs, s, l, p.d, lo, hi);
        let (d, sigma_d) = match patches {
            // Both agree: weighted by their noise. Else the patches, unless they're few (the box edges then).
            Some((dp, sp)) => {
                let (wp, we) = (1.0 / (sp * sp), 1.0 / (edge_sigma * edge_sigma));
                if (dp - p.d).abs() < 3.0 * (sp * sp + edge_sigma * edge_sigma).sqrt() {
                    ((wp * dp + we * p.d) / (wp + we), (wp + we).sqrt().recip())
                } else {
                    (dp, sp)
                }
            }
            None => (p.d, edge_sigma),
        };
        let d = d.max(D_MIN);
        // z = fB/d is convex in d: noise makes it read far, by a factor 1 + (sigma_d / d)^2 on average.
        let z = f * baseline / d / (1.0 + (sigma_d / d).powi(2));
        // How much farther it reads with d one sigma less: z^2 sigma_d / (f B) for a small sigma_d, and the far side's
        // much longer reach when it isn't (22 +- 7 px read 1.0 m for a person 2.1 m away).
        let sd = sigma_d.max(SIGMA_D_MIN_RAD * f);
        let sigma_z = f * baseline / (d - sd).max(D_MIN) - f * baseline / d;
        // One box in the left view: the two views' edges (the right one's shifted by d), each from the views where it
        // isn't cut off.
        let edge = |a: f32, ac: bool, b: f32, bc: bool| match (ac, bc) {
            (false, false) => ((a + b) / 2.0, false),
            (false, true) => (a, false),
            (true, false) => (b, false),
            (true, true) => (a, true),
        };
        let e = [
            edge(l.x0, lc[0], r.x0 + d, rc[0]),
            edge(l.y0, lc[1], r.y0, rc[1]),
            edge(l.x1, lc[2], r.x1 + d, rc[2]),
            edge(l.y1, lc[3], r.y1, rc[3]),
        ];
        let top = if l.score >= r.score { l } else { r };
        let score = (l.score + r.score) / 2.0;
        let (dp, sp) = patches.unwrap_or((f32::NAN, f32::NAN));
        let disp =
            Disparity { edges: p.d, sigma_edges: edge_sigma, patches: dp, sigma_patches: sp, used: d, patch_stats };
        objs.push(make_obj(origins[0], top.cls, score, z, sigma_z, e.map(|e| e.0), e.map(|e| e.1), Some(disp)));
    }
    for (j, r) in views[1].iter().enumerate().filter(|(j, _)| !right_used[*j]) {
        objs.push(make_obj(origins[1], r.cls, r.score, 1.0, 0.0, [r.x0, r.y0, r.x1, r.y1], clips[1][j], None));
    }
    objs
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: usize = 160;
    const F: f32 = 67.0; // 100 deg at 160
    const B: f32 = 0.1056;

    /// Deterministic texture.
    fn texture(seed: u32, n: usize) -> Vec<u8> {
        let mut seed = seed;
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 24) as u8
            })
            .collect()
    }

    /// A stereo pair: a textured background at disparity `bg` with a textured rectangle (x0..x1, y0..y1 in the left
    /// view) in front of it at disparity `fg`; whole pixels.
    fn scene(bg: i32, fg: i32, (x0, y0, x1, y1): (i32, i32, i32, i32)) -> (Vec<u8>, Vec<u8>) {
        let (back, front) = (texture(1, S * S * 2), texture(2, S * S * 2));
        let (mut l, mut r) = (vec![0u8; S * S], vec![0u8; S * S]);
        for y in 0..S as i32 {
            for x in 0..S as i32 {
                let i = (y * S as i32 + x) as usize;
                // A pixel's texture is fixed to the surface: background by its left-view x, object likewise.
                let inside = |lx: i32| lx >= x0 && lx < x1 && y >= y0 && y < y1;
                let tex = |t: &[u8], lx: i32| t[(y * 2 * S as i32 + lx + S as i32 / 2) as usize];
                l[i] = if inside(x) { tex(&front, x) } else { tex(&back, x) };
                r[i] = if inside(x + fg) { tex(&front, x + fg) } else { tex(&back, x + bg) };
            }
        }
        (l, r)
    }

    fn all_valid() -> Valid {
        Valid { rows: vec![(0.0, S as f32); S], cols: vec![(0.0, S as f32); S] }
    }

    fn det(x0: f32, y0: f32, x1: f32, y1: f32) -> Det {
        Det { x0, y0, x1, y1, score: 0.8, cls: 0 }
    }

    #[test]
    fn depth_is_the_object_not_its_background() {
        // A person-sized box at 8 px disparity (about 0.9 m at this resolution) with background at 2 px filling the
        // box's sides; the box's edges are 1.5 px off.
        let (l, r) = scene(2, 8, (60, 40, 90, 120));
        let views = [vec![det(55.0, 38.0, 95.0, 122.0)], vec![det(48.5, 38.0, 88.5, 122.0)]];
        let objs = match_views(&views, [&l, &r], S, F, B, [V3::default(); 2], &[all_valid(), all_valid()]);
        assert_eq!(objs.len(), 1);
        let o = &objs[0];
        let z = F * B / 8.0;
        assert!(o.stereo && (o.dist - z).abs() < 0.03 * z, "{} vs {z}", o.dist);
        assert!(o.sigma_z > 0.0 && o.sigma_z < 0.05 * z);
    }

    #[test]
    fn edges_that_disagree_are_uncertain() {
        // Nothing to match patches on, so depth is the box edges': the sides' disparities 8 and 18 px, then 13 and 13.
        let img = vec![0u8; S * S];
        let sigma = |r: Det| {
            let views = [vec![det(30.0, 40.0, 70.0, 120.0)], vec![r]];
            let o = &match_views(&views, [&img, &img], S, F, B, [V3::default(); 2], &[all_valid(), all_valid()])[0];
            assert!(o.stereo && o.disp.patches.is_nan());
            o.disp.sigma_edges
        };
        assert_eq!(sigma(det(22.0, 40.0, 52.0, 120.0)), 5.0);
        let views = [vec![det(30.0, 40.0, 70.0, 120.0)], vec![det(22.0, 40.0, 52.0, 120.0)]];
        let o = &match_views(&views, [&img, &img], S, F, B, [V3::default(); 2], &[all_valid(), all_valid()])[0];
        assert!((o.sigma_z - (F * B / 8.0 - F * B / 13.0)).abs() < 1e-4, "the far side: {}", o.sigma_z);
        assert!(sigma(det(17.0, 40.0, 57.0, 120.0)) < 1.0);
    }

    #[test]
    fn side_by_side_pair_up_the_right_way() {
        // Two people side by side 10 px apart at 6 px disparity: pairing them crosswise (disparity 6 + 40 or 6 - 40)
        // is implausible for their height; greedy matching by size alone could take it.
        let l = texture(3, S * S);
        let r: Vec<u8> = (0..S * S).map(|i| if i % S + 6 < S { l[i + 6] } else { 0 }).collect(); // all at 6 px
        let left = vec![det(30.0, 40.0, 60.0, 120.0), det(70.0, 40.0, 100.0, 120.0)];
        let right = vec![det(64.0, 40.0, 94.0, 120.0), det(24.0, 40.0, 54.0, 120.0)];
        let objs = match_views(&[left, right], [&l, &r], S, F, B, [V3::default(); 2], &[all_valid(), all_valid()]);
        let stereo: Vec<&Obj> = objs.iter().filter(|o| o.stereo).collect();
        assert_eq!(stereo.len(), 2);
        // 80 px tall at 6 px: 1.4 m; crosswise 46 px would be 0.18 m tall.
        assert!(
            stereo.iter().all(|o| (o.dist - F * B / 6.0).abs() < 0.3),
            "{:?}",
            stereo.iter().map(|o| o.dist).collect::<Vec<_>>()
        );
    }

    #[test]
    fn cut_off_edges_dont_give_the_disparity() {
        // The left box is cut off by the view's left edge (x0 = 0); its x1 gives the disparity.
        let valid = all_valid();
        let (l, r) = scene(2, 6, (0, 40, 30, 120));
        let views = [vec![det(0.0, 40.0, 30.0, 120.0)], vec![det(0.0, 40.0, 24.0, 120.0)]];
        assert_eq!(clipped(&views[0][0], &valid), [true, false, false, false]);
        let objs = match_views(&views, [&l, &r], S, F, B, [V3::default(); 2], &[valid.clone(), valid]);
        assert!(objs[0].stereo && (objs[0].dist - F * B / 6.0).abs() < 0.1 && objs[0].clip[0] && !objs[0].clip[2]);
    }

    #[test]
    fn one_view_boxes_come_out_direction_only() {
        let img = texture(4, S * S);
        let views = [vec![det(10.0, 40.0, 40.0, 120.0)], vec![det(100.0, 20.0, 130.0, 60.0)]];
        let objs =
            match_views(&views, [&img, &img], S, F, B, [V3::default(), v3(B, 0.0, 0.0)], &[all_valid(), all_valid()]);
        assert!(objs.len() == 2 && objs.iter().all(|o| !o.stereo && o.dist == 1.0));
        assert_eq!(objs[1].origin, v3(B, 0.0, 0.0));
    }

    #[test]
    fn alignment_finds_how_the_views_are_off() {
        // The right view is the left one moved 5 px left and 2 px up (r(x, y) = l(x + 5, y + 2)): every patch says d 5,
        // dy 2.
        let l = texture(5, S * S);
        let r: Vec<u8> = (0..S * S)
            .map(|i| {
                let (x, y) = (i % S + 5, i / S + 2);
                if x < S && y < S { l[y * S + x] } else { 0 }
            })
            .collect();
        let (a, st) = alignment(&l, &r, S);
        assert_eq!(st.found as usize, a.len());
        assert!(a.len() > 20);
        assert!(a.iter().all(|p| (p[2] - 5.0).abs() < 0.2 && (p[3] - 2.0).abs() < 0.2 && p[4] > 0.95), "{:?}", &a[..3]);
    }

    #[test]
    fn nearest_cluster_of_patches() {
        // 6 patches on the object at 8, 9 on the wall at 3, an outlier at 20: the object (nearest with half the most).
        let mut ds = vec![8.0, 8.1, 7.9, 8.05, 7.95, 8.0, 3.0, 3.1, 2.9, 3.0, 3.05, 2.95, 3.0, 3.1, 2.9, 20.0];
        let (d, sigma) = nearest_cluster(&mut ds).unwrap();
        assert!((d - 8.0).abs() < 0.06 && (PATCH_SIGMA_MIN..0.2).contains(&sigma));
        // 2 on the object, 9 on the wall: not enough of the object, the wall
        let mut ds = vec![8.0, 8.1, 3.0, 3.1, 2.9, 3.0, 3.05, 2.95, 3.0, 3.1, 2.9];
        assert!((nearest_cluster(&mut ds).unwrap().0 - 3.0).abs() < 0.1);
        assert!(nearest_cluster(&mut [1.0, 5.0]).is_none());
    }
}
