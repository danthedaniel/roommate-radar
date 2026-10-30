//! Camera calibration (SLAM and side) from the factory files (/persist, which the driver copies to shm::CALIB_DIR):
//! xrservice.json (per camera: Kannala-Brandt intrinsics, extrinsics) and device_config.json (`head`, and XRService's
//! `cv/cad_from_cal`), chained into head_from_cam: head <- CAD (`head`) <- cal/cam0 frame (`cad_from_cal`) <- cam
//! (extrinsics, mm). Every frame there is plus_x / plus_z axes (+ position) in its parent, with plus_y = plus_z x
//! plus_x. Camera frames are OpenCV-style (x right, y down, z forward). And the rectification it gives.
use serde_json::Value;

#[derive(Clone, Copy, Debug, Default)]
pub struct CamCal {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub k: [f32; 4],
    pub r: [f32; 9], // head_from_cam rotation, row-major; camera frame is OpenCV (x right, y down, z fwd)
    pub t: [f32; 3], // camera position in head frame (m)
    pub w: usize,    // image size (px)
    pub h: usize,
}

/// parent_from_child: rotation (row-major), translation
type Pose = ([[f64; 3]; 3], [f64; 3]);

fn vec3(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?])
}

fn frame(d: &Value, scale: f64) -> Option<Pose> {
    let (x, z, p) = (vec3(&d["plus_x"])?, vec3(&d["plus_z"])?, vec3(&d["position"])?);
    let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
    Some(([0, 1, 2].map(|i| [x[i], y[i], z[i]]), p.map(|v| v * scale)))
}

fn compose(a: &Pose, b: &Pose) -> Pose {
    let r = [0, 1, 2].map(|i| [0, 1, 2].map(|j| (0..3).map(|k| a.0[i][k] * b.0[k][j]).sum()));
    let t = [0, 1, 2].map(|i| (0..3).map(|k| a.0[i][k] * b.1[k]).sum::<f64>() + a.1[i]);
    (r, t)
}

fn inverse(a: &Pose) -> Pose {
    let rt = [0, 1, 2].map(|i| [0, 1, 2].map(|j| a.0[j][i]));
    (rt, [0, 1, 2].map(|i| -(0..3).map(|k| rt[i][k] * a.1[k]).sum::<f64>()))
}

/// Camera `name` (shm::CAM_NAMES, shm::SIDE_NAMES) from the calibration files in `dir`.
pub fn load(dir: &str, name: &str) -> Result<CamCal, String> {
    let json = |f: &str| -> Result<Value, String> {
        let text = std::fs::read_to_string(format!("{dir}/{f}")).map_err(|e| format!("{dir}/{f}: {e}"))?;
        serde_json::from_str(&text).map_err(|e| format!("{dir}/{f}: {e}"))
    };
    let (dev, xrs) = (json("device_config.json")?, json("xrservice.json")?);
    let bad = |what: &str| format!("{dir}: no {what}");
    let cam = xrs["cameras"]
        .as_array()
        .and_then(|cams| cams.iter().find(|c| c["sourceCamera"] == name))
        .ok_or_else(|| bad(&format!("camera {name}")))?;
    let kb = cam["intrinsics"]
        .as_array()
        .and_then(|i| i.iter().find(|i| i["cameraModel"] == "kb"))
        .ok_or_else(|| bad(&format!("kb intrinsics for {name}")))?;
    let head_from_cad = inverse(&frame(&dev["head"], 1.0).ok_or_else(|| bad("head"))?);
    let cad_from_cal = frame(&dev["cv"]["cad_from_cal"], 1.0).ok_or_else(|| bad("cv/cad_from_cal"))?;
    let cal_from_cam = frame(&cam["extrinsics"], 1e-3).ok_or_else(|| bad(&format!("extrinsics for {name}")))?;
    let (r, t) = compose(&compose(&head_from_cad, &cad_from_cal), &cal_from_cam);
    let dot = |i: usize, j: usize| (0..3).map(|k| r[i][k] * r[j][k]).sum::<f64>();
    if (0..3).any(|i| (0..3).any(|j| (dot(i, j) - if i == j { 1.0 } else { 0.0 }).abs() > 1e-3)) {
        return Err(format!("calibration: {name} rotation not orthonormal"));
    }
    let k = |n: &str| kb[n].as_f64().map(|v| v as f32).ok_or_else(|| bad(&format!("{n} for {name}")));
    Ok(CamCal {
        fx: k("fx")?,
        fy: k("fy")?,
        cx: k("cx")?,
        cy: k("cy")?,
        k: [k("k1")?, k("k2")?, k("k3")?, k("k4")?],
        r: std::array::from_fn(|i| r[i / 3][i % 3] as f32),
        t: t.map(|v| v as f32),
        w: cam["width"].as_u64().ok_or_else(|| bad(&format!("width for {name}")))? as usize,
        h: cam["height"].as_u64().ok_or_else(|| bad(&format!("height for {name}")))? as usize,
    })
}

/// The rotation by w (rad: axis times angle, Rodrigues), row-major.
pub fn rotation(w: [f32; 3]) -> [f32; 9] {
    let t = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
    if t < 1e-12 {
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    }
    let (x, y, z) = (w[0] / t, w[1] / t, w[2] / t);
    let (s, c) = t.sin_cos();
    let k = 1.0 - c;
    [
        c + x * x * k,
        x * y * k - z * s,
        x * z * k + y * s,
        y * x * k + z * s,
        c + y * y * k,
        y * z * k - x * s,
        z * x * k - y * s,
        z * y * k + x * s,
        c + z * z * k,
    ]
}

/// The camera turned by w (rad, head frame axes): what it sees moves by w in its rectified view.
pub fn rotated(c: &CamCal, w: [f32; 3]) -> CamCal {
    let r = rotation(w);
    let m = |i: usize, j: usize| (0..3).map(|k| r[i * 3 + k] * c.r[k * 3 + j]).sum::<f32>();
    CamCal { r: std::array::from_fn(|n| m(n / 3, n % 3)), ..*c }
}

// ---------------------------------------------------------------------------------------------
// Rectification: virtual pinhole camera at the SLAM camera's position, axes aligned with the head
// (x right, y down, looking along head -z). Both views share orientation -> rectified stereo pair.

/// A rectified view's valid region: per row the valid columns [first, last + 1), per column the valid rows; (S, 0)
/// where there are none. The fisheye's image is convex, so that's all of it.
#[derive(Clone, Debug)]
pub struct Valid {
    pub rows: Vec<(f32, f32)>,
    pub cols: Vec<(f32, f32)>,
}

impl Valid {
    /// Whether (x, y) (px) is at least `margin` px inside the region.
    pub fn inside(&self, x: f32, y: f32, margin: f32) -> bool {
        let s = self.rows.len() as f32;
        if !(x >= 0.0 && y >= 0.0 && x < s && y < s) {
            return false;
        }
        let (r, c) = (self.rows[y as usize], self.cols[x as usize]);
        x - r.0 >= margin && r.1 - x >= margin && y - c.0 >= margin && c.1 - y >= margin
    }

    /// The region's left (right) edge, at its furthest in (out) over rows y0..y1 (px): a box edge there is cut off.
    pub fn left(&self, y0: f32, y1: f32) -> f32 {
        self.span(&self.rows, y0, y1).0
    }
    pub fn right(&self, y0: f32, y1: f32) -> f32 {
        self.span(&self.rows, y0, y1).1
    }
    pub fn top(&self, x0: f32, x1: f32) -> f32 {
        self.span(&self.cols, x0, x1).0
    }
    pub fn bottom(&self, x0: f32, x1: f32) -> f32 {
        self.span(&self.cols, x0, x1).1
    }

    /// Over lines a..b of `lines`: the largest start and the smallest end.
    fn span(&self, lines: &[(f32, f32)], a: f32, b: f32) -> (f32, f32) {
        let n = lines.len();
        let (a, b) = ((a.max(0.0) as usize).min(n - 1), (b.max(0.0) as usize).min(n - 1));
        lines[a..=b.max(a)].iter().fold((0.0f32, n as f32), |(s, e), l| (s.max(l.0), e.min(l.1)))
    }
}

pub struct Rectifier {
    s: usize,
    stride: usize, // the camera frames'
    pub f: f32,
    map: Vec<[f32; 2]>, // (src x, src y) per output pixel; x < 0 = outside the fisheye
}

impl Rectifier {
    pub fn new(c: &CamCal, size: usize, hfov_deg: f32, stride: usize) -> Rectifier {
        let s = size as f32;
        let f = (s / 2.0) / (hfov_deg.to_radians() / 2.0).tan();
        let mut map = vec![[-1.0, 0.0]; size * size];
        for v in 0..size {
            for u in 0..size {
                // Ray in head frame for virtual pixel center (u, v).
                let (hx, hy, hz) = ((u as f32 + 0.5 - s / 2.0) / f, -(v as f32 + 0.5 - s / 2.0) / f, -1.0);
                // cam_from_head = R^T
                let r = &c.r;
                let x = r[0] * hx + r[3] * hy + r[6] * hz;
                let y = r[1] * hx + r[4] * hy + r[7] * hz;
                let z = r[2] * hx + r[5] * hy + r[8] * hz;
                let rr = (x * x + y * y).sqrt();
                let th = rr.atan2(z);
                if th > 88f32.to_radians() {
                    continue;
                }
                let t2 = th * th;
                let thd = th * (1.0 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3]))));
                let sc = if rr > 1e-9 { thd / rr } else { 0.0 };
                let m = [c.fx * x * sc + c.cx, c.fy * y * sc + c.cy];
                if m[0] >= 0.0 && m[1] >= 0.0 && m[0] <= (c.w - 2) as f32 && m[1] <= (c.h - 2) as f32 {
                    map[v * size + u] = m;
                }
            }
        }
        Rectifier { s: size, stride, f, map }
    }

    /// Where the view has image (inside the fisheye and the sensor).
    pub fn valid(&self) -> Valid {
        let s = self.s;
        let none: (f32, f32) = (s as f32, 0.0);
        let (mut rows, mut cols) = (vec![none; s], vec![none; s]);
        for (i, m) in self.map.iter().enumerate() {
            let (u, v) = (i % s, i / s);
            if m[0] >= 0.0 {
                let (r, c) = (&mut rows[v], &mut cols[u]);
                (r.0, r.1) = (r.0.min(u as f32), r.1.max(u as f32 + 1.0));
                (c.0, c.1) = (c.0.min(v as f32), c.1.max(v as f32 + 1.0));
            }
        }
        Valid { rows, cols }
    }

    /// The S x S rectified view of a camera frame (Y plane, `stride` as given to new()).
    pub fn apply(&self, src: &[u8], dst: &mut [u8]) {
        for (d, &[sx, sy]) in dst[..self.s * self.s].iter_mut().zip(&self.map) {
            if sx < 0.0 {
                *d = 0;
                continue;
            }
            let (x, y) = (sx as usize, sy as usize);
            let (ax, ay) = (sx - x as f32, sy - y as f32);
            let st = self.stride;
            let p = &src[y * st + x..];
            let at = |i: usize| p[i] as f32;
            let top = at(0) + ax * (at(1) - at(0));
            let bot = at(st) + ax * (at(st + 1) - at(st));
            *d = (top + ay * (bot - top) + 0.5) as u8;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chains_head_cad_and_camera_frames() {
        // Head turned 90 deg about z in CAD; the camera 10 mm along CAD x, axes as CAD's: in the head frame it's at
        // -10 mm y, and its x axis is the head's -y.
        let dir = std::env::temp_dir().join(format!("roommate-radar-calib-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let ident = r#"{"plus_x": [1, 0, 0], "plus_z": [0, 0, 1], "position": [0, 0, 0]}"#;
        let head = r#"{"plus_x": [0, 1, 0], "plus_z": [0, 0, 1], "position": [0, 0, 0]}"#;
        let cam = r#"{"plus_x": [1, 0, 0], "plus_z": [0, 0, 1], "position": [10, 0, 0]}"#;
        let kb = r#"{"cameraModel": "kb", "fx": 1, "fy": 2, "cx": 3, "cy": 4, "k1": 5, "k2": 6, "k3": 7, "k4": 8}"#;
        let dev = format!(r#"{{"head": {head}, "cv": {{"cad_from_cal": {ident}}}}}"#);
        let xrs = format!(
            r#"{{"cameras": [{{"sourceCamera": "slam_left", "width": 1056, "height": 1024, "extrinsics": {cam}, "intrinsics": [{kb}]}}]}}"#
        );
        std::fs::write(dir.join("device_config.json"), dev).unwrap();
        std::fs::write(dir.join("xrservice.json"), xrs).unwrap();
        let c = load(dir.to_str().unwrap(), "slam_left");
        std::fs::remove_dir_all(&dir).unwrap();
        let c = c.unwrap();
        assert!(c.fx == 1.0 && c.cy == 4.0 && c.k == [5.0, 6.0, 7.0, 8.0]);
        assert!(c.t[0].abs() < 1e-7 && (c.t[1] + 0.01).abs() < 1e-7 && c.t[2].abs() < 1e-7);
        assert!(c.r[0].abs() < 1e-7 && (c.r[3] + 1.0).abs() < 1e-7 && (c.r[8] - 1.0).abs() < 1e-7);
    }

    #[test]
    fn rotations() {
        // 90 deg about z takes x to y; a turned camera's content moves the same way: the rectified pixel that saw the
        // camera's ray h now sees R^T h.
        let r = rotation([0.0, 0.0, std::f32::consts::FRAC_PI_2]);
        assert!((r[3] - 1.0).abs() < 1e-6 && r[0].abs() < 1e-6 && (r[8] - 1.0).abs() < 1e-6);
        let c = CamCal { r: rotation([0.1, 0.2, 0.3]), ..Default::default() };
        let back = rotated(&rotated(&c, [0.01, -0.02, 0.03]), [-0.01, 0.02, -0.03]);
        assert!(back.r.iter().zip(&c.r).all(|(a, b)| (a - b).abs() < 1e-6));
    }

    #[test]
    fn valid_region() {
        // An 8 x 8 view whose left 2 columns and bottom row are outside the image.
        let s = 8;
        let map = (0..s * s).map(|i| if i % s < 2 || i / s == s - 1 { [-1.0, 0.0] } else { [1.0, 1.0] }).collect();
        let v = Rectifier { s, stride: s, f: 1.0, map }.valid();
        assert!(
            v.rows[0] == (2.0, 8.0) && v.rows[7] == (8.0, 0.0) && v.cols[0] == (8.0, 0.0) && v.cols[4] == (0.0, 7.0)
        );
        assert!(
            v.inside(4.0, 3.0, 1.0)
                && !v.inside(2.5, 3.0, 1.0)
                && !v.inside(4.0, 6.5, 1.0)
                && !v.inside(-1.0, 3.0, 0.0)
        );
        assert!(v.left(0.0, 6.0) == 2.0 && v.left(0.0, 7.0) == 8.0 && v.bottom(2.0, 7.0) == 7.0);
    }
}
