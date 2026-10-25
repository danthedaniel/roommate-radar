//! SLAM camera calibration from the factory files (/persist, which the driver copies to shm::CALIB_DIR):
//! xrservice.json (per camera: Kannala-Brandt intrinsics, extrinsics) and device_config.json (`head`, and XRService's
//! `cv/cad_from_cal`), chained into head_from_cam: head <- CAD (`head`) <- cal/cam0 frame (`cad_from_cal`) <- cam
//! (extrinsics, mm). Every frame there is plus_x / plus_z axes (+ position) in its parent, with plus_y = plus_z x
//! plus_x. Camera frames are OpenCV-style (x right, y down, z forward). And the rectification it gives.
use serde_json::Value;

use crate::shm;

#[derive(Clone, Copy, Debug, Default)]
pub struct CamCal {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub k: [f32; 4],
    pub r: [f32; 9], // head_from_cam rotation, row-major; camera frame is OpenCV (x right, y down, z fwd)
    pub t: [f32; 3], // camera position in head frame (m)
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

/// Camera `name` (shm::CAM_NAMES) from the calibration files in `dir`.
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
    let k = |n: &str| kb[n].as_f64().map(|v| v as f32).ok_or_else(|| bad(&format!("{n} for {name}")));
    Ok(CamCal {
        fx: k("fx")?,
        fy: k("fy")?,
        cx: k("cx")?,
        cy: k("cy")?,
        k: [k("k1")?, k("k2")?, k("k3")?, k("k4")?],
        r: std::array::from_fn(|i| r[i / 3][i % 3] as f32),
        t: t.map(|v| v as f32),
    })
}

// ---------------------------------------------------------------------------------------------
// Rectification: virtual pinhole camera at the SLAM camera's position, axes aligned with the head
// (x right, y down, looking along head -z). Both views share orientation -> rectified stereo pair.

pub struct Rectifier {
    s: usize,
    pub f: f32,
    map: Vec<[f32; 2]>, // (src x, src y) per output pixel; x < 0 = outside the fisheye
}

impl Rectifier {
    pub fn new(c: &CamCal, size: usize, hfov_deg: f32) -> Rectifier {
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
                if m[0] >= 0.0 && m[1] >= 0.0 && m[0] <= (shm::W - 2) as f32 && m[1] <= (shm::H - 2) as f32 {
                    map[v * size + u] = m;
                }
            }
        }
        Rectifier { s: size, f, map }
    }

    /// The S x S rectified view of a camera frame (Y plane, shm::STRIDE).
    pub fn apply(&self, src: &[u8], dst: &mut [u8]) {
        for (d, &[sx, sy]) in dst[..self.s * self.s].iter_mut().zip(&self.map) {
            if sx < 0.0 {
                *d = 0;
                continue;
            }
            let (x, y) = (sx as usize, sy as usize);
            let (ax, ay) = (sx - x as f32, sy - y as f32);
            let p = &src[y * shm::STRIDE + x..];
            let at = |i: usize| p[i] as f32;
            let top = at(0) + ax * (at(1) - at(0));
            let bot = at(shm::STRIDE) + ax * (at(shm::STRIDE + 1) - at(shm::STRIDE));
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
        let dir = std::env::temp_dir().join(format!("guardian-calib-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let ident = r#"{"plus_x": [1, 0, 0], "plus_z": [0, 0, 1], "position": [0, 0, 0]}"#;
        let head = r#"{"plus_x": [0, 1, 0], "plus_z": [0, 0, 1], "position": [0, 0, 0]}"#;
        let cam = r#"{"plus_x": [1, 0, 0], "plus_z": [0, 0, 1], "position": [10, 0, 0]}"#;
        let kb = r#"{"cameraModel": "kb", "fx": 1, "fy": 2, "cx": 3, "cy": 4, "k1": 5, "k2": 6, "k3": 7, "k4": 8}"#;
        let dev = format!(r#"{{"head": {head}, "cv": {{"cad_from_cal": {ident}}}}}"#);
        let xrs =
            format!(r#"{{"cameras": [{{"sourceCamera": "slam_left", "extrinsics": {cam}, "intrinsics": [{kb}]}}]}}"#);
        std::fs::write(dir.join("device_config.json"), dev).unwrap();
        std::fs::write(dir.join("xrservice.json"), xrs).unwrap();
        let c = load(dir.to_str().unwrap(), "slam_left");
        std::fs::remove_dir_all(&dir).unwrap();
        let c = c.unwrap();
        assert!(c.fx == 1.0 && c.cy == 4.0 && c.k == [5.0, 6.0, 7.0, 8.0]);
        assert!(c.t[0].abs() < 1e-7 && (c.t[1] + 0.01).abs() < 1e-7 && c.t[2].abs() < 1e-7);
        assert!(c.r[0].abs() < 1e-7 && (c.r[3] + 1.0).abs() < 1e-7 && (c.r[8] - 1.0).abs() < 1e-7);
    }
}
