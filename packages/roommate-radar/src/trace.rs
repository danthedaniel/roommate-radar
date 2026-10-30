//! One JSON line per detector frame, for fitting the noise model to real data and replaying the tracker offline:
//! developer builds' PATH, and the calibration's (calibrate.rs). Each line has the capture time (s), the views' size and
//! focal length (px), the head pose and controller positions then (standing universe), both views' boxes (px), the
//! objects made of them (with their disparities) and the measurements, and the tracks after the update; plus the
//! caller's fields.
//! /tmp is RAM on the Frame (tmpfs), so it's bounded: at MAX_BYTES the file moves to <path>.1 (replacing the one
//! before) and a new one starts, keeping the latest 2 x MAX_BYTES (~1 KB a frame: an hour or more at 8 Hz).
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};

use serde_json::{Map, Value, json};

use crate::detector::DetectionSet;
use crate::openvr::HmdMatrix34_t;
use crate::tracker::{Meas, Tracker};
use crate::v3::V3;

const PATH: &str = "/tmp/roommate-radar-trace.jsonl";
const MAX_BYTES: u64 = 32 << 20;

pub struct Trace {
    path: String,
    file: Option<BufWriter<File>>,
    bytes: u64,
}

pub fn xyz(p: V3) -> Value {
    json!([p.x, p.y, p.z])
}

impl Trace {
    /// Developer builds' trace, replacing the last run's; one that writes nothing in release builds.
    pub fn new() -> Trace {
        if cfg!(feature = "developer") {
            Trace::open(PATH, false)
        } else {
            Trace { path: String::new(), file: None, bytes: 0 }
        }
    }

    /// A trace to `path`: added to what's there (append), or replacing it.
    pub fn open(path: &str, append: bool) -> Trace {
        if !append {
            let _ = std::fs::remove_file(format!("{path}.1"));
        }
        let file = OpenOptions::new().create(true).append(append).write(!append).truncate(!append).open(path);
        let bytes = file.as_ref().ok().and_then(|f| f.metadata().ok()).map_or(0, |m| m.len());
        Trace { path: path.into(), file: file.ok().map(BufWriter::new), bytes }
    }

    /// A line of the caller's own (an event).
    pub fn line(&mut self, v: &Value) {
        let Some(file) = &mut self.file else {
            return;
        };
        let text = v.to_string();
        if writeln!(file, "{text}").and_then(|_| file.flush()).is_err() {
            self.file = None; // stop rather than fail every frame
            return;
        }
        self.bytes += text.len() as u64 + 1;
        if self.bytes > MAX_BYTES {
            let rotated = std::fs::rename(&self.path, format!("{}.1", self.path));
            self.file = rotated.and_then(|_| File::create(&self.path)).ok().map(BufWriter::new);
            self.bytes = 0;
        }
    }

    /// The detector frame d and what came of it, with `extra`'s fields added.
    pub fn frame(
        &mut self,
        d: &DetectionSet,
        head: &HmdMatrix34_t,
        ctrl: &[V3],
        ms: &[Meas],
        tracker: &Tracker,
        extra: Map<String, Value>,
    ) {
        if self.file.is_none() {
            return;
        }
        let mut line = json!({
            "t": d.t_capture_ns as f64 / 1e9,
            "s": d.view.as_ref().map(|v| v.s),
            "f": d.view.as_ref().map(|v| v.f),
            "correction": d.correction.map(f32::to_degrees),
            "head": head.m,
            "ctrl": ctrl.iter().map(|&p| xyz(p)).collect::<Vec<_>>(),
            "views": d.views.iter().map(|v| v.iter().map(|b| json!([b.x0, b.y0, b.x1, b.y1, b.score, b.cls])).collect::<Vec<_>>()).collect::<Vec<_>>(),
            "objs": d.objs.iter().map(|o| json!({
                "cls": o.cls, "score": o.score, "stereo": o.stereo, "dist": o.dist, "sigma_z": o.sigma_z,
                "clip": o.clip, "corners": o.corners.map(xyz), "origin": xyz(o.origin),
                "disp": {"edges": o.disp.edges, "sigma_edges": o.disp.sigma_edges, "patches": o.disp.patches,
                    "sigma_patches": o.disp.sigma_patches, "used": o.disp.used,
                    "patch_stats": {"tried": o.disp.patch_stats.tried, "outside": o.disp.patch_stats.outside,
                        "flat": o.disp.patch_stats.flat, "weak": o.disp.patch_stats.weak,
                        "unclear": o.disp.patch_stats.unclear, "matched": o.disp.patch_stats.matched,
                        "contrast": o.disp.patch_stats.contrast}},
            })).collect::<Vec<_>>(),
            "meas": ms.iter().map(|m| json!({
                "p": xyz(m.p), "w": m.w, "h": m.h, "ray": xyz(m.ray), "sigma_ray": m.sigma_ray, "sigma_lat": m.sigma_lat,
                "score": m.score, "cls": m.cls, "weak": m.weak, "clip": [m.clip_w, m.clip_h],
            })).collect::<Vec<_>>(),
            "tracks": tracker.tracks.iter().map(|t| json!({
                "id": t.id, "p": xyz(t.p), "v": xyz(t.v), "cov": t.cov(), "w": t.w, "h": t.h, "hits": t.hits,
                "cls": t.cls, "score": t.score, "last_seen": t.last_seen,
            })).collect::<Vec<_>>(),
        });
        line.as_object_mut().unwrap().extend(extra);
        self.line(&line);
    }
}
