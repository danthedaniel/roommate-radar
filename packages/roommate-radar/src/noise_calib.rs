//! `roommate-radar --calibrate`: records what the detector makes of a person standing still at chosen distances, for
//! fitting the noise model (docs/detection-tracking-prior-art.md, section 7). The person holds the left controller flat
//! on their chest: its tracked position is the truth to compare with.
//!
//! A "Calibration" dashboard tab picks the distance (and the detection resolution); Start counts GET_READY_S down, then
//! records for RECORD_S and stops by itself, every detector frame going to RECORDING_PATH (trace.rs) with the recording's number,
//! its distance and the left controller's pose. The tab shows the countdown as a progress bar, with what the detector
//! sees; then it moves on to the next distance. Finish quits. No images are kept: for checking the rectification, the
//! detector matches patches all over the views about once a second (stereo::alignment) and only those numbers are
//! recorded. "Far" is for a far scene (a hallway, out of a window), whose disparity should be about 0.
//!
//! Runs from its own devkit install (deploy.yml's calibrate play) without the app's setup (register.rs), so the app's
//! own install, driver and autostart are left alone. Of the app's saved settings it reads only the rectification
//! correction (`align`) to start from, and saves nothing.
use std::sync::atomic::Ordering;

use serde_json::{Map, Value, json};

use crate::detector::{self, QUIT};
use crate::draw::{self, Image, WHITE};
use crate::measure;
use crate::openvr::{self, TrackedDevicePose_t, VROverlayHandle_t, vr};
use crate::panel::{BG, BTN_FILL, GREY, LIGHT, TRACK_OFF, fill_circle, fill_rrect, icon};
use crate::settings::{SIZES, Settings};
use crate::shm::now_ns;
use crate::textures::Textures;
use crate::trace::{Trace, xyz};
use crate::tracker::Tracker;
use crate::v3::{V3, position};

pub const RECORDING_PATH: &str = "/tmp/roommate-radar-calibration.jsonl";
pub const LOG_PATH: &str = "/tmp/roommate-radar-calibration.log";
const DISTANCES: [f32; 8] = [1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, FAR];
const FAR: f32 = 100.0; // "Far": a far scene, no one in it needed
const GET_READY_S: f64 = 3.0;
const RECORD_S: f64 = 30.0;
const HZ: f32 = 8.0;
const DASH_KEY: &str = "steamframe.roommate-radar.calibration";

// The tab (texture px): title, status, instructions, the distance and resolution buttons, Start, Finish.
const W: f32 = 1000.0;
const H: f32 = 990.0;
const PAD: f32 = 48.0;
const GAP: f32 = 12.0;
const DIST_Y: f32 = 350.0;
const PILL_H: f32 = 80.0;
const SIZE_Y: f32 = 560.0;
const START_Y: f32 = 700.0;
const START_W: f32 = 560.0;
const START_H: f32 = 110.0;
const FINISH_Y: f32 = 850.0;
const FINISH_W: f32 = 360.0;
const FINISH_H: f32 = 90.0;
const BAR_H: f32 = 30.0;
const RED: draw::Rgb = [205, 70, 70];
const HOVER_TICK_US: u16 = 1500;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Button {
    Distance(usize),
    Size(usize),
    Start, // or Cancel, while armed or recording
    Finish,
}

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Idle,
    Ready(u64),     // counting GET_READY_S down from then (ns)
    Recording(u64), // recording RECORD_S from then (ns)
}

/// The buttons and where they are: x0, y0, w, h.
fn buttons() -> Vec<(Button, [f32; 4])> {
    let n = DISTANCES.len() as f32;
    let dw = (W - 2.0 * PAD - (n - 1.0) * GAP) / n;
    let mut b: Vec<_> = (0..DISTANCES.len())
        .map(|i| (Button::Distance(i), [PAD + i as f32 * (dw + GAP), DIST_Y, dw, PILL_H]))
        .collect();
    let sw = 220.0;
    b.extend((0..SIZES.len()).map(|i| (Button::Size(i), [PAD + i as f32 * (sw + GAP), SIZE_Y, sw, PILL_H])));
    b.push((Button::Start, [(W - START_W) / 2.0, START_Y, START_W, START_H]));
    b.push((Button::Finish, [(W - FINISH_W) / 2.0, FINISH_Y, FINISH_W, FINISH_H]));
    b
}

fn hit(x: f32, y: f32) -> Option<Button> {
    buttons().into_iter().find(|(_, [x0, y0, w, h])| x >= *x0 && x <= x0 + w && y >= *y0 && y <= y0 + h).map(|b| b.0)
}

fn distance_label(i: usize) -> String {
    if DISTANCES[i] == FAR { "Far".into() } else { format!("{} m", DISTANCES[i]) }
}

/// Text centered on x, baseline at y.
fn centered(img: &mut Image, x: f32, y: f32, s: &str, rgb: draw::Rgb, size: f32) {
    img.text((x - draw::text_end_x(0, s, size) as f32 / 2.0) as i32, y as i32, s, rgb, size);
}

struct Calibration {
    dash: VROverlayHandle_t,
    distance_idx: usize,
    size_idx: usize,
    phase: Phase,
    recordings_done: [u32; DISTANCES.len()], // at each distance
    segment: u32,                            // recordings started
    frames: u32,                             // detector frames recorded, this recording
    with_person: u32,                        // of which with a person measured
    hover: Option<Button>,
    drawn: String, // what the tab was last drawn for ("" = redraw)
    live: String,  // what the detector sees
}

impl Calibration {
    fn create(textures: &mut Textures, size_idx: usize) -> Result<Calibration, String> {
        let (dash, icon_ov) = vr()
            .create_dashboard_overlay(DASH_KEY, "Calibration")
            .map_err(|e| format!("CreateDashboardOverlay: {e}"))?;
        textures.set(icon_ov, &icon());
        vr().set_width(dash, 1.5);
        vr().set_mouse_input(dash, W, H);
        vr().show_dashboard(DASH_KEY);
        Ok(Calibration {
            dash,
            distance_idx: 0,
            size_idx,
            phase: Phase::Idle,
            recordings_done: [0; DISTANCES.len()],
            segment: 0,
            frames: 0,
            with_person: 0,
            hover: None,
            drawn: String::new(),
            live: String::new(),
        })
    }

    /// The pointer's events since the last call; Some(Finish) if it was pressed, or Start (to start or cancel).
    fn poll(&mut self) -> Option<Button> {
        let mut pressed = None;
        while let Some(ev) = vr().poll_overlay_event(self.dash) {
            let mouse = unsafe { ev.data.mouse };
            let b = hit(mouse.x, H - mouse.y); // mouse y is up from the bottom
            if ev.eventType == openvr::EVREventType_VREvent_MouseMove && b != self.hover {
                self.hover = b;
                self.drawn.clear();
                if b.is_some() {
                    vr().haptic_pulse(ev.trackedDeviceIndex, HOVER_TICK_US);
                }
            }
            if ev.eventType == openvr::EVREventType_VREvent_MouseButtonDown {
                self.drawn.clear();
                match b {
                    // Not while recording: the recording is for the distance and size it started with
                    Some(Button::Distance(i)) if self.phase == Phase::Idle => self.distance_idx = i,
                    Some(Button::Size(i)) if self.phase == Phase::Idle => {
                        self.size_idx = i;
                        detector::SIZE.store(SIZES[i], Ordering::Relaxed); // the detector reloads
                    }
                    Some(b @ (Button::Start | Button::Finish)) => pressed = Some(b),
                    _ => {}
                }
            }
        }
        pressed
    }

    /// The countdown, while getting ready or recording: whether getting ready, and the seconds left.
    fn countdown(&self, now: u64) -> Option<(bool, f64)> {
        let (ready, t0, total) = match self.phase {
            Phase::Ready(t) => (true, t, GET_READY_S),
            Phase::Recording(t) => (false, t, RECORD_S),
            Phase::Idle => return None,
        };
        Some((ready, (total - now.saturating_sub(t0) as f64 / 1e9).max(0.0)))
    }

    /// Getting ready -> recording (once the model is loaded) -> done, as time passes: what happened ("start", or "end"
    /// and the distance it was at, before moving on to the next one).
    fn advance(&mut self, now: u64, loading: bool) -> Option<(&'static str, usize)> {
        match self.phase {
            Phase::Ready(t) if now - t > (GET_READY_S * 1e9) as u64 && !loading => {
                self.segment += 1;
                (self.frames, self.with_person) = (0, 0);
                self.phase = Phase::Recording(now);
                Some(("start", self.distance_idx))
            }
            Phase::Recording(t) if now - t > (RECORD_S * 1e9) as u64 => {
                let finished_idx = self.distance_idx;
                self.recordings_done[finished_idx] += 1;
                self.distance_idx = (finished_idx + 1) % DISTANCES.len();
                self.phase = Phase::Idle;
                Some(("end", finished_idx))
            }
            _ => None,
        }
    }

    fn update(&mut self, textures: &mut Textures, status: &str, now: u64) {
        let countdown = self.countdown(now);
        // the bar moves in 0.2 s steps
        let key = format!(
            "{status}{:?}{:?}{}{}{}",
            countdown.map(|(r, secs_left)| (r, (secs_left * 5.0) as u32)),
            self.hover,
            self.distance_idx,
            self.size_idx,
            self.live
        );
        if vr().is_visible(self.dash) && self.drawn != key {
            self.drawn = key;
            textures.set(self.dash, &self.render(status, countdown));
        }
    }

    /// The tab; while counting down (getting ready?, seconds left), the countdown in place of the instructions.
    fn render(&self, status: &str, countdown: Option<(bool, f64)>) -> Image {
        let mut img = Image::new(W as usize, H as usize);
        fill_rrect(&mut img, 0.0, 0.0, W, H, 32.0, BG, 0.94);
        img.text(PAD as i32, 100, "Roommate Radar Calibration", WHITE, 52.0);
        img.text(PAD as i32, 150, status, GREY, 30.0);
        let lines = [
            "The person measured holds the LEFT controller flat on their",
            "chest, faces you and stands still. Pick their distance, press",
            "Start: after 3 s it records for 30 s. Keep them in view.",
        ];
        match countdown {
            None => {
                for (i, l) in lines.iter().enumerate() {
                    img.text(PAD as i32, 210 + i as i32 * 36, l, LIGHT, 28.0);
                }
            }
            Some((ready, secs_left)) => {
                let (text, total, color) = if ready {
                    (format!("Get ready… {}", secs_left.ceil()), GET_READY_S, GREY)
                } else {
                    (
                        format!("Recording {} · {} s left", distance_label(self.distance_idx), secs_left.ceil()),
                        RECORD_S,
                        draw::class_rgb(0),
                    )
                };
                img.text(PAD as i32, 218, &text, WHITE, 44.0);
                // the bar empties as time runs out
                let (bw, frac) = (W - 2.0 * PAD, (secs_left / total) as f32);
                fill_rrect(&mut img, PAD, 238.0, bw, BAR_H, BAR_H / 2.0, TRACK_OFF, 1.0);
                fill_rrect(&mut img, PAD, 238.0, (bw * frac).max(BAR_H), BAR_H, BAR_H / 2.0, color, 1.0);
                img.text(PAD as i32, 300, &self.live, LIGHT, 28.0);
            }
        }
        img.text(PAD as i32, (DIST_Y - 16.0) as i32, "Distance", GREY, 26.0);
        img.text(PAD as i32, (SIZE_Y - 16.0) as i32, "Detection Resolution", GREY, 26.0);
        let idle = self.phase == Phase::Idle;
        for (b, [x0, y0, w, h]) in buttons() {
            let hovered = self.hover == Some(b);
            let (btn_text, on, fill, size) = match b {
                Button::Distance(i) => (distance_label(i), i == self.distance_idx, BTN_FILL, 34.0),
                Button::Size(i) => (format!("{} px", SIZES[i]), i == self.size_idx, BTN_FILL, 34.0),
                Button::Start if idle => ("Start".into(), true, BTN_FILL, 48.0),
                Button::Start => ("Cancel".into(), false, RED, 48.0),
                Button::Finish => ("Finish".into(), false, BTN_FILL, 40.0),
            };
            let fill = if on { draw::class_rgb(0) } else { fill };
            let dimmed = !idle && matches!(b, Button::Distance(_) | Button::Size(_)) && !on;
            let alpha = if dimmed {
                0.4
            } else if hovered {
                1.0
            } else {
                0.85
            };
            fill_rrect(&mut img, x0, y0, w, h, h / 2.0, fill, alpha);
            centered(
                &mut img,
                x0 + w / 2.0,
                y0 + h / 2.0 + size * 0.36,
                &btn_text,
                if dimmed { GREY } else { WHITE },
                size,
            );
            if let Button::Distance(i) = b {
                // recordings done at this distance: a dot each
                for k in 0..self.recordings_done[i].min(5) {
                    let cx = x0 + w / 2.0 + (k as f32 - (self.recordings_done[i].min(5) as f32 - 1.0) / 2.0) * 18.0;
                    fill_circle(&mut img, cx, y0 + h + 22.0, 6.0, draw::class_rgb(16), 1.0);
                }
            }
        }
        img
    }
}

/// Horizontal distance (m) from the head at head_pos to p.
fn across_floor(head_pos: V3, p: V3) -> f32 {
    (p.x - head_pos.x).hypot(p.z - head_pos.z)
}

pub fn run() -> i32 {
    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Overlay) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    // The app's defaults, without what would drop the person or their controller: nothing saved.
    let settings = Settings { ignore_hands: false, max_dist: 10.0, ..Settings::default() };
    // Starting from the app's saved rectification correction (read only: nothing here is saved), kept up like the app's
    let saved = Settings::load_saved();
    let mut selfcal = crate::selfcal::SelfCal::new(saved.correction);
    let size_idx = SIZES.iter().position(|&s| s == settings.size).unwrap_or(0);
    detector::SIZE.store(settings.size, Ordering::Relaxed);
    detector::HZ.store(HZ);
    let mut textures = Textures::new();
    let mut cal = match Calibration::create(&mut textures, size_idx) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    let detector_thread = crate::spawn_detector();
    let mut trace = Trace::open(RECORDING_PATH, true);
    trace.write_line(&json!({"event": "session", "t": now_ns() as f64 / 1e9}));
    println!("calibration: recording to {RECORDING_PATH}");
    let mut tracker = Tracker::default();
    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    let event = |trace: &mut Trace, cal: &Calibration, what: &str, distance_idx: usize, t: u64| {
        let distance_m = DISTANCES[distance_idx];
        let size_px = SIZES[cal.size_idx];
        trace.write_line(
            &json!({"event": what, "t": t as f64 / 1e9, "segment": cal.segment, "distance": distance_m, "size": size_px,
            "frames": cal.frames, "seen": cal.with_person}),
        );
        println!(
            "calibration: {what} segment {} at {distance_m} m, {size_px} px: {} frames, {} with a person",
            cal.segment, cal.frames, cal.with_person
        );
    };
    while !QUIT.load(Ordering::Relaxed) {
        if !vr().wait_frame_sync(100) {
            std::thread::sleep(std::time::Duration::from_millis(11));
        }
        while let Some(ev) = vr().poll_event() {
            if ev.eventType == openvr::EVREventType_VREvent_Quit {
                vr().acknowledge_quit();
                QUIT.store(true, Ordering::Relaxed);
            }
        }
        let now = now_ns();
        match cal.poll() {
            Some(Button::Start) if cal.phase == Phase::Idle => cal.phase = Phase::Ready(now),
            Some(b) => {
                if matches!(cal.phase, Phase::Recording(_)) {
                    event(&mut trace, &cal, "cancel", cal.distance_idx, now);
                }
                if b == Button::Finish {
                    break;
                }
                cal.phase = Phase::Idle;
            }
            _ => {}
        }
        let loading = detector::LOADING_SIZE.load(Ordering::Relaxed) != 0;
        if let Some((what, distance_idx)) = cal.advance(now, loading) {
            event(&mut trace, &cal, what, distance_idx, now);
        }

        let dets = {
            let dets = detector::DETECTIONS.lock().unwrap();
            (dets.id != seen_det_id).then(|| dets.clone())
        };
        // Lost tracking: no head pose to measure with (main.rs's head_pose too)
        let dets = dets.filter(|d| crate::head_pose((d.t_capture_ns as i64 - now as i64) as f32 / 1e9).1);
        if let Some(dets) = dets {
            seen_det_id = dets.id;
            last_det_ns = now;
            let from_now_s = (dets.t_capture_ns as i64 - now as i64) as f32 / 1e9;
            let mut poses: [TrackedDevicePose_t; openvr::MAX_TRACKED_DEVICES] = unsafe { std::mem::zeroed() };
            vr().poses(from_now_s, &mut poses);
            let head = poses[0].mDeviceToAbsoluteTracking;
            let left_pose = vr().left_controller().map(|i| &poses[i as usize]);
            let ctrl = crate::controllers(from_now_s);
            let meas = measure::measurements(&dets.objs, &head, &[], &[], &settings);
            let view = dets.view.as_deref();
            tracker.update(&meas, dets.t_capture_ns as f64 / 1e9, |p| {
                view.is_none_or(|v| measure::observable(v, &head, p, &[], &settings))
            });
            // What it sees: the person nearest the left controller, and the controller, across the floor
            let head_pos = position(&head);
            let truth = left_pose.filter(|p| p.bPoseIsValid).map(|p| position(&p.mDeviceToAbsoluteTracking));
            let person = meas
                .iter()
                .filter(|m| m.cls == 0)
                .min_by(|a, b| {
                    let near = |m: &&crate::tracker::Meas| truth.map_or(0.0, |t| (m.p - t).len());
                    near(a).total_cmp(&near(b))
                })
                .map(|m| across_floor(head_pos, m.p));
            cal.live = match (person, truth) {
                (Some(p), Some(t)) => format!("seen {p:.2} m · controller {:.2} m", across_floor(head_pos, t)),
                (None, Some(t)) => format!("no person · controller {:.2} m", across_floor(head_pos, t)),
                (Some(p), None) => format!("seen {p:.2} m · no controller"),
                (None, None) => "no person · no controller".into(),
            };
            if let Phase::Recording(_) = cal.phase {
                cal.frames += 1;
                cal.with_person += person.is_some() as u32;
                let mut extra = Map::new();
                extra.insert(
                    "calib".into(),
                    json!({"segment": cal.segment, "distance": DISTANCES[cal.distance_idx], "size": SIZES[cal.size_idx]}),
                );
                let left_json = left_pose.map_or(Value::Null, |p| {
                    json!({"m": p.mDeviceToAbsoluteTracking.m, "valid": p.bPoseIsValid,
                        "p": xyz(position(&p.mDeviceToAbsoluteTracking))})
                });
                extra.insert("left".into(), left_json);
                if let Some(matches) = &dets.align_matches {
                    // rounded: 0.01 px is plenty
                    let r = |v: f32| (v * 100.0).round() / 100.0;
                    extra.insert("align".into(), json!(matches.iter().map(|p| p.map(r)).collect::<Vec<_>>()));
                }
                trace.frame(&dets, &head, &ctrl, &meas, &tracker, extra);
            }
            selfcal.step(&dets, &head);
        }

        let recordings_total: u32 = cal.recordings_done.iter().sum();
        let status = if detector::WAITING.load(Ordering::Relaxed) || (!loading && now - last_det_ns > 2_000_000_000) {
            let p = detector::PROBLEM.lock().unwrap();
            if p.is_empty() { "Waiting for camera frames".into() } else { p.clone() }
        } else if loading {
            format!("Loading {}px model…", detector::LOADING_SIZE.load(Ordering::Relaxed))
        } else {
            match cal.phase {
                Phase::Idle => format!("{recordings_total} recorded · {}", cal.live),
                Phase::Ready(_) => format!("Get ready · {}", cal.live),
                Phase::Recording(_) => format!("Recording {}… · {}", distance_label(cal.distance_idx), cal.live),
            }
        };
        cal.update(&mut textures, &status, now);
    }
    trace.write_line(&json!({"event": "finish", "t": now_ns() as f64 / 1e9}));
    QUIT.store(true, Ordering::Relaxed);
    if let Some(detector_thread) = detector_thread {
        let _ = detector_thread.join();
    }
    openvr::shutdown();
    crate::ncnn::destroy_gpu_instance();
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout() {
        // Every button inside the tab, none overlapping, each found where it's drawn.
        let b = buttons();
        for (i, (k, [x0, y0, w, h])) in b.iter().enumerate() {
            assert!(*x0 >= 0.0 && *y0 >= 0.0 && x0 + w <= W && y0 + h <= H, "{k:?} outside");
            assert_eq!(hit(x0 + w / 2.0, y0 + h / 2.0), Some(*k));
            for (k2, [x1, y1, w1, h1]) in &b[i + 1..] {
                assert!(x0 + w <= *x1 || x1 + w1 <= *x0 || y0 + h <= *y1 || y1 + h1 <= *y0, "{k:?} overlaps {k2:?}");
            }
        }
        assert_eq!(hit(5.0, 5.0), None);
        // the done-dots under the distance buttons fit above the resolution label
        const { assert!(DIST_Y + PILL_H + 22.0 + 6.0 < SIZE_Y - 16.0 - 26.0) };
    }

    fn calibration() -> Calibration {
        Calibration {
            dash: 0,
            distance_idx: 2,
            size_idx: 1,
            phase: Phase::Idle,
            recordings_done: [1, 2, 0, 0, 0, 0, 0, 0],
            segment: 0,
            frames: 0,
            with_person: 0,
            hover: Some(Button::Start),
            drawn: String::new(),
            live: "seen 2.48 m · controller 2.51 m".into(),
        }
    }

    #[test]
    fn records_30_s_then_moves_on() {
        let s = 1_000_000_000u64;
        let mut c = Calibration { phase: Phase::Ready(10 * s), ..calibration() };
        assert_eq!(c.countdown(11 * s), Some((true, 2.0)));
        assert_eq!(c.advance(12 * s, false), None, "getting ready for 3 s");
        assert_eq!(c.advance(14 * s, true), None, "and until the model is loaded");
        assert_eq!(c.advance(14 * s, false), Some(("start", 2)));
        assert!(c.phase == Phase::Recording(14 * s) && c.segment == 1);
        assert_eq!(c.countdown(24 * s), Some((false, 20.0)));
        assert_eq!(c.advance(44 * s, false), None, "30 s");
        assert_eq!(c.advance(44 * s + 1, false), Some(("end", 2)), "then stops by itself");
        assert!(c.phase == Phase::Idle && c.countdown(45 * s).is_none());
        assert!(c.distance_idx == 3 && c.recordings_done[2] == 1, "on to the next distance");
    }

    #[test]
    fn renders() {
        let c = calibration();
        let mut img = c.render("Watching", None);
        assert!(img.at(0, 0)[3] == 0 && img.at(500, 5)[3] > 200);
        let mut img = c.render("Recording", Some((false, 12.0)));
        // 12 of 30 s left: the bar's filled 40% of the way across
        let (filled, empty) = (img.at(100, 253).to_vec(), img.at(800, 253).to_vec());
        assert!(filled != empty && img.at(380, 253).to_vec() == filled && img.at(450, 253).to_vec() == empty);
    }
}
