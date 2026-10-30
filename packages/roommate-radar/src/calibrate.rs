//! `roommate-radar --calibrate`: records what the detector makes of a person standing still at chosen distances, for
//! fitting the noise model (docs/detection-tracking-prior-art.md, section 7). The person holds the left controller flat
//! on their chest: its tracked position is the truth to compare with.
//!
//! A "Calibration" dashboard tab picks the distance (and the detection resolution); Start counts GET_READY_S down, then
//! records for RECORD_S and stops by itself, every detector frame going to PATH (trace.rs) with the recording's number,
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
use crate::measure::{self, position};
use crate::openvr::{self, TrackedDevicePose_t, VROverlayHandle_t, vr};
use crate::panel::{BG, BTN, GREY, LIGHT, TRACK_OFF, fill_circle, fill_rrect, icon};
use crate::settings::{SIZES, Settings};
use crate::shm::now_ns;
use crate::textures::Textures;
use crate::trace::{Trace, xyz};
use crate::tracker::Tracker;
use crate::v3::V3;

pub const PATH: &str = "/tmp/roommate-radar-calibration.jsonl";
pub const LOG: &str = "/tmp/roommate-radar-calibration.log";
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

#[derive(Clone, Copy, PartialEq, Debug)]
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

fn label(i: usize) -> String {
    if DISTANCES[i] == FAR { "Far".into() } else { format!("{} m", DISTANCES[i]) }
}

/// Text centered on x, baseline at y.
fn centered(img: &mut Image, x: f32, y: f32, s: &str, rgb: draw::Rgb, size: f32) {
    img.text((x - draw::text_width(0, s, size) as f32 / 2.0) as i32, y as i32, s, rgb, size);
}

struct Calibration {
    dash: VROverlayHandle_t,
    distance: usize,
    size: usize,
    phase: Phase,
    done: [u32; DISTANCES.len()], // recordings finished at each distance
    segment: u32,                 // recordings started
    frames: u32,                  // detector frames recorded, this recording
    seen: u32,                    // of which with a person measured
    hover: Option<Button>,
    drawn: String, // what the tab was last drawn for ("" = redraw)
    live: String,  // what the detector sees
}

impl Calibration {
    fn create(textures: &mut Textures, size: usize) -> Result<Calibration, String> {
        let (dash, icon_ov) = vr()
            .create_dashboard_overlay(DASH_KEY, "Calibration")
            .map_err(|e| format!("CreateDashboardOverlay: {e}"))?;
        textures.set(icon_ov, &icon());
        vr().set_width(dash, 1.5);
        vr().set_mouse_input(dash, W, H);
        vr().show_dashboard(DASH_KEY);
        Ok(Calibration {
            dash,
            distance: 0,
            size,
            phase: Phase::Idle,
            done: [0; DISTANCES.len()],
            segment: 0,
            frames: 0,
            seen: 0,
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
                    Some(Button::Distance(i)) if self.phase == Phase::Idle => self.distance = i,
                    Some(Button::Size(i)) if self.phase == Phase::Idle => {
                        self.size = i;
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
                (self.frames, self.seen) = (0, 0);
                self.phase = Phase::Recording(now);
                Some(("start", self.distance))
            }
            Phase::Recording(t) if now - t > (RECORD_S * 1e9) as u64 => {
                let d = self.distance;
                self.done[d] += 1;
                self.distance = (d + 1) % DISTANCES.len();
                self.phase = Phase::Idle;
                Some(("end", d))
            }
            _ => None,
        }
    }

    fn update(&mut self, textures: &mut Textures, status: &str, now: u64) {
        let countdown = self.countdown(now);
        // the bar moves in 0.2 s steps
        let key = format!(
            "{status}{:?}{:?}{}{}{}",
            countdown.map(|(r, left)| (r, (left * 5.0) as u32)),
            self.hover,
            self.distance,
            self.size,
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
            Some((ready, left)) => {
                let (text, total, color) = if ready {
                    (format!("Get ready… {}", left.ceil()), GET_READY_S, GREY)
                } else {
                    (
                        format!("Recording {} · {} s left", label(self.distance), left.ceil()),
                        RECORD_S,
                        draw::class_rgb(0),
                    )
                };
                img.text(PAD as i32, 218, &text, WHITE, 44.0);
                // the bar empties as time runs out
                let (bw, frac) = (W - 2.0 * PAD, (left / total) as f32);
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
            let (label, on, fill, size) = match b {
                Button::Distance(i) => (label(i), i == self.distance, BTN, 34.0),
                Button::Size(i) => (format!("{} px", SIZES[i]), i == self.size, BTN, 34.0),
                Button::Start if idle => ("Start".into(), true, BTN, 48.0),
                Button::Start => ("Cancel".into(), false, RED, 48.0),
                Button::Finish => ("Finish".into(), false, BTN, 40.0),
            };
            let fill = if on { draw::class_rgb(0) } else { fill };
            let dimmed = !idle && matches!(b, Button::Distance(_) | Button::Size(_)) && !on;
            let a = if dimmed {
                0.4
            } else if hovered {
                1.0
            } else {
                0.85
            };
            fill_rrect(&mut img, x0, y0, w, h, h / 2.0, fill, a);
            centered(
                &mut img,
                x0 + w / 2.0,
                y0 + h / 2.0 + size * 0.36,
                &label,
                if dimmed { GREY } else { WHITE },
                size,
            );
            if let Button::Distance(i) = b {
                // recordings done at this distance: a dot each
                for k in 0..self.done[i].min(5) {
                    let cx = x0 + w / 2.0 + (k as f32 - (self.done[i].min(5) as f32 - 1.0) / 2.0) * 18.0;
                    fill_circle(&mut img, cx, y0 + h + 22.0, 6.0, draw::class_rgb(16), 1.0);
                }
            }
        }
        img
    }
}

/// Horizontal distance (m) from the head at h to p.
fn across_floor(h: V3, p: V3) -> f32 {
    (p.x - h.x).hypot(p.z - h.z)
}

pub fn run() -> i32 {
    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Overlay) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    // The app's defaults, without what would drop the person or their controller: nothing saved.
    let set = Settings { hands: false, play_area: false, max_dist: 10.0, hz: HZ, ..Settings::default() };
    // Starting from the app's saved rectification correction (read only: nothing here is saved), kept up like the app's
    let home = std::env::var("HOME").unwrap_or_default();
    let saved = Settings::load(format!("{home}/{}", crate::shm::CONF).into());
    let mut selfcal = crate::selfcal::SelfCal::new(saved.align);
    let size = SIZES.iter().position(|&s| s == set.size).unwrap_or(0);
    detector::SIZE.store(set.size, Ordering::Relaxed);
    detector::HZ.store(HZ);
    let mut textures = Textures::new();
    let mut cal = match Calibration::create(&mut textures, size) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    let det = crate::spawn_detector();
    let mut trace = Trace::open(PATH, true);
    trace.line(&json!({"event": "session", "t": now_ns() as f64 / 1e9}));
    println!("calibration: recording to {PATH}");
    let mut tracker = Tracker::default();
    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    let event = |trace: &mut Trace, cal: &Calibration, what: &str, distance: usize, t: u64| {
        let d = DISTANCES[distance];
        let size = SIZES[cal.size];
        trace.line(&json!({"event": what, "t": t as f64 / 1e9, "segment": cal.segment, "distance": d, "size": size,
            "frames": cal.frames, "seen": cal.seen}));
        println!(
            "calibration: {what} segment {} at {d} m, {size} px: {} frames, {} with a person",
            cal.segment, cal.frames, cal.seen
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
            Some(Button::Finish) => {
                if matches!(cal.phase, Phase::Recording(_)) {
                    event(&mut trace, &cal, "cancel", cal.distance, now);
                }
                break;
            }
            Some(Button::Start) if cal.phase == Phase::Idle => cal.phase = Phase::Ready(now),
            Some(Button::Start) => {
                if matches!(cal.phase, Phase::Recording(_)) {
                    event(&mut trace, &cal, "cancel", cal.distance, now);
                }
                cal.phase = Phase::Idle;
            }
            _ => {}
        }
        let loading = detector::LOADING.load(Ordering::Relaxed) != 0;
        if let Some((what, distance)) = cal.advance(now, loading) {
            event(&mut trace, &cal, what, distance, now);
        }

        let d = {
            let d = detector::DETS.lock().unwrap();
            (d.id != seen_det_id).then(|| d.clone())
        };
        // Lost tracking: no head pose to measure with (main.rs's head_pose too)
        let d = d.filter(|d| crate::head_pose((d.t_capture_ns as i64 - now as i64) as f32 / 1e9).1);
        if let Some(d) = d {
            seen_det_id = d.id;
            last_det_ns = now;
            let ago = (d.t_capture_ns as i64 - now as i64) as f32 / 1e9;
            let mut poses: [TrackedDevicePose_t; openvr::MAX_TRACKED_DEVICES] = unsafe { std::mem::zeroed() };
            vr().poses(ago, &mut poses);
            let head = poses[0].mDeviceToAbsoluteTracking;
            let left = vr().left_hand().map(|i| &poses[i as usize]);
            let ctrl = crate::controllers(ago);
            let ms = measure::measurements(&d.objs, &head, 0.0, &[], &[], &set);
            let view = d.view.as_deref();
            tracker.update(&ms, d.t_capture_ns as f64 / 1e9, |p| {
                view.is_none_or(|v| measure::observable(v, &head, p, &[], &set))
            });
            // What it sees: the person nearest the left controller, and the controller, across the floor
            let h = position(&head);
            let truth = left.filter(|p| p.bPoseIsValid).map(|p| position(&p.mDeviceToAbsoluteTracking));
            let person = ms
                .iter()
                .filter(|m| m.cls == 0)
                .min_by(|a, b| {
                    let near = |m: &&crate::tracker::Meas| truth.map_or(0.0, |t| (m.p - t).len());
                    near(a).total_cmp(&near(b))
                })
                .map(|m| across_floor(h, m.p));
            cal.live = match (person, truth) {
                (Some(p), Some(t)) => format!("seen {p:.2} m · controller {:.2} m", across_floor(h, t)),
                (None, Some(t)) => format!("no person · controller {:.2} m", across_floor(h, t)),
                (Some(p), None) => format!("seen {p:.2} m · no controller"),
                (None, None) => "no person · no controller".into(),
            };
            if let Phase::Recording(_) = cal.phase {
                cal.frames += 1;
                cal.seen += person.is_some() as u32;
                let mut extra = Map::new();
                extra.insert(
                    "calib".into(),
                    json!({"segment": cal.segment, "distance": DISTANCES[cal.distance], "size": SIZES[cal.size]}),
                );
                let left = left.map_or(Value::Null, |p| {
                    json!({"m": p.mDeviceToAbsoluteTracking.m, "valid": p.bPoseIsValid,
                        "p": xyz(position(&p.mDeviceToAbsoluteTracking))})
                });
                extra.insert("left".into(), left);
                if let Some(a) = &d.align {
                    // rounded: 0.01 px is plenty
                    let r = |v: f32| (v * 100.0).round() / 100.0;
                    extra.insert("align".into(), json!(a.iter().map(|p| p.map(r)).collect::<Vec<_>>()));
                }
                trace.frame(&d, &head, &ctrl, &ms, &tracker, extra);
            }
            selfcal.step(&d, &head);
        }

        let finished: u32 = cal.done.iter().sum();
        let status = if detector::WAITING.load(Ordering::Relaxed) {
            let p = detector::PROBLEM.lock().unwrap();
            if p.is_empty() { "Waiting for camera frames".into() } else { p.clone() }
        } else if loading {
            format!("Loading {}px model…", detector::LOADING.load(Ordering::Relaxed))
        } else if now - last_det_ns > 2_000_000_000 {
            let p = detector::PROBLEM.lock().unwrap();
            if p.is_empty() { "Waiting for camera frames".into() } else { p.clone() }
        } else {
            match cal.phase {
                Phase::Idle => format!("{finished} recorded · {}", cal.live),
                Phase::Ready(_) => format!("Get ready · {}", cal.live),
                Phase::Recording(_) => format!("Recording {}… · {}", label(cal.distance), cal.live),
            }
        };
        cal.update(&mut textures, &status, now);
    }
    trace.line(&json!({"event": "finish", "t": now_ns() as f64 / 1e9}));
    QUIT.store(true, Ordering::Relaxed);
    if let Some(det) = det {
        let _ = det.join();
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
            distance: 2,
            size: 1,
            phase: Phase::Idle,
            done: [1, 2, 0, 0, 0, 0, 0, 0],
            segment: 0,
            frames: 0,
            seen: 0,
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
        assert!(c.distance == 3 && c.done[2] == 1, "on to the next distance");
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
