//! roommate-radar: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
//! without passthrough. Stereo YOLO detector in detector.rs, at a low size and rate to leave the GPU to the game.
//!
//! Per compositor frame: detections (detector.rs) -> measurements (measure.rs) -> tracks (tracker.rs) -> box and label
//! overlays (boxes.rs); people and pets beside you, where there's no box, as chevrons (hud.rs). A "Roommate Radar" tab in the SteamVR dashboard (panel.rs) toggles it and what it shows;
//! settings persist in ~/.config/roommate-radar.conf (settings.rs).
//! Steam runs bin/roommate-radar (roommate_radar-argv.json) in the Steam Linux Runtime container; it works from the
//! install directory (models/, NotoSans-Medium.ttf, roommate-radar.vrmanifest), logs to /tmp/roommate-radar.log, and
//! first does the setup an installer would (register.rs).
mod boxes;
mod calib;
mod calibrate;
mod detector;
mod draw;
mod hud;
mod measure;
mod ncnn;
mod openvr;
mod openvr_sys;
mod panel;
mod register;
mod selfcal;
mod settings;
#[allow(dead_code)] // the writer's half is camserver's
mod shm;
mod stereo;
mod textures;
mod trace;
mod tracker;
mod v3;

use std::sync::atomic::Ordering;
use std::time::Duration;

use detector::{QUIT, YOLO_MS};
use openvr::{HmdMatrix34_t, TrackedDevicePose_t, vr};
use shm::now_ns;
use v3::V3;

// The head moving more than this (m) between two frames is tracking relocalizing: the standing universe moved (6 m
// and 37 deg on the Frame, when it recognized the room a minute after standby) and the tracks are elsewhere now.
// The ChaperoneUniverseHasChanged clear below didn't catch it (the trace kept the old track).
const POSE_JUMP_M: f32 = 0.5;

/// A one-view person or pet this far (rad) off the head's forward is at the edge of the SLAM views: a chevron, as for the side
/// cameras, unless a box is shown there.
const EDGE_RAD: f32 = 0.52; // 30 deg; the views reach 50

/// The head's pose (standing universe) seconds_from_now (< 0: in the past), and whether it's valid.
fn head_pose(seconds_from_now: f32) -> (HmdMatrix34_t, bool) {
    let mut p: [TrackedDevicePose_t; 1] = unsafe { std::mem::zeroed() };
    vr().poses(seconds_from_now, &mut p);
    (p[0].mDeviceToAbsoluteTracking, p[0].bPoseIsValid)
}

/// Positions (standing universe) of the tracked controllers, seconds_from_now.
fn controllers(seconds_from_now: f32) -> Vec<V3> {
    let mut poses: [TrackedDevicePose_t; openvr::MAX_TRACKED_DEVICES] = unsafe { std::mem::zeroed() };
    vr().poses(seconds_from_now, &mut poses);
    (0..poses.len())
        .filter(|&i| {
            poses[i].bPoseIsValid
                && vr().device_class(i as u32) == openvr::ETrackedDeviceClass_TrackedDeviceClass_Controller
        })
        .map(|i| measure::position(&poses[i].mDeviceToAbsoluteTracking))
        .collect()
}

fn main() {
    std::process::exit(run());
}

/// Into the install directory (bin/..), output to `log`.
fn start(log: &str) {
    if let Some(dir) = std::env::current_exe().ok().and_then(|e| Some(e.parent()?.parent()?.to_path_buf())) {
        let _ = std::env::set_current_dir(dir);
    }
    use std::os::fd::AsRawFd;
    use std::os::unix::fs::OpenOptionsExt;
    match std::fs::OpenOptions::new().append(true).create(true).mode(0o644).open(log) {
        Ok(f) => unsafe {
            libc::dup2(f.as_raw_fd(), 1);
            libc::dup2(f.as_raw_fd(), 2);
        },
        Err(e) => eprintln!("{log}: {e}"),
    }
    // Low GPU priority: the game preempts the detector (patched ncnn, the third_party/ncnn submodule)
    unsafe { std::env::set_var("NCNN_QUEUE_GLOBAL_PRIORITY", "low") }; // no other threads yet
    // Quit cleanly on SIGTERM (Steam stopping it, deploy.yml) and SIGINT
    extern "C" fn quit(_: libc::c_int) {
        QUIT.store(true, Ordering::Relaxed);
    }
    unsafe {
        libc::signal(libc::SIGINT, quit as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, quit as *const () as libc::sighandler_t);
    }
}

/// The detector thread, unless the driver hasn't been loaded yet: then there's no calibration (the driver copies the
/// factory one when it loads; the first run registers it) and nothing to detect from.
fn spawn_detector() -> Option<std::thread::JoinHandle<()>> {
    if !std::path::Path::new(shm::CALIB_DIR).join("xrservice.json").exists() {
        println!("roommate-radar: no calibration in {}: restart SteamVR to load driver_roommate_radar", shm::CALIB_DIR);
        return None;
    }
    Some(std::thread::spawn(|| {
        // Stays up to show the error in the tab
        if let Err(e) = detector::run_loop() {
            eprintln!("detector: {e}");
            detector::LOADING.store(0, Ordering::Relaxed);
            *detector::PROBLEM.lock().unwrap() = e;
        }
    }))
}

fn run() -> i32 {
    match std::env::args().nth(1).as_deref() {
        Some("--register") => return register::register_app(),
        Some("--calibrate") => {
            start(calibrate::LOG); // its own install: no setup (calibrate.rs)
            return calibrate::run();
        }
        _ => {}
    }
    start("/tmp/roommate-radar.log");
    register::setup();
    let mut tracker = tracker::Tracker::default();
    let mut trace = trace::Trace::new();
    let home = std::env::var("HOME").unwrap_or_default();
    let mut set = settings::Settings::load(format!("{home}/{}", shm::CONF).into());
    // How the driver starts the app with SteamVR (driver/autostart.rs)
    if let Some(gameid) = std::env::var("SteamGameId").ok().and_then(|g| g.parse().ok())
        && set.gameid != Some(gameid)
    {
        set.gameid = Some(gameid);
        set.save();
    }
    detector::SIZE.store(set.size, Ordering::Relaxed);
    detector::HZ.store(set.hz);

    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Overlay) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    let mut textures = textures::Textures::new();
    let mut panel = panel::Panel::default();
    let made = boxes::Boxes::create().and_then(|b| Ok((b, hud::Hud::create()?)));
    let (mut boxes, mut hud) = match made.and_then(|b| panel.create(&mut textures, &set).map(|_| b)) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    let mut in_game = vr().game_running();
    let mut walls = vr().play_area_walls();

    let mut selfcal = selfcal::SelfCal::new(set.align); // before the detector starts
    let mut saved_align_ns = now_ns();
    let det = spawn_detector();
    let setup_pending = det.is_none();

    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    // Playback runs this far behind now: detection age (capture -> here) plus one detection period, smoothed.
    let mut lag_s = 1.0 / set.hz + 0.1;
    let mut last_head: Option<V3> = None;
    while !QUIT.load(Ordering::Relaxed) {
        // Once per compositor frame; sleep instead if there's no frame to sync to.
        if !vr().wait_frame_sync(100) {
            std::thread::sleep(Duration::from_millis(11));
        }
        while let Some(ev) = vr().poll_event() {
            if ev.eventType == openvr::EVREventType_VREvent_Quit {
                vr().acknowledge_quit();
                QUIT.store(true, Ordering::Relaxed);
            }
            if ev.eventType == openvr::EVREventType_VREvent_SceneApplicationStateChanged {
                in_game = vr().game_running();
            }
            if ev.eventType == openvr::EVREventType_VREvent_ChaperoneDataHasChanged
                || ev.eventType == openvr::EVREventType_VREvent_ChaperoneUniverseHasChanged
            {
                walls = vr().play_area_walls();
                tracker.tracks.clear(); // in the standing universe, which may have moved
            }
        }
        let (head_now, head_valid) = head_pose(0.0);
        if head_valid {
            let p = measure::position(&head_now);
            if last_head.is_some_and(|q| (p - q).len() > POSE_JUMP_M) {
                tracker.tracks.clear();
            }
            last_head = Some(p);
        }
        panel.poll(&mut set);
        let run = set.warned && set.enabled && (in_game || !set.games_only); // the tab's first-run warning first

        detector::PAUSED.store(!run, Ordering::Relaxed);
        if !run {
            tracker.tracks.clear();
        }

        let d = {
            let d = detector::DETS.lock().unwrap();
            (d.id != seen_det_id).then(|| d.clone())
        };
        let now = now_ns();
        let now_s = now as f64 / 1e9;
        if let Some(d) = d {
            seen_det_id = d.id;
            last_det_ns = now;
            // Negative prediction = head pose in the past, at capture time.
            let ago = (d.t_capture_ns as i64 - now as i64) as f32 / 1e9;
            lag_s += (-ago + 1.0 / set.hz - lag_s) * 0.1;
            if let (head, true) = head_pose(ago) {
                let walls: &[_] = if set.play_area { &walls } else { &[] };
                let ctrl = controllers(ago);
                let ms = measure::measurements(&d.objs, &head, 0.0, &ctrl, walls, &set);
                // A track missing where the views can't see it, or out of range, isn't gone.
                let view = d.view.as_deref();
                let seen = |p| view.is_none_or(|v| measure::observable(v, &head, p, walls, &set));
                let t_capture = d.t_capture_ns as f64 / 1e9;
                tracker.update(&ms, t_capture, seen);
                // People and pets seen at the edge of the views only (one view: weak, which doesn't confirm a track); hud.rs
                // hides any chevron that has a box
                let chevron = |m: &&tracker::Meas| m.score >= tracker::HIGH && measure::near_by_size(m, &set);
                for m in ms.iter().filter(|m| m.weak && chevron(m)) {
                    let q = measure::to_head(&head, m.p);
                    if q.x.atan2(-q.z).abs() > EDGE_RAD {
                        hud.saw((q.x > 0.0) as usize, m.cls, m.score, m.p, now_s, "edge of view");
                    }
                }
                // and by the side cameras, each at its own capture time
                for (k, (objs, &t)) in d.side.iter().flat_map(|s| s.objs.iter().zip(&s.t_capture_ns).enumerate()) {
                    let ago = (t as i64 - now as i64) as f32 / 1e9;
                    if let (head, true) = head_pose(ago)
                        && t != 0
                    {
                        let ms =
                            measure::measurements(objs, &head, detector::SIDE_YAW[k], &controllers(ago), walls, &set);
                        for m in ms.iter().filter(chevron) {
                            hud.saw(k, m.cls, m.score, m.p, now_s, "side camera");
                        }
                    }
                }
                let mut extra = serde_json::Map::new();
                if let Some(a) = &d.align {
                    let r = |v: f32| (v * 100.0).round() / 100.0;
                    extra.insert("align".into(), serde_json::json!(a.iter().map(|p| p.map(r)).collect::<Vec<_>>()));
                }
                trace.frame(&d, &head, &ctrl, &ms, &tracker, extra);
                selfcal.step(&d, &head);
                // saved at most once a minute, for the next start
                if selfcal.moved_from(set.align) && now - saved_align_ns > 60_000_000_000 {
                    (set.align, saved_align_ns) = (selfcal.omega, now);
                    set.save();
                }
            }
        }

        boxes.show(&tracker, &head_now, now, now_s - lag_s as f64, &mut textures);
        let n = boxes.drawn.len();
        // The boxes, and the tracks matched lately that aren't boxes yet (tentative) or any more
        let recent = tracker.tracks.iter().filter(|t| now_s - t.last_seen < tracker::SHOW_S).map(|t| t.p);
        let ahead: Vec<V3> = boxes.drawn.iter().copied().chain(recent).collect();
        hud.show(now_s, &head_now, &ahead, &mut textures);

        let status = if !set.enabled {
            "Off".to_string()
        } else if setup_pending {
            "Restart SteamVR to finish setup".into()
        } else if !run {
            "Paused: no game running".into()
        } else if detector::LOADING.load(Ordering::Relaxed) != 0 && !detector::WAITING.load(Ordering::Relaxed) {
            format!("Loading {}px model…", detector::LOADING.load(Ordering::Relaxed))
        } else if detector::WAITING.load(Ordering::Relaxed) || now - last_det_ns > 2_000_000_000 {
            let p = detector::PROBLEM.lock().unwrap();
            if p.is_empty() { "Waiting for camera frames".into() } else { p.clone() }
        } else if cfg!(feature = "developer") {
            format!("Watching  ·  YOLO {:.0} ms  ·  {n} in view", YOLO_MS.load())
        } else {
            format!("Watching  ·  {n} in view")
        };
        panel.update(&mut textures, &status, &set);
    }
    if let Some(det) = det {
        let _ = det.join();
    }
    openvr::shutdown();
    ncnn::destroy_gpu_instance();
    0
}
