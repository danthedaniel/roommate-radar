//! roommate-radar: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
//! without passthrough. Stereo YOLO detector in detector.rs, at a low size and rate to leave the GPU to the game.
//!
//! Per compositor frame: detections (detector.rs) -> measurements (measure.rs) -> tracks (tracker.rs) -> box and label
//! overlays (outlines.rs). A "Roommate Radar" tab in the SteamVR dashboard (panel.rs) toggles it and what it shows;
//! settings persist in ~/.config/roommate-radar.conf (settings.rs).
//! Steam runs bin/roommate-radar (roommate_radar-argv.json) in the Steam Linux Runtime container; it works from the
//! install directory (models/, NotoSans-Medium.ttf, roommate-radar.vrmanifest), logs to /tmp/roommate-radar.log, and
//! first does the setup an installer would (register.rs).
mod assign;
mod detector;
mod draw;
mod measure;
mod ncnn;
mod noise_calib;
mod openvr;
mod openvr_sys;
mod outlines;
mod panel;
mod rectify;
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
use v3::V3;

// The head moving more than this (m) between two frames is tracking relocalizing: the standing universe moved (6 m
// and 37 deg on the Frame, when it recognized the room a minute after standby) and the tracks are elsewhere now.
// The ChaperoneUniverseHasChanged clear below didn't catch it (the trace kept the old track).
const POSE_JUMP_M: f32 = 0.5;

/// The head's pose (standing universe) seconds_from_now (< 0: in the past), and whether it's valid.
fn head_pose(seconds_from_now: f32) -> (HmdMatrix34_t, bool) {
    let mut pose: [TrackedDevicePose_t; 1] = unsafe { std::mem::zeroed() };
    vr().poses(seconds_from_now, &mut pose);
    (pose[0].mDeviceToAbsoluteTracking, pose[0].bPoseIsValid)
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
        .map(|i| v3::position(&poses[i].mDeviceToAbsoluteTracking))
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
        let r = std::panic::catch_unwind(detector::run_loop)
            .unwrap_or_else(|_| Err("Detector crashed (see the log)".into()));
        if let Err(e) = r {
            eprintln!("detector: {e}");
            detector::LOADING_SIZE.store(0, Ordering::Relaxed);
            *detector::PROBLEM.lock().unwrap_or_else(|poisoned| poisoned.into_inner()) = e;
        }
    }))
}

fn run() -> i32 {
    match std::env::args().nth(1).as_deref() {
        Some("--register") => return register::register_app(),
        Some("--calibrate") => {
            start(noise_calib::LOG_PATH); // its own install: no setup (noise_calib.rs)
            return noise_calib::run();
        }
        _ => {}
    }
    start("/tmp/roommate-radar.log");
    register::setup();
    let mut tracker = tracker::Tracker::default();
    let mut trace = trace::Trace::new();
    let mut settings = settings::Settings::load_saved();
    // How the driver starts the app with SteamVR (driver/autostart.rs)
    if let Some(gameid) = std::env::var("SteamGameId").ok().and_then(|g| g.parse().ok())
        && settings.gameid != Some(gameid)
    {
        settings.gameid = Some(gameid);
        settings.save();
    }
    detector::SIZE.store(settings.size, Ordering::Relaxed);
    detector::HZ.store(settings.hz);

    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Overlay) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    let mut textures = textures::Textures::new();
    let mut panel = panel::Panel::default();
    let mut outlines =
        match outlines::Outlines::create().and_then(|b| panel.create(&mut textures, &settings).map(|_| b)) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("{e}");
                return 1;
            }
        };
    let mut in_game = vr().game_running();
    let mut walls = vr().boundary_walls();

    let mut selfcal = selfcal::SelfCal::new(settings.correction); // before the detector starts
    let mut saved_correction_ns = shm::now_ns();
    let detector_thread = spawn_detector();
    let setup_pending = detector_thread.is_none();

    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    // Playback runs this far behind now: detection age (capture -> here) plus one detection period, smoothed.
    let mut lag_s = 1.0 / settings.hz + 0.1;
    let mut last_head_pos: Option<V3> = None;
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
                walls = vr().boundary_walls();
                tracker.tracks.clear(); // in the standing universe, which may have moved
            }
        }
        let (head_now, head_valid) = head_pose(0.0);
        if head_valid {
            let head_pos = v3::position(&head_now);
            if last_head_pos.is_some_and(|last| (head_pos - last).len() > POSE_JUMP_M) {
                tracker.tracks.clear();
            }
            last_head_pos = Some(head_pos);
        }
        panel.poll(&mut settings);
        let detecting = settings.warning_acked && settings.enabled && (in_game || !settings.games_only); // the tab's first-run warning first

        detector::PAUSED.store(!detecting, Ordering::Relaxed);
        if !detecting {
            tracker.tracks.clear();
        }

        let d = {
            let d = detector::DETECTIONS.lock().unwrap_or_else(|poisoned| poisoned.into_inner()); // the detector may have crashed
            (d.id != seen_det_id).then(|| d.clone())
        };
        let now_ns = shm::now_ns();
        let now_s = now_ns as f64 / 1e9;
        if let Some(d) = d {
            seen_det_id = d.id;
            last_det_ns = now_ns;
            // Negative prediction = head pose in the past, at capture time.
            let from_now_s = (d.t_capture_ns as i64 - now_ns as i64) as f32 / 1e9;
            lag_s += (-from_now_s + 1.0 / settings.hz - lag_s) * 0.1;
            if let (head_at_capture, true) = head_pose(from_now_s) {
                let active_walls: &[_] = if settings.play_area_only { &walls } else { &[] };
                let ctrl = controllers(from_now_s);
                let meas = measure::measurements(&d.objs, &head_at_capture, &ctrl, active_walls, &settings);
                // A track missing where the views can't see it, or out of range, isn't gone.
                let view = d.view.as_deref();
                let seen =
                    |p| view.is_none_or(|v| measure::observable(v, &head_at_capture, p, active_walls, &settings));
                tracker.update(&meas, d.t_capture_ns as f64 / 1e9, seen);
                let mut extra = serde_json::Map::new();
                if let Some(matches) = &d.align_matches {
                    let r = |v: f32| (v * 100.0).round() / 100.0;
                    extra.insert(
                        "align".into(),
                        serde_json::json!(matches.iter().map(|m| m.map(r)).collect::<Vec<_>>()),
                    );
                }
                trace.frame(&d, &head_at_capture, &ctrl, &meas, &tracker, extra);
                selfcal.step(&d, &head_at_capture);
                // saved at most once a minute, for the next start
                if selfcal.moved_from(settings.correction) && now_ns - saved_correction_ns > 60_000_000_000 {
                    (settings.correction, saved_correction_ns) = (selfcal.correction, now_ns);
                    settings.save();
                }
            }
        }

        let shown = outlines.show(&tracker, &head_now, now_ns, now_s - lag_s as f64, &mut textures);

        let status = if !settings.enabled {
            "Off".to_string()
        } else if setup_pending {
            "Restart SteamVR to finish setup".into()
        } else if !detecting {
            "Paused: no game running".into()
        } else if detector::LOADING_SIZE.load(Ordering::Relaxed) != 0 && !detector::WAITING.load(Ordering::Relaxed) {
            format!("Loading {}px model…", detector::LOADING_SIZE.load(Ordering::Relaxed))
        } else if detector::WAITING.load(Ordering::Relaxed) || now_ns - last_det_ns > 2_000_000_000 {
            let problem = detector::PROBLEM.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
            if problem.is_empty() { "Waiting for camera frames".into() } else { problem.clone() }
        } else if cfg!(feature = "developer") {
            format!("Watching  ·  YOLO {:.0} ms  ·  {shown} in view", YOLO_MS.load())
        } else {
            format!("Watching  ·  {shown} in view")
        };
        panel.update(&mut textures, &status, &settings);
    }
    if let Some(detector_thread) = detector_thread {
        let _ = detector_thread.join();
    }
    openvr::shutdown();
    ncnn::destroy_gpu_instance();
    0
}
