//! roommate-radar: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
//! without passthrough. Stereo YOLO detector in detector.rs, at a low size and rate to leave the GPU to the game.
//!
//! Per compositor frame: detections (detector.rs) -> measurements (measure.rs) -> tracks (tracker.rs) -> box and label
//! overlays (boxes.rs). A "Roommate Radar" tab in the SteamVR dashboard (panel.rs) toggles it and what it shows;
//! settings persist in ~/.config/roommate-radar.conf (settings.rs).
//! Steam runs bin/roommate-radar (roommate_radar-argv.json) in the Steam Linux Runtime container; it works from the
//! install directory (models/, NotoSans-Medium.ttf, roommate-radar.vrmanifest), logs to /tmp/roommate-radar.log, and
//! first does the setup an installer would (register.rs).
mod boxes;
mod calib;
mod detector;
mod draw;
mod measure;
mod ncnn;
mod openvr;
mod openvr_sys;
mod panel;
mod register;
mod settings;
#[allow(dead_code)] // the writer's half is camserver's
mod shm;
mod stereo;
mod textures;
mod tracker;
mod v3;

use std::sync::atomic::Ordering;
use std::time::Duration;

use detector::{QUIT, YOLO_MS};
use openvr::{HmdMatrix34_t, TrackedDevicePose_t, vr};
use shm::now_ns;
use v3::V3;

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

/// Into the install directory (bin/..), output to the log, then register.rs's setup.
fn start() {
    if let Some(dir) = std::env::current_exe().ok().and_then(|e| Some(e.parent()?.parent()?.to_path_buf())) {
        let _ = std::env::set_current_dir(dir);
    }
    use std::os::fd::AsRawFd;
    use std::os::unix::fs::OpenOptionsExt;
    match std::fs::OpenOptions::new().append(true).create(true).mode(0o644).open("/tmp/roommate-radar.log") {
        Ok(log) => unsafe {
            libc::dup2(log.as_raw_fd(), 1);
            libc::dup2(log.as_raw_fd(), 2);
        },
        Err(e) => eprintln!("/tmp/roommate-radar.log: {e}"),
    }
    // Low GPU priority: the game preempts the detector (patched ncnn, the third_party/ncnn submodule)
    unsafe { std::env::set_var("NCNN_QUEUE_GLOBAL_PRIORITY", "low") }; // no other threads yet
    register::setup();
}

fn run() -> i32 {
    if std::env::args().nth(1).as_deref() == Some("--register") {
        return register::register_app();
    }
    start();
    let mut tracker = tracker::Tracker::default();
    let home = std::env::var("HOME").unwrap_or_default();
    let mut set = settings::Settings::load(format!("{home}/.config/roommate-radar.conf").into());
    // How the driver starts the app with SteamVR (driver/autostart.rs)
    if let Some(gameid) = std::env::var("SteamGameId").ok().and_then(|g| g.parse().ok())
        && set.gameid != Some(gameid)
    {
        set.gameid = Some(gameid);
        set.save();
    }
    detector::SIZE.store(set.size, Ordering::Relaxed);
    detector::HZ.store(set.hz);
    extern "C" fn quit(_: libc::c_int) {
        QUIT.store(true, Ordering::Relaxed);
    }
    unsafe {
        libc::signal(libc::SIGINT, quit as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, quit as *const () as libc::sighandler_t);
    }

    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Overlay) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    let mut textures = textures::Textures::new();
    let mut panel = panel::Panel::default();
    let mut boxes = match boxes::Boxes::create().and_then(|b| panel.create(&mut textures, &set).map(|_| b)) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    let mut in_game = vr().game_running();

    // The driver copies the factory calibration when it loads: none means driver_roommate_radar wasn't loaded when this
    // SteamVR started (the first run registers it).
    let setup_pending = !std::path::Path::new(shm::CALIB_DIR).join("xrservice.json").exists();
    if setup_pending {
        println!("roommate-radar: no calibration in {}: restart SteamVR to load driver_roommate_radar", shm::CALIB_DIR);
    }
    let det = (!setup_pending).then(|| {
        std::thread::spawn(|| {
            // Stays up to show the error in the tab
            if let Err(e) = detector::run_loop() {
                eprintln!("detector: {e}");
                detector::LOADING.store(0, Ordering::Relaxed);
                *detector::PROBLEM.lock().unwrap() = e;
            }
        })
    });

    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    // Playback runs this far behind now: detection age (capture -> here) plus one detection period, smoothed.
    let mut lag_s = 1.0 / set.hz + 0.1;
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
        if let Some(d) = d {
            seen_det_id = d.id;
            last_det_ns = now;
            // Negative prediction = head pose in the past, at capture time.
            let ago = (d.t_capture_ns as i64 - now as i64) as f32 / 1e9;
            lag_s += (-ago + 1.0 / set.hz - lag_s) * 0.1;
            if let (head, true) = head_pose(ago) {
                // ponytail: boundary fetched per detection (cheap IPC at <= 16 Hz); cache it on ChaperoneDataHasChanged if not
                let walls = if set.play_area { vr().play_area_walls() } else { vec![] };
                let ms = measure::measurements(&d.objs, &head, &controllers(ago), &walls, &set);
                tracker.update(&ms, d.t_capture_ns as f64 / 1e9);
            }
        }

        let n = boxes.show(&tracker, &head_pose(0.0).0, now, now as f64 / 1e9 - lag_s as f64, &mut textures);

        let status = if !set.enabled {
            "Off".to_string()
        } else if setup_pending {
            "Restart SteamVR to finish setup".into()
        } else if !run {
            "Paused: no game running".into()
        } else if detector::LOADING.load(Ordering::Relaxed) != 0 {
            format!("Loading {}px model…", detector::LOADING.load(Ordering::Relaxed))
        } else if now - last_det_ns > 2_000_000_000 {
            let p = detector::PROBLEM.lock().unwrap();
            if p.is_empty() { "Waiting for camera frames".into() } else { p.clone() }
        } else {
            format!("Watching  ·  YOLO {:.0} ms  ·  {n} in view", YOLO_MS.load())
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
