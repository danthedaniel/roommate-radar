//! roommate-radar: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
//! without passthrough. Stereo YOLO detector in detector.rs, at a low size and rate to leave the GPU to the game.
//!
//! Per compositor frame: detections (detector.rs) -> measurements (measure.rs) -> tracks (tracker.rs) -> box and label
//! overlays (boxes.rs). A "Roommate Radar" tab in the SteamVR dashboard (panel.rs) toggles it and what it shows;
//! settings persist in ~/.config/roommate-radar.conf (settings.rs).
//! Steam runs bin/roommate-radar (roommate-radar-argv.json) in the Steam Linux Runtime container; it works from the
//! install directory (models/, NotoSans-Medium.ttf, roommate-radar.vrmanifest), logs to /tmp/roommate-radar.log, and
//! first does the setup an installer would (register.rs).
//!
//! Usage: roommate-radar [--max-dist 3] [--size 320] [--max-hz 4] [detector options, detector.rs] [tracker options]
//!        roommate-radar --no-detector   (only log the VR app's frame stats, as a baseline)
//!        roommate-radar --register   (add the overlay's manifest to SteamVR, auto-launch; run by the setup first)
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
use openvr::{Compositor_CumulativeStats, Compositor_FrameTiming, HmdMatrix34_t, TrackedDevicePose_t, vr};
use shm::now_ns;
use v3::V3;

/// Seconds from `then` to `now` (CLOCK_MONOTONIC ns), signed: negating an unsigned difference wraps around.
fn seconds(now: u64, then: u64) -> f32 {
    now.wrapping_sub(then) as i64 as f32 / 1e9
}

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

/// Every 5 s: the running VR app's presented/dropped/reprojected frames since the last log and its GPU frame time.
fn log_frame_stats(prev: &mut Option<Compositor_CumulativeStats>, last: &mut u64) {
    let now = now_ns();
    if now - *last < 5_000_000_000 {
        return;
    }
    *last = now;
    let mut ft: [Compositor_FrameTiming; 90] = unsafe { std::mem::zeroed() };
    let Some((st, n)) = vr().frame_stats(&mut ft) else {
        return;
    };
    let gpu = ft[..n].iter().map(|f| f.m_flTotalRenderGpuMs);
    let (sum, max) = gpu.fold((0.0, 0f32), |(s, m), g| (s + g, m.max(g)));
    if let Some(p) = prev
        && p.m_nPid == st.m_nPid
    {
        println!(
            "frames: {} presented, {} dropped, {} reprojected | gpu {:.1} ms avg, {max:.1} max (last {n})",
            st.m_nNumFramePresents - p.m_nNumFramePresents,
            st.m_nNumDroppedFrames - p.m_nNumDroppedFrames,
            st.m_nNumReprojectedFrames - p.m_nNumReprojectedFrames,
            if n > 0 { sum / n as f32 } else { 0.0 }
        );
    }
    *prev = Some(st);
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
    if std::env::var_os("NCNN_QUEUE_GLOBAL_PRIORITY").is_none() {
        unsafe { std::env::set_var("NCNN_QUEUE_GLOBAL_PRIORITY", "low") }; // no other threads yet
    }
    register::setup();
}

fn run() -> i32 {
    if !std::env::args().any(|a| a == "--register") {
        start();
    }
    let mut opts = detector::Options { conf: 0.1, ..Default::default() }; // ByteTrack wants the weak detections too
    let mut tracker = tracker::Tracker::default();
    let mut detect = true; // --no-detector: just log frame stats (baseline for the detector's cost)
    let home = std::env::var("HOME").unwrap_or_default();
    let mut set = settings::Settings::load(format!("{home}/.config/roommate-radar.conf").into());
    let app_key = std::env::var("SteamAppId").ok().map(|id| format!("steam.overlay.{id}"));
    (opts.size, opts.max_hz) = (set.size, set.hz); // --size and --max-hz override them for this run

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut next = || args.next().ok_or_else(|| format!("missing value for {a}"));
        let next: &mut dyn FnMut() -> Result<String, String> = &mut next;
        let prm = &mut tracker.prm;
        let parsed = match a.as_str() {
            "--register" => return register::register_app(app_key.as_deref(), &mut set),
            "--max-dist" => detector::num(&a, next).map(|v| set.max_dist = v),
            "--confirm-hits" => detector::num(&a, next).map(|v| prm.min_hits = v as u32),
            "--hold-ms" => detector::num(&a, next).map(|v| prm.max_age = v / 1000.0),
            "--track-thresh" => detector::num(&a, next).map(|v| prm.high = v), // only this confident starts tracks
            "--gate" => detector::num(&a, next).map(|v| prm.gate = v),
            "--accel" => detector::num(&a, next).map(|v| prm.accel = v),
            "--size-alpha" => detector::num(&a, next).map(|v| prm.size_alpha = v.clamp(0.01, 1.0)),
            "--dist-alpha" => detector::num(&a, next).map(|v| prm.dist_alpha = v.clamp(0.01, 1.0)),
            "--no-detector" => {
                detect = false;
                Ok(())
            }
            _ => match opts.parse(&a, next) {
                Ok(false) => Err(format!("unknown option {a}")),
                r => r.map(|_| ()),
            },
        };
        if let Err(e) = parsed {
            eprintln!("{e}");
            return 2;
        }
    }
    (set.size, set.hz) = (opts.size, opts.max_hz);
    detector::SIZE.store(opts.size, Ordering::Relaxed);
    detector::HZ.store(opts.max_hz);
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
    let via = if textures.vulkan() { "Vulkan" } else { "SetOverlayRaw (Vulkan setup failed)" };
    println!("roommate-radar: overlay textures via {via}");
    let mut panel = panel::Panel::new(app_key);
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
    let setup_pending = !std::path::Path::new(&opts.calib).join("xrservice.json").exists();
    if setup_pending {
        println!("roommate-radar: no calibration in {}: restart SteamVR to load driver_roommate_radar", opts.calib);
    }
    let det = (detect && !setup_pending).then(|| {
        let opts = opts.clone();
        std::thread::spawn(move || {
            if let Err(e) = detector::run_loop(opts) {
                eprintln!("detector: {e}");
                QUIT.store(true, Ordering::Relaxed);
            }
        })
    });

    let (mut seen_det_id, mut last_det_ns) = (0, 0);
    // Playback runs this far behind now: detection age (capture -> here) plus one detection period, smoothed.
    let period_s = || {
        let hz = detector::HZ.load();
        if hz > 0.0 { 1.0 / hz } else { 0.1 }
    };
    let mut lag_s = period_s() + 0.1;
    let (mut prev_stats, mut last_stats) = (None, 0);
    while !QUIT.load(Ordering::Relaxed) {
        // Once per compositor frame; sleep instead if there's no frame to sync to.
        if !vr().wait_frame_sync(100) {
            std::thread::sleep(Duration::from_millis(11));
        }
        log_frame_stats(&mut prev_stats, &mut last_stats);
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
        let run = set.enabled && (in_game || !set.games_only);
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
            let ago = -seconds(now, d.t_capture_ns);
            lag_s += (-ago + period_s() - lag_s) * 0.1;
            if let (head, true) = head_pose(ago) {
                let ms = measure::measurements(&d.objs, &head, &controllers(ago), &set, &opts);
                tracker.update(&ms, d.t_capture_ns as f64 / 1e9);
            }
        }

        let n = boxes.show(&tracker, &head_pose(0.0).0, now, now as f64 / 1e9 - lag_s as f64, &mut textures, &opts);

        let status = if !set.enabled {
            "Off".to_string()
        } else if setup_pending {
            "Restart SteamVR to finish setup".into()
        } else if !run {
            "Paused: no game running".into()
        } else if !detect {
            "Detector off (--no-detector)".into()
        } else if detector::LOADING.load(Ordering::Relaxed) != 0 {
            format!("Loading {}px model…", detector::LOADING.load(Ordering::Relaxed))
        } else if now - last_det_ns > 2_000_000_000 {
            "Waiting for camera frames".into()
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

#[cfg(test)]
mod tests {
    #[test]
    fn signed_seconds_between_timestamps() {
        assert!(-super::seconds(3_000_000_000, 2_950_000_000) == -0.05 && super::seconds(1, 2) < 0.0);
    }
}
