// guardian: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
// without passthrough. Stereo YOLO detector in detector.h, at a low size and rate to leave the GPU to the game.
//
// Per compositor frame: detections (detector.h) -> measurements (measure.h) -> tracks (tracker.h) -> box and label
// overlays (boxes.h). A "Guardian" tab in the SteamVR dashboard (panel.h) toggles it and what it shows; settings
// persist in ~/.config/guardian.conf (settings.h).
// Run from the app's install directory (guardian.py does): models/, NotoSans-Medium.ttf, guardian.vrmanifest.
//
// Usage: guardian [--max-dist 3] [--size 320] [--max-hz 4] [detector options, detector.h]
//        guardian --no-detector   (only log the VR app's frame stats, as a baseline)
//        guardian --register   (add the overlay's manifest to SteamVR, auto-launch; guardian.py runs it first)
//        guardian --selftest
#include <signal.h>
#include <stdlib.h>
#include <openvr.h>

#include <thread>

#include "boxes.h"
#include "detector.h"
#include "measure.h"
#include "panel.h"
#include "register.h"
#include "selftest.h"
#include "settings.h"
#include "textures.h"
#include "tracker.h"

static bool g_detector = true;  // --no-detector: just log frame stats (baseline for the detector's cost)

static vr::HmdMatrix34_t head_pose(float seconds_from_now, bool& ok) {
    vr::TrackedDevicePose_t p;
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, seconds_from_now, &p, 1);
    ok = p.bPoseIsValid;
    return p.mDeviceToAbsoluteTracking;
}

// Positions (standing universe) of the tracked controllers, seconds_from_now (< 0: in the past).
static std::vector<XrVector3f> controllers(float seconds_from_now) {
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, seconds_from_now, poses,
                                                    vr::k_unMaxTrackedDeviceCount);
    std::vector<XrVector3f> out;
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (poses[i].bPoseIsValid && vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
            const vr::HmdMatrix34_t& m = poses[i].mDeviceToAbsoluteTracking;
            out.push_back({m.m[0][3], m.m[1][3], m.m[2][3]});
        }
    }
    return out;
}

// Every 5 s: the running VR app's presented/dropped/reprojected frames since the last log and its GPU frame time.
static void log_frame_stats() {
    static vr::Compositor_CumulativeStats prev{};
    static uint64_t last = 0;
    uint64_t now = now_ns();
    if (now - last < 5000000000ull || !vr::VRCompositor()) {
        return;
    }
    last = now;
    vr::Compositor_CumulativeStats st{};
    vr::VRCompositor()->GetCumulativeStats(&st, sizeof st);
    vr::Compositor_FrameTiming ft[90];
    ft[0].m_nSize = sizeof ft[0];
    uint32_t n = vr::VRCompositor()->GetFrameTimings(ft, 90);
    float sum = 0, mx = 0;
    for (uint32_t i = 0; i < n; i++) {
        sum += ft[i].m_flTotalRenderGpuMs, mx = std::max(mx, ft[i].m_flTotalRenderGpuMs);
    }
    if (st.m_nPid == prev.m_nPid) {
        printf("frames: %u presented, %u dropped, %u reprojected | gpu %.1f ms avg, %.1f max (last %u)\n",
               st.m_nNumFramePresents - prev.m_nNumFramePresents, st.m_nNumDroppedFrames - prev.m_nNumDroppedFrames,
               st.m_nNumReprojectedFrames - prev.m_nNumReprojectedFrames, n ? sum / n : 0.f, mx, n);
    }
    prev = st;
}

int main(int argc, char** argv) {
    g_det.conf = 0.1f;  // ByteTrack wants the weak detections too; only >= --track-thresh ones start tracks
    track::Tracker tracker;
    if (const char* home = getenv("HOME")) {
        g_conf = std::string(home) + "/.config/guardian.conf";
    }
    if (const char* appid = getenv("SteamAppId")) {
        g_app_key = std::string("steam.overlay.") + appid;
    }
    load_settings();
    g_det.size = g_set.size, g_det.max_hz = g_set.hz;  // --size and --max-hz override them for this run
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + a);
            }
            return argv[++i];
        };
        if (a == "--selftest") {
            return g_conf.clear(), g_set = {}, selftest();
        } else if (a == "--register") {
            return register_app();
        } else if (a == "--max-dist") {
            g_set.max_dist = float(atof(next()));
        } else if (a == "--confirm-hits") {
            tracker.prm.min_hits = atoi(next());
        } else if (a == "--hold-ms") {
            tracker.prm.max_age = float(atof(next())) / 1000;
        } else if (a == "--track-thresh") {
            tracker.prm.high = float(atof(next()));
        } else if (a == "--gate") {
            tracker.prm.gate = float(atof(next()));
        } else if (a == "--accel") {
            tracker.prm.accel = float(atof(next()));
        } else if (a == "--size-alpha") {
            tracker.prm.size_alpha = std::clamp(float(atof(next())), 0.01f, 1.f);
        } else if (a == "--dist-alpha") {
            tracker.prm.dist_alpha = std::clamp(float(atof(next())), 0.01f, 1.f);
        } else if (a == "--no-detector") {
            g_detector = false;
        } else if (parse_detector_option(a, next)) {
            continue;
        } else {
            fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }
    if (!finish_detector_options()) {
        return 2;
    }
    g_set.size = g_det.size, g_set.hz = g_det.max_hz;
    setvbuf(stdout, nullptr, _IOLBF, 0);
    signal(SIGINT, [](int) {
        g_quit = true;
    });
    signal(SIGTERM, [](int) {
        g_quit = true;
    });

    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Overlay);
    if (err) {
        fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err));
        return 1;
    }
    Textures textures;
    printf("guardian: overlay textures via %s\n", textures.init() ? "Vulkan" : "SetOverlayRaw (Vulkan setup failed)");
    if (!boxes::create() || !panel::create(textures)) {
        return 1;
    }
    // A game is starting, loading or running (its intro counts); passthrough itself isn't reported to apps.
    auto game_running = [] {
        return vr::VRApplications()->GetSceneApplicationState() != vr::EVRSceneApplicationState_None;
    };
    bool in_game = game_running();

    // guardian.py makes calib.txt from the driver's copy of the factory calibration: none means driver_guardian wasn't
    // loaded when this SteamVR started (the first run registers it).
    const bool setup_pending = access(g_det.calib.c_str(), R_OK) != 0;
    if (setup_pending) {
        printf("guardian: no %s: restart SteamVR to load driver_guardian\n", g_det.calib.c_str());
    }
    std::thread det;
    if (g_detector && !setup_pending) {
        det = std::thread([] {
            try {
                detect_loop();
            } catch (const std::exception& e) {
                fprintf(stderr, "detector: %s\n", e.what());
                g_quit = true;
            }
        });
    }

    uint64_t seen_det_id = 0, last_det_ns = 0;
    // Playback runs this far behind now: detection age (capture -> here) plus one detection period, smoothed.
    auto period_s = [] {
        float hz = g_det_hz;
        return hz > 0 ? 1 / hz : 0.1f;
    };
    float lag_s = period_s() + 0.1f;
    while (!g_quit) {
        // Once per compositor frame; sleep instead if there's no frame to sync to.
        if (vr::VROverlay()->WaitFrameSync(100) != vr::VROverlayError_None) {
            usleep(11000);
        }
        log_frame_stats();
        vr::VREvent_t ev;
        while (vr::VRSystem()->PollNextEvent(&ev, sizeof ev)) {
            if (ev.eventType == vr::VREvent_Quit) {
                vr::VRSystem()->AcknowledgeQuit_Exiting();
                g_quit = true;
            }
            if (ev.eventType == vr::VREvent_SceneApplicationStateChanged) {
                in_game = game_running();
            }
        }
        panel::poll();
        bool run = g_set.enabled && (in_game || !g_set.games_only);
        g_det_paused = !run;
        if (!run) {
            tracker.tracks.clear();
        }

        DetectionSet d;
        {
            std::lock_guard<std::mutex> lk(g_det_mutex);
            if (g_dets.id != seen_det_id) {
                d = g_dets;
            }
        }
        uint64_t now = now_ns();
        bool ok;
        if (d.id) {
            seen_det_id = d.id;
            last_det_ns = now;
            // Negative prediction = head pose in the past, at capture time.
            float ago = -seconds(now, d.t_capture_ns);
            lag_s += (-ago + period_s() - lag_s) * 0.1f;
            vr::HmdMatrix34_t head = head_pose(ago, ok);
            if (ok) {
                tracker.update(measurements(d.objs, head, controllers(ago)), d.t_capture_ns / 1e9);
            }
        }

        int n = boxes::show(tracker, head_pose(0, ok), now, now / 1e9 - lag_s, textures);

        char st[128];
        if (!g_set.enabled) {
            snprintf(st, sizeof st, "Off");
        } else if (setup_pending) {
            snprintf(st, sizeof st, "Restart SteamVR to finish setup");
        } else if (!run) {
            snprintf(st, sizeof st, "Paused: no game running");
        } else if (!g_detector) {
            snprintf(st, sizeof st, "Detector off (--no-detector)");
        } else if (int s = g_det_loading) {
            snprintf(st, sizeof st, "Loading %dpx model…", s);
        } else if (now - last_det_ns > 2000000000ull) {
            snprintf(st, sizeof st, "Waiting for camera frames");
        } else {
            snprintf(st, sizeof st, "Watching  ·  YOLO %.0f ms  ·  %d in view", g_det_yolo_ms.load(), n);
        }
        panel::update(textures, st);
    }
    if (det.joinable()) {
        det.join();
    }
    vr::VR_Shutdown();
    ncnn::destroy_gpu_instance();
    return 0;
}
