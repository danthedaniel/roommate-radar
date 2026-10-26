//! The app's setup with SteamVR, done on every start since Steam has no install scripts on Linux:
//! - driver/ (driver_roommate_radar, the camera frames) registered with vrpathreg; it loads on SteamVR's next start
//! - roommate-radar.vrmanifest (the overlay, launched through Steam) written, and added by `roommate-radar --register`
//!   where Steam hasn't, with auto-launch.
//!
//! SteamVR knows an overlay app started by Steam as steam.overlay.<SteamAppId>. A Store install's manifest for that key
//! comes from Steam ("Launch SteamVR Overlay"); a devkit install has ours. Adding it has to happen before the overlay
//! connects: SteamVR then makes up an entry for the key (a "binary" app, not a dashboard overlay) and skips manifests
//! describing it until it restarts, hence --register as a process of its own, first. Once the manifest's entry is
//! there, auto-launch is switched on, once (the user may turn it off).
use std::process::Command;

use serde_json::{Value, json};

use crate::openvr::{self, vr};
use crate::settings::Settings;

/// From the install directory, before the overlay starts.
pub fn setup() {
    register_driver();
    if let (Ok(appid), Ok(gameid)) = (std::env::var("SteamAppId"), std::env::var("SteamGameId")) {
        write_manifest(&appid, &gameid);
    }
    if let Ok(exe) = std::env::current_exe() {
        let _ = Command::new(exe).arg("--register").status();
    }
}

fn register_driver() {
    let path = format!("{}/.config/openvr/openvrpaths.vrpath", std::env::var("HOME").unwrap_or_default());
    let Some(paths) = std::fs::read_to_string(&path).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()) else {
        return eprintln!("roommate-radar: can't read {path}; driver not registered");
    };
    let driver = std::env::current_dir().unwrap_or_default().join("driver").to_string_lossy().into_owned();
    if paths["external_drivers"].as_array().is_some_and(|d| d.iter().any(|p| *p == driver)) {
        return;
    }
    let Some(runtime) = paths["runtime"][0].as_str() else {
        return eprintln!("roommate-radar: no runtime in {path}; driver not registered");
    };
    let _ = Command::new(format!("{runtime}/bin/vrpathreg.sh")).args(["adddriver", &driver]).status();
    println!("roommate-radar: registered {driver} with SteamVR; it loads when SteamVR next starts");
}

/// The entry Steam generates for a "Launch SteamVR Overlay" app.
fn write_manifest(appid: &str, gameid: &str) {
    let manifest = json!({
        "source": "user",
        "applications": [{
            "app_key": format!("steam.overlay.{appid}"),
            "launch_type": "url",
            "url": format!("steam://rungameid/{gameid}"),
            "is_dashboard_overlay": true,
            "strings": {
                "en_us": {"name": "Roommate Radar", "description": "Outlines people, cats and dogs over any VR app"},
            },
        }],
    });
    if let Err(e) = std::fs::write("roommate-radar.vrmanifest", serde_json::to_string_pretty(&manifest).unwrap()) {
        eprintln!("roommate-radar: roommate-radar.vrmanifest: {e}");
    }
}

/// roommate-radar --register
pub fn register_app(app_key: Option<&str>, set: &mut Settings) -> i32 {
    let Some(key) = app_key else {
        return 0; // not started by Steam
    };
    if let Err(e) = openvr::init(openvr::EVRApplicationType_VRApplication_Utility) {
        eprintln!("VR_Init: {e}");
        return 1;
    }
    if !vr().is_dashboard_overlay(key)
        && let Ok(path) = std::fs::canonicalize("roommate-radar.vrmanifest")
    {
        let e = vr().add_manifest(&path.to_string_lossy());
        println!("roommate-radar: AddApplicationManifest({}): {}", path.display(), vr().app_error_name(e));
    }
    if !vr().is_dashboard_overlay(key) {
        println!("roommate-radar: {key} isn't a dashboard overlay yet; restart SteamVR");
    } else if !set.autolaunch_set {
        let e = vr().set_auto_launch(key, true);
        println!("roommate-radar: SetApplicationAutoLaunch({key}): {}", vr().app_error_name(e));
        set.autolaunch_set = e == 0;
        set.save();
    }
    openvr::shutdown();
    0
}
