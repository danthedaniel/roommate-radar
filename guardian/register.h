// guardian --register, run by guardian.py before the overlay. SteamVR knows an overlay app started by Steam as
// steam.overlay.<SteamAppId>. A Store install's manifest for that key comes from Steam ("Launch SteamVR Overlay"); a
// devkit install has the one guardian.py writes, added here. That has to happen before the overlay connects: SteamVR
// then makes up an entry for the key (a "binary" app, not a dashboard overlay) and skips manifests describing it until
// it restarts. Once the manifest's entry is there, auto-launch is switched on, once (the user may turn it off).
#pragma once
#include <limits.h>
#include <stdlib.h>
#include <openvr.h>

#include "settings.h"

static std::string g_app_key;  // SteamVR's key for this app, steam.overlay.<SteamAppId>; empty if not started by Steam

static int register_app() {
    if (g_app_key.empty()) {
        return 0;  // not started by Steam
    }
    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Utility);
    if (err) {
        fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err));
        return 1;
    }
    const std::string& key = g_app_key;
    auto* apps = vr::VRApplications();
    auto described = [&] {
        vr::EVRApplicationError pe;
        return apps->GetApplicationPropertyBool(key.c_str(), vr::VRApplicationProperty_IsDashboardOverlay_Bool, &pe);
    };
    char path[PATH_MAX];
    if (!described() && realpath("guardian.vrmanifest", path)) {
        vr::EVRApplicationError e = apps->AddApplicationManifest(path, false);
        printf("guardian: AddApplicationManifest(%s): %s\n", path, apps->GetApplicationsErrorNameFromEnum(e));
    }
    if (!described()) {
        printf("guardian: %s isn't a dashboard overlay yet; restart SteamVR\n", key.c_str());
    } else if (!g_set.autolaunch_set) {
        vr::EVRApplicationError e = apps->SetApplicationAutoLaunch(key.c_str(), true);
        printf("guardian: SetApplicationAutoLaunch(%s): %s\n", key.c_str(), apps->GetApplicationsErrorNameFromEnum(e));
        g_set.autolaunch_set = !e;
        save_settings();
    }
    vr::VR_Shutdown();
    return 0;
}
