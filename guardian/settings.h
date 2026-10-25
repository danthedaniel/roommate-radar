// guardian's settings: set from the dashboard tab (panel.h), saved to ~/.config/guardian.conf.
#pragma once
#include <algorithm>
#include <cstdio>
#include <iterator>
#include <string>

struct Settings {
    bool enabled = true, people = true, pets = true;  // pets: cats and dogs
    bool hands = true;                                // ignore boxes around a controller: your hands (holds_controller)
    bool games_only = false;                          // pause while no game is running (SteamVR home, dashboard)
    float max_dist = 3;                               // ignore detections farther than this (m, horizontal)
    int size = 320;                                   // YOLO per-view size, one of kSizes
    float hz = 4;                                     // YOLO runs per second, one of kRates
    bool autolaunch_set = false;                      // not in the tab: auto-launch was switched on once (--register)
};
static const int kSizes[] = {320, 480, 640};
static const float kRates[] = {2, 4, 8, 16};
static Settings g_set;
static std::string g_conf;  // empty: don't save (selftest)

static void load_settings() {
    FILE* f = fopen(g_conf.c_str(), "r");
    if (!f) {
        return;
    }
    char k[32];
    float v;
    while (fscanf(f, " %31[^=]=%f", k, &v) == 2) {
        std::string key = k;
        if (key == "enabled") {
            g_set.enabled = v != 0;
        } else if (key == "people") {
            g_set.people = v != 0;
        } else if (key == "pets") {
            g_set.pets = v != 0;
        } else if (key == "hands") {
            g_set.hands = v != 0;
        } else if (key == "games_only") {
            g_set.games_only = v != 0;
        } else if (key == "autolaunch_set") {
            g_set.autolaunch_set = v != 0;
        } else if (key == "max_dist") {
            g_set.max_dist = v;
        } else if (key == "size" && std::find(std::begin(kSizes), std::end(kSizes), int(v)) != std::end(kSizes)) {
            g_set.size = int(v);
        } else if (key == "hz" && std::find(std::begin(kRates), std::end(kRates), v) != std::end(kRates)) {
            g_set.hz = v;
        }
    }
    fclose(f);
}

static void save_settings() {
    FILE* f = g_conf.empty() ? nullptr : fopen(g_conf.c_str(), "w");
    if (!f) {
        return;
    }
    fprintf(
        f,
        "enabled=%d\npeople=%d\npets=%d\nhands=%d\ngames_only=%d\nmax_dist=%.1f\nsize=%d\nhz=%g\nautolaunch_set=%d\n",
        g_set.enabled, g_set.people, g_set.pets, g_set.hands, g_set.games_only, g_set.max_dist, g_set.size, g_set.hz,
        g_set.autolaunch_set);
    fclose(f);
}
