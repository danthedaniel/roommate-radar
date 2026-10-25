// The "Guardian" dashboard tab: header with a status line, then one row per setting. Drawn on the CPU like the labels
// (draw.h); the dashboard's laser pointer arrives as mouse events in texture pixels.
#pragma once
#include <openvr.h>

#include "detector.h"
#include "draw.h"
#include "register.h"
#include "settings.h"
#include "textures.h"

namespace panel {

const int W = 1000, H = 1048, kPad = 48, kTop = 190, kRowH = 92;
enum Ctl {
    kEnabled,
    kAutoLaunch,  // SteamVR's auto-launch for this app, not one of g_set
    kGamesOnly,
    kPeople,
    kPets,
    kHands,
    kSize,  // dropdowns: open their popup
    kRate,
    kRangeMinus,
    kRangePlus,
    kItem  // the open popup's items: kItem + choice index
};
// one row per switch, then the dropdown rows and the range row
const int kToggles = kSize, kRangeRow = kRate + 1;
const char* kRows[] = {
    "Enabled",         "Start with SteamVR",   "Only in Games",  "Detect People", "Detect Cats and Dogs",
    "Ignore My Hands", "Detection Resolution", "Detection Rate", "Range"};
const char* kSizeLabels[] = {"Low (320px)", "Medium (480px)", "High (640px)"};
const char* kRateLabels[] = {"2Hz", "4Hz", "8Hz", "16Hz"};
const int kToggleW = 104, kToggleH = 56, kBtnR = 28, kDropW = 300, kDropH = 60, kItemH = 60;
const unsigned short kHoverTickUs = 1500;  // hover haptic
static int popup = -1;                     // the dropdown (kSize, kRate) whose popup is showing
static vr::VROverlayHandle_t dash;
static int hover = -1;
static std::string drawn;  // the status the tab was last drawn with ("" = redraw)

static float row_cy(int row) {
    return kTop + (row + 0.5f) * kRowH;
}

static bool autolaunch() {
    return !g_app_key.empty() && vr::VRApplications()->GetApplicationAutoLaunch(g_app_key.c_str());
}
// Greyed out: the detection settings while Enabled is off, Start with SteamVR when not started by Steam.
static bool dimmed(int row) {
    return row == kAutoLaunch ? g_app_key.empty() : row > kAutoLaunch && !g_set.enabled;
}

static int choices(int drop) {
    return drop == kSize ? 3 : 4;
}
static const char* label(int drop, int i) {
    return drop == kSize ? kSizeLabels[i] : kRateLabels[i];
}
static int choice(int drop) {  // 0 for a --size / --max-hz that isn't one of the choices
    int i = drop == kSize ? int(std::find(std::begin(kSizes), std::end(kSizes), g_set.size) - std::begin(kSizes))
                          : int(std::find(std::begin(kRates), std::end(kRates), g_set.hz) - std::begin(kRates));
    return i < choices(drop) ? i : 0;
}
static float drop_x0() {
    return W - kPad - kDropW;
}
// The popup opens with the current choice over the dropdown, moved up as needed to stay inside the panel.
static float popup_top(int drop) {
    return std::min(row_cy(drop) - (choice(drop) + 0.5f) * kItemH, H - choices(drop) * kItemH - 8.f);
}

static float minus_cx() {
    return W - kPad - kBtnR - 220.f;
}
static float plus_cx() {
    return W - kPad - kBtnR;
}

// Which control (texture pixel, y down) is under the pointer: the whole row for toggles, the buttons for range.
static int hit(float x, float y) {
    if (popup >= 0) {
        int i = int(std::floor((y - popup_top(popup)) / kItemH));
        return x >= drop_x0() && x <= drop_x0() + kDropW && i >= 0 && i < choices(popup) ? kItem + i : -1;
    }
    int row = int(std::floor((y - kTop) / kRowH));
    if (x < kPad || x > W - kPad || row < 0 || row > kRangeRow || dimmed(row)) {
        return -1;  // greyed-out rows don't hover or press
    }
    if (row < kRangeRow) {
        return row;
    }
    if (std::hypot(x - minus_cx(), y - row_cy(kRangeRow)) < kBtnR + 12) {
        return kRangeMinus;
    }
    if (std::hypot(x - plus_cx(), y - row_cy(kRangeRow)) < kBtnR + 12) {
        return kRangePlus;
    }
    return -1;
}

// While a popup is open any press closes it (-1: outside it); an item also picks that choice.
static void press(int ctl) {
    if (popup >= 0) {
        int drop = popup, i = ctl - kItem;
        popup = -1;
        if (i < 0) {
            return;
        }
        if (drop == kSize) {
            g_set.size = g_det_size = kSizes[i];  // the detector reloads at the new size
        } else {
            g_set.hz = g_det_hz = kRates[i];
        }
        save_settings();
        return;
    }
    switch (ctl) {
    case kEnabled:
        g_set.enabled = !g_set.enabled;
        break;
    case kAutoLaunch:
        vr::VRApplications()->SetApplicationAutoLaunch(g_app_key.c_str(), !autolaunch());
        return;
    case kPeople:
        g_set.people = !g_set.people;
        break;
    case kPets:
        g_set.pets = !g_set.pets;
        break;
    case kHands:
        g_set.hands = !g_set.hands;
        break;
    case kGamesOnly:
        g_set.games_only = !g_set.games_only;
        break;
    case kRangeMinus:
        g_set.max_dist = std::max(1.0f, g_set.max_dist - 0.5f);
        break;
    case kRangePlus:
        g_set.max_dist = std::min(8.0f, g_set.max_dist + 0.5f);
        break;
    case kSize:
    case kRate:
        popup = ctl;
        return;
    default:
        return;
    }
    save_settings();
}

static void fill_rrect(std::vector<uint8_t>& px, float x0, float y0, float w, float h, float r, const uint8_t rgb[3],
                       float a) {
    for (int y = std::max(0, int(y0) - 1); y < std::min(H, int(y0 + h) + 2); y++) {
        for (int x = std::max(0, int(x0) - 1); x < std::min(W, int(x0 + w) + 2); x++) {
            over(&px[(size_t(y) * W + x) * 4], rgb,
                 a * std::clamp(0.5f - rrect_sd(x + 0.5f - x0, y + 0.5f - y0, w, h, 0, r), 0.f, 1.f));
        }
    }
}

static void fill_circle(std::vector<uint8_t>& px, float cx, float cy, float r, const uint8_t rgb[3], float a) {
    fill_rrect(px, cx - r, cy - r, 2 * r, 2 * r, r, rgb, a);
}

static std::vector<uint8_t> render(int hover, const std::string& status) {
    static const uint8_t bg[3] = {20, 20, 24}, white[3] = {245, 245, 247}, grey[3] = {150, 150, 160},
                         off[3] = {68, 68, 76}, btn[3] = {52, 52, 60};
    uint8_t accent[3];
    class_rgb(0, accent);
    std::vector<uint8_t> px(size_t(W) * H * 4);
    fill_rrect(px, 0, 0, W, H, 32, bg, 0.94f);
    draw_text(&px, W, H, kPad, 100, "Guardian", white, 56);
    draw_text(&px, W, H, kPad, 150, status.c_str(), grey, 30);
    bool on[kToggles] = {g_set.enabled, autolaunch(), g_set.games_only, g_set.people, g_set.pets, g_set.hands};
    for (int row = 0; row <= kRangeRow; row++) {
        float cy = row_cy(row);
        if (hover == row) {
            fill_rrect(px, kPad - 20, cy - kRowH / 2.f + 4, W - 2 * kPad + 40, kRowH - 8, 18, white, 0.05f);
        }
        if (row) {
            fill_rrect(px, kPad, cy - kRowH / 2.f, W - 2 * kPad, 1.5f, 0, white, 0.08f);
        }
        bool dim = dimmed(row);
        if (row == kHands) {  // title nudged up, detail line below it
            draw_text(&px, W, H, kPad, int(cy + 1), kRows[row], dim ? grey : white, 36);
            draw_text(&px, W, H, kPad, int(cy + 33), "Works best when holding controllers", grey, 24);
        } else {
            draw_text(&px, W, H, kPad, int(cy + 13), kRows[row], dim ? grey : white, 36);
        }
        if (row < kToggles) {  // switch: accent track with the knob right when on; grey when dimmed
            float x0 = W - kPad - kToggleW, y0 = cy - kToggleH / 2.f;
            fill_rrect(px, x0, y0, kToggleW, kToggleH, kToggleH / 2.f, on[row] && !dim ? accent : off, dim ? 0.5f : 1);
            fill_circle(px, on[row] ? x0 + kToggleW - kToggleH / 2.f : x0 + kToggleH / 2.f, cy, kToggleH / 2.f - 5,
                        white, dim ? 0.6f : 1);
            continue;
        }
        if (row < kRangeRow) {  // dropdown: current choice and a chevron
            fill_rrect(px, drop_x0(), cy - kDropH / 2.f, kDropW, kDropH, 14, btn,
                       (hover == row ? 1 : 0.8f) * (dim ? 0.5f : 1));
            draw_text(&px, W, H, int(drop_x0() + 22), int(cy + 10), label(row, choice(row)), dim ? grey : white, 28);
            for (int k = -8; k <= 8; k++) {
                fill_circle(px, W - kPad - 34 + k, cy - 3 + (8 - std::abs(k)) * 0.6f, 2.f, dim ? grey : white, 1);
            }
            continue;
        }
        for (int b = 0; b < 2; b++) {  // range: (-) value (+)
            float cx = b ? plus_cx() : minus_cx();
            fill_circle(px, cx, cy, kBtnR, btn, (hover == kRangeMinus + b ? 1 : 0.8f) * (dim ? 0.5f : 1));
            fill_rrect(px, cx - 11, cy - 1.5f, 22, 3, 1.5f, dim ? grey : white, 1);
            if (b) {
                fill_rrect(px, cx - 1.5f, cy - 11, 3, 22, 1.5f, dim ? grey : white, 1);
            }
        }
        char v[16];
        snprintf(v, sizeof v, "%.1f m", g_set.max_dist);
        int tw = draw_text(nullptr, 0, 0, 0, 0, v, white, 36);
        draw_text(&px, W, H, int((minus_cx() + plus_cx()) / 2 - tw / 2.f), int(cy + 13), v, dim ? grey : white, 36);
    }
    if (popup >= 0) {  // over the rows: the choices, the current one in the accent color
        float y0 = popup_top(popup);
        int n = choices(popup);
        fill_rrect(px, drop_x0() - 2, y0 - 2, kDropW + 4, n * kItemH + 4, 16, white, 0.15f);
        fill_rrect(px, drop_x0(), y0, kDropW, n * kItemH, 14, btn, 1);
        for (int i = 0; i < n; i++) {
            float iy = y0 + i * kItemH;
            if (hover == kItem + i) {
                fill_rrect(px, drop_x0() + 6, iy + 4, kDropW - 12, kItemH - 8, 10, white, 0.1f);
            }
            draw_text(&px, W, H, int(drop_x0() + 22), int(iy + kItemH / 2.f + 10), label(popup, i),
                      i == choice(popup) ? accent : white, 28);
        }
    }
    return px;
}

// Tab icon: the box's corner brackets on a dark tile.
static std::vector<uint8_t> icon(int& size) {
    size = 128;
    std::vector<uint8_t> px(size_t(size) * size * 4), box = outline(80, 96, 0, 5);
    static const uint8_t bg[3] = {20, 20, 24};
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            uint8_t* p = &px[(size_t(y) * size + x) * 4];
            over(p, bg, std::clamp(0.5f - rrect_sd(x + 0.5f, y + 0.5f, size, size, 0, 24), 0.f, 1.f));
            int bx = x - 24, by = y - 16;
            if (bx >= 0 && by >= 0 && bx < 80 && by < 96) {
                over(p, &box[(size_t(by) * 80 + bx) * 4], box[(size_t(by) * 80 + bx) * 4 + 3] / 255.f);
            }
        }
    }
    return px;
}

static bool create(Textures& textures) {
    vr::VROverlayHandle_t dash_icon;
    if (vr::VROverlay()->CreateDashboardOverlay("steamframe.guardian.dashboard", "Guardian", &dash, &dash_icon)) {
        fprintf(stderr, "CreateDashboardOverlay failed\n");
        return false;
    }
    int size;
    std::vector<uint8_t> px = icon(size);
    textures.set(dash_icon, px, size, size);
    vr::VROverlay()->SetOverlayWidthInMeters(dash, 1.5f);
    vr::VROverlay()->SetOverlayInputMethod(dash, vr::VROverlayInputMethod_Mouse);
    vr::HmdVector2_t scale{float(W), float(H)};
    vr::VROverlay()->SetOverlayMouseScale(dash, &scale);
    px = render(-1, "Starting");
    textures.set(dash, px, W, H);
    return true;
}

// The pointer's events since the last call: hover and presses.
static void poll() {
    vr::VREvent_t ev;
    while (vr::VROverlay()->PollNextOverlayEvent(dash, &ev, sizeof ev)) {
        // Mouse y is up from the bottom of the texture.
        int c = hit(ev.data.mouse.x, H - ev.data.mouse.y);
        if (ev.eventType == vr::VREvent_MouseMove && c != hover) {
            hover = c, drawn.clear();
            if (c >= 0) {  // the tick the Steam UI gives when the pointer reaches a new item
                vr::VRSystem()->TriggerHapticPulse(ev.trackedDeviceIndex, 0, kHoverTickUs);
            }
        }
        if (ev.eventType == vr::VREvent_MouseButtonDown && (c >= 0 || popup >= 0)) {  // outside closes it
            press(c), drawn.clear();
        }
    }
}

// Redraws the tab, while it's showing, when the status or anything on it changed.
static void update(Textures& textures, const std::string& status) {
    if (vr::VROverlay()->IsOverlayVisible(dash) && drawn != status) {
        drawn = status;
        std::vector<uint8_t> px = render(hover, status);
        textures.set(dash, px, W, H);
    }
}

}  // namespace panel
