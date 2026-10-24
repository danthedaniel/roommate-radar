// guardian: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
// without passthrough. Stereo YOLO detector in detector.h, at a low size and rate to leave the GPU to the game.
//
// Each confirmed track is a box overlay (rounded outline) plus a label overlay (class, confidence): upright sprites
// turned to face you every compositor frame, in the standing universe. Detections are matched into tracks by a
// ByteTrack-style tracker (tracker.h); each box's center and size follow a cubic spline (time-based Catmull-Rom)
// through its track's last detections, played back one detection period behind the newest one so there is always
// a segment to be in.
//
// A "Guardian" tab in the SteamVR dashboard toggles it and what it shows; settings persist in ~/.config/guardian.conf.
//
// Usage: guardian [--max-dist 3] [--size 320] [--max-hz 4] [detector options, detector.h]
//        guardian --no-detector   (only log the VR app's frame stats, as a baseline)
//        guardian --register   (add guardian.vrmanifest to SteamVR, auto-launch with SteamVR, start it now)
//        guardian --selftest
#include <limits.h>
#include <signal.h>
#include <stdlib.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <vulkan/vulkan.h>
#include <openvr.h>

#include <numeric>
#include <thread>
#include <unordered_map>

#include "detector.h"
#include "tracker.h"

static const char* kAppKey = "steamframe.guardian";
static const int kMaxOverlays = 8;
static bool g_detector = true;       // --no-detector: just log frame stats (baseline for the detector's cost)
static const unsigned short kHoverTickUs = 1500;  // dashboard hover haptic
static const float kLabelH = 0.07f;  // label height (m), just above the box
static const char* kFont = "/usr/share/fonts/noto/NotoSans-Medium.ttf";

// Set from the dashboard tab, saved to g_conf.
struct Settings {
    bool enabled = true, people = true, pets = true;  // pets: cats and dogs
    bool hands = true;                                // ignore boxes around a controller: your hands (holds_controller)
    bool games_only = false;                          // pause while no game is running (SteamVR home, dashboard)
    float max_dist = 3;                               // ignore detections farther than this (m, horizontal)
    int size = 320;                                   // YOLO per-view size, one of kSizes
    float hz = 4;                                     // YOLO runs per second, one of kRates
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
    fprintf(f, "enabled=%d\npeople=%d\npets=%d\nhands=%d\ngames_only=%d\nmax_dist=%.1f\nsize=%d\nhz=%g\n",
            g_set.enabled, g_set.people, g_set.pets, g_set.hands, g_set.games_only, g_set.max_dist, g_set.size, g_set.hz);
    fclose(f);
}

// person, cat, dog
static bool wanted(int cls) {
    return cls == 0 ? g_set.people : (cls == 15 || cls == 16) && g_set.pets;
}

static XrVector3f v_add(XrVector3f a, XrVector3f b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
static XrVector3f v_sub(XrVector3f a, XrVector3f b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
static XrVector3f v_mul(XrVector3f a, float s) {
    return {a.x * s, a.y * s, a.z * s};
}

// Your own arms and hands (YOLO often calls them "person"; it doesn't know the controller, so an arm box can stop at
// the wrist and leave it out): a point on the forearm, from 40% of the way from the shoulder to a controller up to the
// controller, inside the box: within its rectangle scaled to that point's depth (seen from the camera) and grown by
// 15 cm, and from 40 cm behind its depth to 80 cm in front of it (arm boxes measure too far: cut off by the image edge
// differently in each view, or merged with what's behind the arm). Box and controllers are in the head frame at
// capture time.
static bool holds_controller(const Obj& o, const std::vector<XrVector3f>& controllers) {
    const float m = 0.15f;
    const XrVector3f c = o.origin;
    const XrVector3f* b = o.corners;
    for (const XrVector3f& hand : controllers) {
        const XrVector3f shoulder{std::copysign(0.18f, hand.x), -0.25f, 0};  // ponytail: one body size for everyone
        for (int k = 4; k <= 10; k++) {
            XrVector3f p = v_add(shoulder, v_mul(v_sub(hand, shoulder), k / 10.f));
            float s = (p.z - c.z) / (b[0].z - c.z);
            if (s > 0 && p.x > c.x + (b[0].x - c.x) * s - m && p.x < c.x + (b[1].x - c.x) * s + m &&
                p.y > c.y + (b[3].y - c.y) * s - m && p.y < c.y + (b[0].y - c.y) * s + m &&
                p.z - b[0].z > -0.4f && p.z - b[0].z < 0.8f) {
                return true;
            }
        }
    }
    return false;
}

// Your own arm or lap, controller or not: the box reaches the bottom 10% of the view, is within 1.2 m, and its top is
// below eye level (anyone else that close shows their head). Head frame, like holds_controller.
static bool own_arm(const Obj& o) {
    const XrVector3f* b = o.corners;
    float Z = o.origin.z - b[3].z, edge = std::tan(g_det.hfov_deg * float(M_PI) / 360);
    return Z > 0 && Z < 1.2f && (o.origin.y - b[3].y) / Z > 0.8f * edge && b[0].y < o.origin.y;
}

// A box (center p, width w, height h; standing universe) as seen from the head position: azimuth/elevation (rad) of
// its edges, so duplicate() can tell two tracks of one person apart from two people.
static Det angular_box(const float* p, float w, float h, XrVector3f head) {
    float d = std::hypot(p[0] - head.x, p[2] - head.z), az = std::atan2(p[0] - head.x, head.z - p[2]),
          el = std::atan2(p[1] - head.y, d), hw = std::atan2(w / 2, d), hh = std::atan2(h / 2, d);
    return {az - hw, el - hh, az + hw, el + hh, 0, 0};
}

static float v_len(XrVector3f a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
static XrVector3f v_center(const XrVector3f* c) {
    return v_mul(v_add(v_add(c[0], c[1]), v_add(c[2], c[3])), 0.25f);
}

static XrVector3f apply(const vr::HmdMatrix34_t& m, XrVector3f p) {
    return {m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3],
            m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3],
            m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3]};
}

// Upright quad at c facing the head position h (overlay +z points at the viewer).
static vr::HmdMatrix34_t facing(XrVector3f c, XrVector3f h) {
    float zx = h.x - c.x, zz = h.z - c.z, n = std::hypot(zx, zz);
    if (n < 1e-4f) {
        zx = 0, zz = 1, n = 1;
    }
    zx /= n, zz /= n;
    return {{{zz, 0, zx, c.x}, {0, 1, 0, c.y}, {-zx, 0, zz, c.z}}};
}

static void class_rgb(int cls, uint8_t rgb[3]) {
    static const uint8_t person[3] = {255, 69, 58}, pet[3] = {255, 184, 0};
    memcpy(rgb, cls == 0 ? person : pet, 3);
}

// Straight-alpha "over" of color rgb at coverage a onto RGBA pixel p.
static void over(uint8_t* p, const uint8_t rgb[3], float a) {
    if (a <= 0) {
        return;
    }
    float da = p[3] / 255.f, oa = a + da * (1 - a);
    for (int c = 0; c < 3; c++) {
        p[c] = uint8_t(std::lround((rgb[c] * a + p[c] * da * (1 - a)) / oa));
    }
    p[3] = uint8_t(std::lround(oa * 255));
}

// Signed distance (px, < 0 inside) from (x, y) to a w x h rounded rect inset by `inset`, corner radius r.
static float rrect_sd(float x, float y, float w, float h, float inset, float r) {
    float qx = std::fabs(x - w / 2.f) - (w / 2.f - inset - r), qy = std::fabs(y - h / 2.f) - (h / 2.f - inset - r);
    return std::hypot(std::max(qx, 0.f), std::max(qy, 0.f)) + std::min(std::max(qx, qy), 0.f) - r;
}

// Anti-aliased rounded outline, `line` px wide (thinner and fainter along the sides, bold at the corners), over a
// barely-there fill. Sized to the box's aspect.
static std::vector<uint8_t> outline(int w, int h, int cls, float line) {
    uint8_t rgb[3];
    class_rgb(cls, rgb);
    std::vector<uint8_t> px(size_t(w) * h * 4);
    const float m = float(std::min(w, h)), r = m * 0.08f, corner = m * 0.2f;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float fx = x + 0.5f, fy = y + 0.5f, sd = rrect_sd(fx, fy, w, h, 1, r);
            bool c = std::fabs(fx - w / 2.f) > w / 2.f - corner && std::fabs(fy - h / 2.f) > h / 2.f - corner;
            float lw = c ? line * 2.5f : line;  // stroke lies just inside the edge: sd in [-lw, 0]
            float a = std::clamp(lw / 2 + 0.5f - std::fabs(sd + lw / 2), 0.f, 1.f) * (c ? 1 : 0.6f);
            over(&px[(size_t(y) * w + x) * 4], rgb, std::max(a, sd < 0 ? 0.06f : 0.f));
        }
    }
    return px;
}

static FT_Face font() {
    static FT_Library lib;
    static FT_Face face;
    if (!face) {
        if (FT_Init_FreeType(&lib) || FT_New_Face(lib, kFont, 0, &face)) {
            throw std::runtime_error(std::string("can't load ") + kFont);
        }
    }
    return face;
}

// Draws (or with px == nullptr, measures) UTF-8 text `size` px tall with its baseline at y; returns the pen x after it.
static int draw_text(std::vector<uint8_t>* px, int w, int h, int x, int y, const char* s, const uint8_t rgb[3],
                     int size) {
    FT_Face f = font();
    FT_Set_Pixel_Sizes(f, 0, size);
    for (; *s; s++) {
        uint32_t cp = (unsigned char)*s;
        if (cp >= 0xE0 && s[1] && s[2]) {
            cp = (cp & 0x0F) << 12 | (s[1] & 0x3F) << 6 | (s[2] & 0x3F), s += 2;
        } else if (cp >= 0xC0 && s[1]) {
            cp = (cp & 0x1F) << 6 | (s[1] & 0x3F), s += 1;
        }
        if (FT_Load_Char(f, cp, px ? FT_LOAD_RENDER : FT_LOAD_DEFAULT)) {
            continue;
        }
        const FT_GlyphSlot g = f->glyph;
        for (int r = 0; px && r < int(g->bitmap.rows); r++) {
            for (int c = 0; c < int(g->bitmap.width); c++) {
                int X = x + g->bitmap_left + c, Y = y - g->bitmap_top + r;
                if (X >= 0 && Y >= 0 && X < w && Y < h) {
                    over(&(*px)[(size_t(Y) * w + X) * 4], rgb, g->bitmap.buffer[r * g->bitmap.pitch + c] / 255.f);
                }
            }
        }
        x += int(g->advance.x >> 6);
    }
    return x;
}

// Dark translucent pill: class-colored dot, class name in white, confidence in the class color.
static std::vector<uint8_t> label(const char* name, int pct, int cls, int& w, int& h) {
    uint8_t rgb[3], white[3] = {245, 245, 247}, bg[3] = {16, 16, 20};
    class_rgb(cls, rgb);
    char num[16];
    snprintf(num, sizeof num, "%d%%", pct);
    h = 64;
    const int dot = 14, gap = 12, space = 12, pad = h / 2 - 6;
    int x_name = pad + dot + gap, x_num = draw_text(nullptr, 0, 0, x_name, 0, name, white, 40) + space;
    w = draw_text(nullptr, 0, 0, x_num, 0, num, rgb, 40) + pad;
    std::vector<uint8_t> px(size_t(w) * h * 4);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t* p = &px[(size_t(y) * w + x) * 4];
            over(p, bg, 0.72f * std::clamp(0.5f - rrect_sd(x + 0.5f, y + 0.5f, w, h, 0, h / 2.f), 0.f, 1.f));
            over(p, rgb,
                 std::clamp(dot / 2.f + 0.5f - std::hypot(x + 0.5f - (pad + dot / 2.f), y + 0.5f - h / 2.f), 0.f, 1.f));
        }
    }
    FT_Face f = font();
    int base = h / 2 + int((f->size->metrics.ascender + f->size->metrics.descender) >> 7);  // vertically centered
    draw_text(&px, w, h, x_name, base, name, white, 40);
    draw_text(&px, w, h, x_num, base, num, rgb, 40);
    return px;
}

// ---------------------------------------------------------------------------------------------
// Dashboard tab: header with a status line, then one row per setting. Drawn on the CPU like the labels; the
// dashboard's laser pointer arrives as mouse events in texture pixels.

namespace panel {

const int W = 1000, H = 956, kPad = 48, kTop = 190, kRowH = 92;
enum Ctl {
    kEnabled,
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
const char* kRows[] = {"Enabled",         "Only in Games",        "Detect People",  "Detect Cats and Dogs",
                       "Ignore My Hands", "Detection Resolution", "Detection Rate", "Range"};
const char* kSizeLabels[] = {"Low (320px)", "Medium (480px)", "High (640px)"};
const char* kRateLabels[] = {"2Hz", "4Hz", "8Hz", "16Hz"};
const int kToggleW = 104, kToggleH = 56, kBtnR = 28, kDropW = 300, kDropH = 60, kItemH = 60;
static int popup = -1;  // the dropdown (kSize, kRate) whose popup is showing

static float row_cy(int row) {
    return kTop + (row + 0.5f) * kRowH;
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
    if (x < kPad || x > W - kPad || row < 0 || row > kRangeRow || (row > kEnabled && !g_set.enabled)) {
        return -1;  // greyed-out rows (Enabled off) don't hover or press
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
    bool on[kToggles] = {g_set.enabled, g_set.games_only, g_set.people, g_set.pets, g_set.hands};
    for (int row = 0; row <= kRangeRow; row++) {
        float cy = row_cy(row);
        if (hover == row) {
            fill_rrect(px, kPad - 20, cy - kRowH / 2.f + 4, W - 2 * kPad + 40, kRowH - 8, 18, white, 0.05f);
        }
        if (row) {
            fill_rrect(px, kPad, cy - kRowH / 2.f, W - 2 * kPad, 1.5f, 0, white, 0.08f);
        }
        bool dim = row > 0 && !g_set.enabled;  // the other settings don't matter while it's off
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

}  // namespace panel

// Cubic Hermite through the samples with Catmull-Rom tangents scaled by the real sample spacing (one-sided at the
// ends), so uneven detector timing doesn't kink the path. Clamps to the first/last sample outside their span.
static track::Sample spline_at(const std::deque<track::Sample>& s, double t) {
    if (t <= s.front().t) {
        return s.front();
    }
    if (t >= s.back().t) {
        return s.back();
    }
    size_t i = 0;
    while (s[i + 1].t <= t) {
        i++;
    }
    auto tangent = [&](size_t k, int c) {  // per second
        size_t a = k ? k - 1 : k, b = std::min(k + 1, s.size() - 1);
        return (s[b].v[c] - s[a].v[c]) / float(s[b].t - s[a].t);
    };
    float dt = float(s[i + 1].t - s[i].t), u = float(t - s[i].t) / dt, u2 = u * u, u3 = u2 * u;
    track::Sample r{t, {}};
    for (int c = 0; c < 5; c++) {
        r.v[c] = (2 * u3 - 3 * u2 + 1) * s[i].v[c] + (u3 - 2 * u2 + u) * dt * tangent(i, c) +
                 (-2 * u3 + 3 * u2) * s[i + 1].v[c] + (u3 - u2) * dt * tangent(i + 1, c);
    }
    return r;
}

// Seconds from `then` to `now` (CLOCK_MONOTONIC ns), signed: negating an unsigned difference wraps around.
static float seconds(uint64_t now, uint64_t then) {
    return float(int64_t(now - then)) / 1e9f;
}

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

// Detections this frame that guardian cares about, in the standing universe (head pose at capture): stereo (has
// depth), a wanted class, not around a controller (`ctrl`, standing universe, at capture), within range.
static std::vector<track::Meas> measurements(const std::vector<Obj>& objs, const vr::HmdMatrix34_t& head,
                                             const std::vector<XrVector3f>& ctrl) {
    XrVector3f h{head.m[0][3], head.m[1][3], head.m[2][3]};
    std::vector<XrVector3f> ctrl_head;  // head_from_world = R^T (p - t)
    for (XrVector3f p : g_set.hands ? ctrl : std::vector<XrVector3f>{}) {
        XrVector3f d = v_sub(p, h);
        ctrl_head.push_back({head.m[0][0] * d.x + head.m[1][0] * d.y + head.m[2][0] * d.z,
                             head.m[0][1] * d.x + head.m[1][1] * d.y + head.m[2][1] * d.z,
                             head.m[0][2] * d.x + head.m[1][2] * d.y + head.m[2][2] * d.z});
    }
    std::vector<track::Meas> ms;
    for (const Obj& o : objs) {
        if (!o.stereo || !wanted(o.cls) || holds_controller(o, ctrl_head) || (g_set.hands && own_arm(o))) {
            continue;
        }
        XrVector3f w[4];
        for (int k = 0; k < 4; k++) {
            w[k] = apply(head, o.corners[k]);
        }
        XrVector3f c = v_center(w);
        float dist = std::hypot(c.x - h.x, c.z - h.z);
        if (dist > g_set.max_dist || (o.cls == 0 && dist < 0.5f)) {  // a "person" closer than 50 cm is you
            continue;
        }
        // ponytail: isotropic position noise growing like stereo depth error (~Z^2); per-axis noise along the view
        // ray would track lateral motion tighter
        float sigma = 0.05f + 0.02f * o.dist * o.dist;
        ms.push_back({{c.x, c.y, c.z, v_len(v_sub(w[1], w[0])), v_len(v_sub(w[3], w[0]))}, sigma, o.score, o.cls});
    }
    return ms;
}

// ---------------------------------------------------------------------------------------------
// Overlay textures. SetOverlayRaw makes the compositor re-create the overlay's texture on every call, and the overlay
// flickers while it does (the dashboard panel on every hover change). So pixels go into our own Vulkan images,
// handed over with SetOverlayTexture; each overlay has two and alternates, so we never write the one on screen.

class Textures {
  public:
    // Vulkan on the device the compositor renders with; false: use SetOverlayRaw.
    bool init() {
        std::vector<std::string> iext = split(req(true, nullptr));
        std::vector<const char*> ip;
        for (auto& e : iext) {
            ip.push_back(e.c_str());
        }
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "guardian";
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ici.pApplicationInfo = &app;
        ici.enabledExtensionCount = uint32_t(ip.size());
        ici.ppEnabledExtensionNames = ip.data();
        if (vkCreateInstance(&ici, nullptr, &inst_)) {
            return false;
        }
        uint64_t out = 0;
        vr::VRSystem()->GetOutputDevice(&out, vr::TextureType_Vulkan, inst_);
        phys_ = VkPhysicalDevice(out);
        if (!phys_) {
            return false;
        }

        uint32_t n = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &n, nullptr);
        std::vector<VkQueueFamilyProperties> qf(n);
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &n, qf.data());
        for (qfam_ = 0; qfam_ < n && !(qf[qfam_].queueFlags & VK_QUEUE_GRAPHICS_BIT); qfam_++) {
        }
        if (qfam_ == n) {
            return false;
        }
        std::vector<std::string> dext = split(req(false, phys_));
        std::vector<const char*> dp;
        for (auto& e : dext) {
            dp.push_back(e.c_str());
        }
        float prio = 1;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = qfam_;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = uint32_t(dp.size());
        dci.ppEnabledExtensionNames = dp.data();
        if (vkCreateDevice(phys_, &dci, nullptr, &dev_)) {
            return false;
        }
        vkGetDeviceQueue(dev_, qfam_, 0, &queue_);
        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pci.queueFamilyIndex = qfam_;
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        return !vkCreateCommandPool(dev_, &pci, nullptr, &pool_) && (cai.commandPool = pool_, true) &&
               !vkAllocateCommandBuffers(dev_, &cai, &cmd_) && !vkCreateFence(dev_, &fci, nullptr, &fence_);
    }

    // Shows w x h RGBA pixels on overlay h (Vulkan if init() succeeded, else SetOverlayRaw).
    void set(vr::VROverlayHandle_t ov, const std::vector<uint8_t>& px, int w, int h) {
        if (!dev_) {
            vr::VROverlay()->SetOverlayRaw(ov, (void*)px.data(), w, h, 4);
            return;
        }
        Slot& s = slots_[ov];
        Image& im = s.img[s.next];
        s.next ^= 1;
        if (im.w != w || im.h != h) {
            destroy(im);
            if (!create(im, w, h)) {
                vr::VROverlay()->SetOverlayRaw(ov, (void*)px.data(), w, h, 4);
                return;
            }
        }
        if (px.size() > staging_size_ && !grow_staging(px.size())) {
            return;
        }
        memcpy(staging_map_, px.data(), px.size());

        vkResetCommandBuffer(cmd_, 0);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd_, &bi);
        barrier(im.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0,
                VK_ACCESS_TRANSFER_WRITE_BIT);
        VkBufferImageCopy copy{};
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.imageExtent = {uint32_t(w), uint32_t(h), 1};
        vkCmdCopyBufferToImage(cmd_, staging_, im.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
        // The layout OpenVR expects Vulkan textures in.
        barrier(im.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        vkEndCommandBuffer(cmd_);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd_;
        vkResetFences(dev_, 1, &fence_);
        vkQueueSubmit(queue_, 1, &si, fence_);
        vkWaitForFences(dev_, 1, &fence_, VK_TRUE, UINT64_MAX);

        vr::VRVulkanTextureData_t vk{
            uint64_t(im.image),   dev_, phys_, inst_, queue_, qfam_, uint32_t(w), uint32_t(h), VK_FORMAT_R8G8B8A8_UNORM,
            VK_SAMPLE_COUNT_1_BIT};
        vr::Texture_t tex{&vk, vr::TextureType_Vulkan, vr::ColorSpace_Auto};  // 8-bit UNORM = gamma, like Raw
        vr::VROverlay()->SetOverlayTexture(ov, &tex);
    }

  private:
    struct Image {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        int w = 0, h = 0;
    };
    struct Slot {
        Image img[2];
        int next = 0;
    };
    std::unordered_map<vr::VROverlayHandle_t, Slot> slots_;
    VkInstance inst_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice dev_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t qfam_ = 0;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory staging_mem_ = VK_NULL_HANDLE;
    void* staging_map_ = nullptr;
    size_t staging_size_ = 0;

    static std::string req(bool instance, VkPhysicalDevice phys) {
        char buf[4096] = "";
        if (instance) {
            vr::VRCompositor()->GetVulkanInstanceExtensionsRequired(buf, sizeof buf);
        } else {
            vr::VRCompositor()->GetVulkanDeviceExtensionsRequired(phys, buf, sizeof buf);
        }
        return buf;
    }
    static std::vector<std::string> split(const std::string& s) {
        std::vector<std::string> out;
        for (size_t i = 0, j; i < s.size(); i = j + 1) {
            j = s.find(' ', i);
            if (j == std::string::npos) {
                j = s.size();
            }
            if (j > i) {
                out.push_back(s.substr(i, j - i));
            }
        }
        return out;
    }
    uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags want) {
        VkPhysicalDeviceMemoryProperties mp;
        vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
        for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
            if ((bits >> i & 1) && (mp.memoryTypes[i].propertyFlags & want) == want) {
                return i;
            }
        }
        return UINT32_MAX;
    }
    bool create(Image& im, int w, int h) {
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = VK_FORMAT_R8G8B8A8_UNORM;
        ci.extent = {uint32_t(w), uint32_t(h), 1};
        ci.mipLevels = ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        if (vkCreateImage(dev_, &ci, nullptr, &im.image)) {
            return false;
        }
        VkMemoryRequirements mr;
        vkGetImageMemoryRequirements(dev_, im.image, &mr);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = mr.size;
        ai.memoryTypeIndex = memory_type(mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (vkAllocateMemory(dev_, &ai, nullptr, &im.mem) || vkBindImageMemory(dev_, im.image, im.mem, 0)) {
            return false;
        }
        im.w = w, im.h = h;
        return true;
    }
    void destroy(Image& im) {
        if (im.image) {
            vkDestroyImage(dev_, im.image, nullptr);
        }
        if (im.mem) {
            vkFreeMemory(dev_, im.mem, nullptr);
        }
        im = {};
    }
    bool grow_staging(size_t size) {
        if (staging_) {
            vkDestroyBuffer(dev_, staging_, nullptr), vkFreeMemory(dev_, staging_mem_, nullptr);
        }
        // Cleared so a failed allocation below can't leave the freed handles to be freed again next time.
        staging_ = VK_NULL_HANDLE, staging_mem_ = VK_NULL_HANDLE, staging_size_ = 0;
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = size;
        bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(dev_, &bci, nullptr, &staging_)) {
            return false;
        }
        VkMemoryRequirements mr;
        vkGetBufferMemoryRequirements(dev_, staging_, &mr);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = mr.size;
        ai.memoryTypeIndex =
            memory_type(mr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (vkAllocateMemory(dev_, &ai, nullptr, &staging_mem_) ||
            vkBindBufferMemory(dev_, staging_, staging_mem_, 0) ||
            vkMapMemory(dev_, staging_mem_, 0, VK_WHOLE_SIZE, 0, &staging_map_)) {
            return false;
        }
        staging_size_ = size;
        return true;
    }
    void barrier(VkImage image, VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.oldLayout = from, b.newLayout = to, b.srcAccessMask = src, b.dstAccessMask = dst;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
    }
};

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

static int register_app() {
    // VR_Init would start SteamVR outside its systemd service, which then can't come up until that one is killed.
    if (system("pgrep -x vrserver > /dev/null")) {
        fprintf(stderr, "SteamVR isn't running\n");
        return 1;
    }
    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Utility);
    if (err) {
        fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err));
        return 1;
    }
    char path[PATH_MAX];
    if (!realpath("guardian.vrmanifest", path)) {
        perror("guardian.vrmanifest");
        return 1;
    }
    auto* apps = vr::VRApplications();
    vr::EVRApplicationError e = apps->AddApplicationManifest(path, false);
    printf("AddApplicationManifest(%s): %s\n", path, apps->GetApplicationsErrorNameFromEnum(e));
    e = apps->SetApplicationAutoLaunch(kAppKey, true);
    printf("SetApplicationAutoLaunch: %s\n", apps->GetApplicationsErrorNameFromEnum(e));
    if (!apps->GetApplicationProcessId(kAppKey)) {
        e = apps->LaunchApplication(kAppKey);
        printf("LaunchApplication: %s\n", apps->GetApplicationsErrorNameFromEnum(e));
    }
    vr::VR_Shutdown();
    return e ? 1 : 0;
}

static int selftest() {
    int fails = 0;
    auto check = [&](bool ok, const char* what) {
        printf("%s %s\n", ok ? "ok  " : "FAIL", what);
        fails += !ok;
    };
    // Box 2 m ahead of an identity head: a quad facing the head has +z toward it, and x right.
    vr::HmdMatrix34_t m = facing({0, 1, -2}, {0, 1.6f, 0});
    check(std::fabs(m.m[2][2] - 1) < 1e-5f && std::fabs(m.m[0][0] - 1) < 1e-5f, "quad ahead faces the head");
    m = facing({2, 1, 0}, {0, 1.6f, 0});  // box to the right: +z points to -x
    check(std::fabs(m.m[0][2] + 1) < 1e-5f && std::fabs(m.m[1][1] - 1) < 1e-5f, "quad to the right faces the head");

    // Spline: passes through the samples, exact for steady motion even with uneven spacing, clamps at the ends.
    std::deque<track::Sample> s;
    for (double t : {0.0, 0.25, 0.45, 0.75}) {
        s.push_back({t, {float(t), 1, -2, 0.5f, 1.7f}});  // 1 m/s along x
    }
    check(std::fabs(spline_at(s, 0.45).v[0] - 0.45f) < 1e-5f, "spline passes through a sample");
    float worst = 0;
    for (int ms = 0; ms <= 750; ms += 10) {
        worst = std::max(worst, std::fabs(spline_at(s, ms / 1000.0).v[0] - ms / 1000.f));
    }
    check(worst < 1e-4f, "spline is exact for steady motion");
    check(spline_at(s, 0.9).v[0] == 0.75f && spline_at(s, 0.9).v[4] == 1.7f, "spline clamps after the end");

    fails += track::selftest([](bool ok, const char* what) {
        printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    });

    check(-seconds(3000000000ull, 2950000000ull) == -0.05f && seconds(1, 2) < 0, "signed seconds between timestamps");

    // Label: pill sized to the text (transparent outside its rounded ends), white text, colored dot.
    int lw, lh;
    std::vector<uint8_t> px = label("person", 87, 0, lw, lh);
    auto at = [&](const std::vector<uint8_t>& v, int w, int x, int y) {
        return &v[(size_t(y) * w + x) * 4];
    };
    size_t white = 0;
    for (size_t i = 0; i < px.size(); i += 4) {
        white += px[i] > 230 && px[i + 1] > 230 && px[i + 3] > 230;
    }
    check(lh == 64 && lw > 3 * lh && at(px, lw, 0, 0)[3] == 0 && at(px, lw, lw / 2, 1)[3] > 150 && white > 200 &&
              at(px, lw, lh / 2 - 6 + 7, lh / 2)[1] == 69,
          "label texture");
    // Box: bold opaque corners, fainter sides, nearly clear middle, nothing outside the rounded corner.
    std::vector<uint8_t> bx = outline(256, 512, 0, 4);
    check(at(bx, 256, 0, 0)[3] == 0 && at(bx, 256, 3, 40)[3] > 230 && at(bx, 256, 3, 256)[3] > 100 &&
              at(bx, 256, 3, 256)[3] < 180 && at(bx, 256, 128, 256)[3] < 20,
          "box texture");

    // Dashboard: rows and buttons hit-test to their controls, pressing them changes the settings.
    check(panel::hit(500, panel::row_cy(panel::kPeople)) == panel::kPeople &&
              panel::hit(panel::plus_cx(), panel::row_cy(panel::kRangeRow)) == panel::kRangePlus &&
              panel::hit(500, panel::row_cy(panel::kHands)) == panel::kHands && panel::hit(500, 50) == -1 &&
              (g_set.enabled = false, panel::hit(500, panel::row_cy(panel::kPeople)) == -1) &&
              panel::hit(500, panel::row_cy(panel::kEnabled)) == panel::kEnabled && (g_set.enabled = true),
          "dashboard hit test");
    panel::press(panel::kPeople);
    for (int i = 0; i < 20; i++) {
        panel::press(panel::kRangePlus);
    }
    bool pressed = !g_set.people && g_set.max_dist == 8 && !wanted(0) && wanted(16);
    panel::press(panel::kPeople);
    g_set.max_dist = 3;
    panel::press(panel::kSize);
    float iy = panel::popup_top(panel::kSize) + 2.5f * panel::kItemH;  // the third choice
    bool picked = panel::popup == panel::kSize && panel::hit(500, iy) == -1 &&
                  panel::hit(panel::drop_x0() + 10, iy) == panel::kItem + 2;
    panel::press(panel::kItem + 2);
    picked = picked && g_set.size == 640 && g_det_size == 640 && panel::popup < 0;
    panel::press(panel::kSize), panel::press(-1);
    panel::press(panel::kRate);
    float bottom = panel::popup_top(panel::kRate) + 4 * panel::kItemH;
    panel::press(panel::kItem + 3);
    check(picked && g_set.size == 640 && g_set.hz == 16 && g_det_hz == 16 && panel::popup < 0 && bottom <= panel::H,
          "dropdowns: pick, a press outside closes, the rate popup fits the panel");
    g_set.size = 320, g_set.hz = 4;
    std::vector<uint8_t> pn = panel::render(panel::kPets, "Watching");
    check(pressed && g_set.people && at(pn, panel::W, 0, 0)[3] == 0 && at(pn, panel::W, 500, 5)[3] > 200,
          "dashboard press and render");

    // Measurements: a person 2 m ahead is kept, in the standing universe; a far one, a cup and a hand holding a
    // controller are not. A controller between you and the person doesn't hide them (different depth).
    vr::HmdMatrix34_t head = {{{1, 0, 0, 0}, {0, 1, 0, 1.6f}, {0, 0, 1, 0}}};
    auto obj = [](int cls, float z, float x) {
        Obj o{cls,  0.9f, z, {{x - 0.3f, 0.5f, -z}, {x + 0.3f, 0.5f, -z}, {x + 0.3f, -1, -z}, {x - 0.3f, -1, -z}},
              true, {}};
        return o;
    };
    Obj hand{0,    0.8f,
             0.6f, {{-0.1f, -0.25f, -0.6f}, {0.1f, -0.25f, -0.6f}, {0.1f, -0.45f, -0.6f}, {-0.1f, -0.45f, -0.6f}},
             true, {}};
    std::vector<XrVector3f> ctrl = {{0, 1.25f, -0.6f},
                                    {0.2f, 1.3f, -0.6f}};  // one in the hand box, one before the person
    std::vector<track::Meas> ms = measurements({hand, obj(0, 2, 0.2f), obj(0, 5, 0), obj(41, 1, 0)}, head, ctrl);
    bool hand_kept_without = measurements({hand}, head, {}).size() == 1;
    g_set.hands = false;
    bool hand_kept_when_off = measurements({hand}, head, ctrl).size() == 1;
    g_set.hands = true;
    check(ms.size() == 1 && hand_kept_without && hand_kept_when_off,
          "box around a controller, far person and cup dropped; person behind a controller kept");
    Obj arm{0,    0.8f,
            0.55f, {{0.18f, -0.2f, -0.55f}, {0.3f, -0.2f, -0.55f}, {0.3f, -0.27f, -0.55f}, {0.18f, -0.27f, -0.55f}},
            true, {}};  // forearm reaching out to a controller 45 cm further, outside the box
    check(measurements({arm}, head, {{0.3f, 1.3f, -1.0f}}).empty() && measurements({arm}, head, {}).size() == 1,
          "arm box that leaves out the controller dropped");
    Obj far_arm = obj(0, 1.2f, 0);  // the arm merged with the chair behind it: measured at 1.2 m, controller at 0.6 m
    check(measurements({far_arm}, head, {{0, 1.3f, -0.6f}}).empty(), "arm box measured too far dropped");
    auto low = [](float z, float top) {  // box from `top` down to the bottom edge of the view (100 deg), 0.8 m deep
        float y1 = -1.2f * z;
        return Obj{0, 0.8f, z, {{-0.3f, top, -z}, {0.3f, top, -z}, {0.3f, y1, -z}, {-0.3f, y1, -z}}, true, {}};
    };
    check(measurements({low(0.8f, -0.3f)}, head, {}).empty() && measurements({low(0.8f, 0.1f)}, head, {}).size() == 1 &&
              measurements({low(2, -0.3f)}, head, {}).size() == 1,
          "bare arm at the bottom edge dropped; with a head above eye level, or 2 m away, kept");
    check(measurements({obj(0, 0.4f, 0)}, head, {}).empty(), "person closer than 50 cm dropped");
    {
        const float near[] = {0, 1.2f, -2}, far[] = {0, 1.2f, -2.5f}, beside[] = {0.8f, 1.2f, -2};
        Det a = angular_box(near, 0.5f, 1.6f, {0, 1.6f, 0});
        check(duplicate(a, angular_box(far, 0.5f, 1.6f, {0, 1.6f, 0})) &&
                  !duplicate(a, angular_box(beside, 0.5f, 1.6f, {0, 1.6f, 0})),
              "one person at two depths is a duplicate, two side by side aren't");
    }
    check(ms.size() == 1 && std::fabs(ms[0].v[0] - 0.2f) < 1e-5f && std::fabs(ms[0].v[1] - 1.35f) < 1e-5f &&
              std::fabs(ms[0].v[3] - 0.6f) < 1e-5f && std::fabs(ms[0].v[4] - 1.5f) < 1e-5f,
          "head frame -> standing universe center and size");
    return fails ? 1 : 0;
}

int main(int argc, char** argv) {
    g_det.conf = 0.1f;  // ByteTrack wants the weak detections too; only >= --track-thresh ones start tracks
    track::Tracker tracker;
    if (const char* home = getenv("HOME")) {
        g_conf = std::string(home) + "/.config/guardian.conf";
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
    vr::VROverlayHandle_t overlays[kMaxOverlays], labels[kMaxOverlays];
    int tex_h[kMaxOverlays] = {}, tex_cls[kMaxOverlays] = {};  // what each box overlay's texture was made for
    std::string label_text[kMaxOverlays];
    for (int i = 0; i < kMaxOverlays * 2; i++) {
        std::string key =
            std::string(kAppKey) + (i < kMaxOverlays ? ".box" : ".label") + std::to_string(i % kMaxOverlays);
        if (vr::VROverlay()->CreateOverlay(key.c_str(), "guardian",
                                           i < kMaxOverlays ? &overlays[i] : &labels[i - kMaxOverlays])) {
            fprintf(stderr, "CreateOverlay %s failed\n", key.c_str());
            return 1;
        }
    }

    vr::VROverlayHandle_t dash, dash_icon;
    if (vr::VROverlay()->CreateDashboardOverlay((std::string(kAppKey) + ".dashboard").c_str(), "Guardian", &dash,
                                                &dash_icon)) {
        fprintf(stderr, "CreateDashboardOverlay failed\n");
        return 1;
    }
    {
        int size;
        std::vector<uint8_t> px = panel::icon(size);
        textures.set(dash_icon, px, size, size);
        vr::VROverlay()->SetOverlayWidthInMeters(dash, 1.5f);
        vr::VROverlay()->SetOverlayInputMethod(dash, vr::VROverlayInputMethod_Mouse);
        vr::HmdVector2_t scale{float(panel::W), float(panel::H)};
        vr::VROverlay()->SetOverlayMouseScale(dash, &scale);
        px = panel::render(-1, "Starting");
        textures.set(dash, px, panel::W, panel::H);
    }
    int hover = -1;
    // A game is starting, loading or running (its intro counts); passthrough itself isn't reported to apps.
    auto game_running = [] {
        return vr::VRApplications()->GetSceneApplicationState() != vr::EVRSceneApplicationState_None;
    };
    bool in_game = game_running();
    std::string panel_status;  // what the panel was last drawn with ("" = redraw)

    std::thread det;
    if (g_detector) {
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
    int shown_before = 0;
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
        while (vr::VROverlay()->PollNextOverlayEvent(dash, &ev, sizeof ev)) {
            // Mouse y is up from the bottom of the texture.
            int c = panel::hit(ev.data.mouse.x, panel::H - ev.data.mouse.y);
            if (ev.eventType == vr::VREvent_MouseMove && c != hover) {
                hover = c, panel_status.clear();
                if (c >= 0) {  // the tick the Steam UI gives when the pointer reaches a new item
                    vr::VRSystem()->TriggerHapticPulse(ev.trackedDeviceIndex, 0, kHoverTickUs);
                }
            }
            if (ev.eventType == vr::VREvent_MouseButtonDown && (c >= 0 || panel::popup >= 0)) {  // outside closes it
                panel::press(c), panel_status.clear();
            }
        }
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

        vr::HmdMatrix34_t head = head_pose(0, ok);
        XrVector3f h{head.m[0][3], head.m[1][3], head.m[2][3]};
        double t_play = now / 1e9 - lag_s;
        std::string where;  // for the log: each shown box relative to the head
        int n = 0;
        // Most recently matched first: when a depth jump misses the tracker's gate, the new track takes over and the
        // old one, still coasting, would be drawn as a second box just in front of or behind it.
        std::vector<size_t> order(tracker.tracks.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            const track::Track &x = tracker.tracks[a], &y = tracker.tracks[b];
            return x.last_seen != y.last_seen ? x.last_seen > y.last_seen : x.score > y.score;
        });
        std::vector<Det> drawn;  // angular boxes (rad) seen from the head, for duplicate()
        for (size_t ti : order) {
            const track::Track& t = tracker.tracks[ti];
            // Confirmed and matched recently (the tracker drops stale tracks only when the next detections come).
            if (!t.confirmed || now / 1e9 - t.last_seen > tracker.prm.max_age || n == kMaxOverlays) {
                continue;
            }
            track::Sample sp = spline_at(t.history, t_play);
            const float* p = sp.v;
            float w = std::max(p[3], 0.05f), ht = std::max(p[4], 0.05f);
            Det a = angular_box(p, w, ht, h);
            if (std::any_of(drawn.begin(), drawn.end(), [&](const Det& b) {
                    return duplicate(a, b);
                })) {
                continue;
            }
            drawn.push_back(a);
            {
                // Bearing from the head's horizontal forward (-z), + = right; height relative to the eyes.
                float vx = p[0] - h.x, vz = p[2] - h.z, fx = -head.m[0][2], fz = -head.m[2][2];
                char b[96];
                snprintf(b, sizeof b, " | %s %.0f%% %.1fm %+.0fdeg %+.1fm", kCoco[t.cls], t.score * 100,
                         std::hypot(vx, vz), std::atan2(fx * vz - fz * vx, fx * vx + fz * vz) * 180 / float(M_PI),
                         p[1] - h.y);
                where += b;
            }
            // 256 texels across; height in steps of 8 so the texture is only redrawn when the aspect really changes
            int pw = 256, ph = std::clamp(int(std::lround(pw * ht / w / 8)) * 8, 32, 1024);
            if (ph != tex_h[n] || t.cls != tex_cls[n]) {
                std::vector<uint8_t> px = outline(pw, ph, t.cls, std::clamp(pw * 0.008f / w, 1.5f, 8.f));  // ~8 mm line
                textures.set(overlays[n], px, pw, ph);
                tex_h[n] = ph, tex_cls[n] = t.cls;
            }
            vr::VROverlay()->SetOverlayWidthInMeters(overlays[n], w);
            vr::HmdMatrix34_t m = facing({p[0], p[1], p[2]}, h);
            vr::VROverlay()->SetOverlayTransformAbsolute(overlays[n], vr::TrackingUniverseStanding, &m);

            int pct = int(std::lround(t.score * 100));
            std::string text = std::string(kCoco[t.cls]) + std::to_string(pct);
            if (label_text[n] != text) {
                int lw, lh;
                std::vector<uint8_t> px = label(kCoco[t.cls], pct, t.cls, lw, lh);
                textures.set(labels[n], px, lw, lh);
                vr::VROverlay()->SetOverlayWidthInMeters(labels[n], kLabelH * lw / lh);
                label_text[n] = text;
            }
            m.m[1][3] += ht / 2 + kLabelH / 2 + 0.015f;  // same sprite plane, just above the box
            vr::VROverlay()->SetOverlayTransformAbsolute(labels[n], vr::TrackingUniverseStanding, &m);
            if (n >= shown_before) {
                vr::VROverlay()->ShowOverlay(overlays[n]), vr::VROverlay()->ShowOverlay(labels[n]);
            }
            n++;
        }
        for (int i = n; i < shown_before; i++) {
            vr::VROverlay()->HideOverlay(overlays[i]), vr::VROverlay()->HideOverlay(labels[i]);
        }
        static uint64_t last_where_ns = 0;
        if (n != shown_before || (n && now - last_where_ns > 2000000000ull)) {
            printf("guardian: showing %d%s\n", n, where.c_str());
            last_where_ns = now;
        }
        shown_before = n;

        if (vr::VROverlay()->IsOverlayVisible(dash)) {
            char st[128];
            if (!g_set.enabled) {
                snprintf(st, sizeof st, "Off");
            } else if (!run) {
                snprintf(st, sizeof st, "Paused: no game running");
            } else if (!g_detector) {
                snprintf(st, sizeof st, "Detector off (--no-detector)");
            } else if (now - last_det_ns > 2000000000ull) {
                snprintf(st, sizeof st, "Waiting for camera frames");
            } else {
                snprintf(st, sizeof st, "Watching  ·  YOLO %.0f ms  ·  %d in view", g_det_yolo_ms.load(), n);
            }
            if (panel_status != st) {
                panel_status = st;
                std::vector<uint8_t> px = panel::render(hover, panel_status);
                textures.set(dash, px, panel::W, panel::H);
            }
        }
    }
    if (det.joinable()) {
        det.join();
    }
    vr::VR_Shutdown();
    ncnn::destroy_gpu_instance();
    return 0;
}
