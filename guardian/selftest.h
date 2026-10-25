// guardian --selftest: checks that need no headset.
#pragma once
#include "boxes.h"
#include "draw.h"
#include "measure.h"
#include "panel.h"
#include "tracker.h"

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
              panel::hit(500, panel::row_cy(panel::kEnabled)) == panel::kEnabled && (g_set.enabled = true) &&
              panel::hit(500, panel::row_cy(panel::kAutoLaunch)) == -1,  // greyed out: not started by Steam
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
    Obj arm{0,     0.8f,
            0.55f, {{0.18f, -0.2f, -0.55f}, {0.3f, -0.2f, -0.55f}, {0.3f, -0.27f, -0.55f}, {0.18f, -0.27f, -0.55f}},
            true,  {}};  // forearm reaching out to a controller 45 cm further, outside the box
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
