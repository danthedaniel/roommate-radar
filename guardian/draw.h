// CPU drawing into straight-alpha RGBA pixels (anti-aliased shapes, FreeType text): the box and label textures here,
// the dashboard tab in panel.h.
#pragma once
#include <ft2build.h>
#include FT_FREETYPE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

static const char* kFont = "NotoSans-Medium.ttf";  // shipped with the app

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
