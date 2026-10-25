// SLAM camera calibration (calib.txt, written by calib.py): Kannala-Brandt intrinsics + head_from_cam pose per camera,
// and the rectification it gives.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "shm.h"

struct CamCal {
    float fx, fy, cx, cy, k[4];
    float R[9];  // head_from_cam rotation, row-major; camera frame is OpenCV (x right, y down, z fwd)
    float t[3];  // camera position in head frame (m)
};

static CamCal load_calib(const std::string& path, const char* name) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) {
        throw std::runtime_error("cannot open " + path + " (run calib.py)");
    }
    char line[2048];
    while (fgets(line, sizeof line, f)) {
        char n[64];
        CamCal c;
        float* v[] = {&c.fx,   &c.fy,   &c.cx,   &c.cy,   &c.k[0], &c.k[1], &c.k[2], &c.k[3], &c.R[0], &c.R[1],
                      &c.R[2], &c.R[3], &c.R[4], &c.R[5], &c.R[6], &c.R[7], &c.R[8], &c.t[0], &c.t[1], &c.t[2]};
        int off = 0;
        if (sscanf(line, "%63s%n", n, &off) != 1 || strcmp(n, name)) {
            continue;
        }
        char* p = line + off;
        for (float* x : v) {
            *x = strtof(p, &p);
        }
        fclose(f);
        return c;
    }
    fclose(f);
    throw std::runtime_error(std::string("camera ") + name + " not in " + path);
}

// ---------------------------------------------------------------------------------------------
// Rectification: virtual pinhole camera at the SLAM camera's position, axes aligned with the head
// (x right, y down, looking along head -z). Both views share orientation -> rectified stereo pair.

struct Rectifier {
    int S;
    float f;
    std::vector<float> map;  // (src x, src y) per output pixel; x < 0 = outside the fisheye

    Rectifier(const CamCal& c, int size, float hfov_deg) : S(size), map(size * size * 2) {
        f = (S / 2.0f) / std::tan(hfov_deg * float(M_PI) / 360.0f);
        for (int v = 0; v < S; v++) {
            for (int u = 0; u < S; u++) {
                // Ray in head frame for virtual pixel center (u, v).
                float hx = (u + 0.5f - S / 2.0f) / f, hy = -(v + 0.5f - S / 2.0f) / f, hz = -1;
                // cam_from_head = R^T
                float x = c.R[0] * hx + c.R[3] * hy + c.R[6] * hz;
                float y = c.R[1] * hx + c.R[4] * hy + c.R[7] * hz;
                float z = c.R[2] * hx + c.R[5] * hy + c.R[8] * hz;
                float* m = &map[(v * S + u) * 2];
                float r = std::sqrt(x * x + y * y), th = std::atan2(r, z);
                if (th > 88 * float(M_PI) / 180) {
                    m[0] = -1;
                    continue;
                }
                float t2 = th * th;
                float thd = th * (1 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3]))));
                float s = r > 1e-9f ? thd / r : 0;
                m[0] = c.fx * x * s + c.cx;
                m[1] = c.fy * y * s + c.cy;
                if (m[0] < 0 || m[1] < 0 || m[0] > shm::kW - 2 || m[1] > shm::kH - 2) {
                    m[0] = -1;
                }
            }
        }
    }

    void apply(const uint8_t* src, uint8_t* dst) const {
        for (int i = 0; i < S * S; i++) {
            float sx = map[i * 2], sy = map[i * 2 + 1];
            if (sx < 0) {
                dst[i] = 0;
                continue;
            }
            int x = int(sx), y = int(sy);
            float ax = sx - x, ay = sy - y;
            const uint8_t* p = src + y * shm::kStride + x;
            float top = p[0] + ax * (p[1] - p[0]);
            float bot = p[shm::kStride] + ax * (p[shm::kStride + 1] - p[shm::kStride]);
            dst[i] = uint8_t(top + ay * (bot - top) + 0.5f);
        }
    }
};
