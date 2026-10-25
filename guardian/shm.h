// Shared-memory frame handoff from camserver (driver_guardian, in vrserver) to guardian.
// One seqlock-protected slot per SLAM camera holding the newest normal-exposure Y plane.
#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>

namespace shm {

constexpr const char* kPath = "/dev/shm/steamframe-cams";
constexpr const char* kCalibDir = "/dev/shm/steamframe-calib";  // the driver's copy of /persist's calibration
constexpr int kW = 1056, kH = 1024, kStride = 1152;
constexpr int kNumCams = 2;  // left, right
constexpr const char* kCamNames[kNumCams] = {"slam_left", "slam_right"};

struct Slot {
    std::atomic<uint64_t> seq;  // odd while the writer is copying
    uint64_t t_mono_ns;         // camera timestamp mapped to CLOCK_MONOTONIC (frame arrival)
    alignas(64) uint8_t y[kStride * kH];
};

struct Header {
    Slot slots[kNumCams];
};

inline void write(Slot& s, const uint8_t* y, uint64_t t_ns) {
    // Rounded down to even: a camserver killed mid-write leaves seq odd, and q + 2 would keep it odd for good.
    uint64_t q = s.seq.load(std::memory_order_relaxed) & ~uint64_t(1);
    s.seq.store(q + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(s.y, y, sizeof s.y);
    s.t_mono_ns = t_ns;
    std::atomic_thread_fence(std::memory_order_release);
    s.seq.store(q + 2, std::memory_order_release);
}

// Copies the slot if it changed since `last_seq`. Returns false if unchanged or torn; `y` is garbage after a torn
// read, the other outputs are left alone.
inline bool read(const Slot& s, uint64_t& last_seq, uint8_t* y, uint64_t& t_ns) {
    uint64_t q = s.seq.load(std::memory_order_acquire);
    if (q & 1 || q == last_seq) {
        return false;
    }
    std::memcpy(y, s.y, sizeof s.y);
    uint64_t t = s.t_mono_ns;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (s.seq.load(std::memory_order_relaxed) != q) {
        return false;
    }
    last_seq = q, t_ns = t;
    return true;
}

}  // namespace shm
