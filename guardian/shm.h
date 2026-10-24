// Shared-memory frame handoff from camserver (root) to guardian (user).
// One seqlock-protected slot per SLAM camera holding the newest normal-exposure Y plane.
#pragma once
#include <atomic>
#include <cstdint>
#include <cstring>

namespace shm {

constexpr const char* kPath = "/dev/shm/steamframe-cams";
constexpr uint32_t kMagic = 0x53464331;  // "SFC1"
constexpr int kW = 1056, kH = 1024, kStride = 1152;
enum Cam {
    kLeft = 0,
    kRight = 1,
    kNumCams = 2
};
constexpr const char* kCamNames[kNumCams] = {"slam_left", "slam_right"};

struct Slot {
    std::atomic<uint64_t> seq;  // odd while the writer is copying
    uint64_t t_mono_ns;         // camera timestamp mapped to CLOCK_MONOTONIC (frame arrival)
    uint32_t frame_seq;         // V4L2 sequence number
    uint32_t pad;
    alignas(64) uint8_t y[kStride * kH];
};

struct Header {
    uint32_t magic;
    uint32_t pad;
    Slot slots[kNumCams];
};

inline void write(Slot& s, const uint8_t* y, uint64_t t_ns, uint32_t frame_seq) {
    uint64_t q = s.seq.load(std::memory_order_relaxed);
    s.seq.store(q + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(s.y, y, sizeof s.y);
    s.t_mono_ns = t_ns;
    s.frame_seq = frame_seq;
    std::atomic_thread_fence(std::memory_order_release);
    s.seq.store(q + 2, std::memory_order_release);
}

// Copies the slot if it changed since `last_seq`. Returns false if unchanged or torn.
inline bool read(const Slot& s, uint64_t& last_seq, uint8_t* y, uint64_t& t_ns, uint32_t& frame_seq) {
    uint64_t q = s.seq.load(std::memory_order_acquire);
    if (q & 1 || q == last_seq) {
        return false;
    }
    std::memcpy(y, s.y, sizeof s.y);
    t_ns = s.t_mono_ns;
    frame_seq = s.frame_seq;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (s.seq.load(std::memory_order_relaxed) != q) {
        return false;
    }
    last_seq = q;
    return true;
}

}  // namespace shm
