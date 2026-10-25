//! Shared-memory frame handoff from camserver (driver_guardian, in vrserver) to guardian.
//! One seqlock-protected slot per SLAM camera holding the newest normal-exposure Y plane.
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU64, Ordering, fence};

pub const PATH: &str = "/dev/shm/steamframe-cams";
pub const CALIB_DIR: &str = "/dev/shm/steamframe-calib"; // the driver's copy of /persist's calibration
pub const W: usize = 1056;
pub const H: usize = 1024;
pub const STRIDE: usize = 1152;
pub const FRAME: usize = STRIDE * H;
pub const NUM_CAMS: usize = 2; // left, right
pub const CAM_NAMES: [&str; NUM_CAMS] = ["slam_left", "slam_right"];

#[repr(C, align(64))]
struct Plane(UnsafeCell<[u8; FRAME]>);

#[repr(C)]
pub struct Slot {
    seq: AtomicU64,       // odd while the writer is copying
    t_mono_ns: AtomicU64, // camera timestamp mapped to CLOCK_MONOTONIC (frame arrival)
    y: Plane,
}

#[repr(C)]
pub struct Header {
    pub slots: [Slot; NUM_CAMS],
}

// The writer and the reader are in different processes; the seqlock is what keeps a reader off a torn plane.
unsafe impl Sync for Header {}

pub fn write(s: &Slot, y: &[u8], t_ns: u64) {
    // Rounded down to even: a camserver killed mid-write leaves seq odd, and q + 2 would keep it odd for good.
    let q = s.seq.load(Ordering::Relaxed) & !1;
    s.seq.store(q + 1, Ordering::Relaxed);
    fence(Ordering::Release);
    unsafe { std::ptr::copy_nonoverlapping(y[..FRAME].as_ptr(), s.y.0.get().cast::<u8>(), FRAME) };
    s.t_mono_ns.store(t_ns, Ordering::Relaxed);
    s.seq.store(q + 2, Ordering::Release);
}

/// Copies the slot into `y` if it changed since `last_seq`, returning its timestamp. None if unchanged or torn (`y` is
/// garbage after a torn read).
pub fn read(s: &Slot, last_seq: &mut u64, y: &mut [u8]) -> Option<u64> {
    let q = s.seq.load(Ordering::Acquire);
    if q & 1 != 0 || q == *last_seq {
        return None;
    }
    unsafe { std::ptr::copy_nonoverlapping(s.y.0.get().cast::<u8>(), y[..FRAME].as_mut_ptr(), FRAME) };
    let t = s.t_mono_ns.load(Ordering::Relaxed);
    fence(Ordering::Acquire);
    if s.seq.load(Ordering::Relaxed) != q {
        return None;
    }
    *last_seq = q;
    Some(t)
}

/// CLOCK_MONOTONIC (ns), the slots' clock.
pub fn now_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}
