//! driver_roommate_radar: SteamVR driver that runs camserver's loop (camserver.rs) on a thread inside vrserver, and
//! starts the app with SteamVR (autostart.rs).
//! vrserver is XRService's parent, so Yama (ptrace_scope=1) lets it pidfd_getfd XRService's camera fds without root or
//! CAP_SYS_PTRACE. Adds no devices. Log: vrserver.txt. roommate-radar registers it with vrpathreg on its first run
//! (register.rs).
//!
//! openvr_driver.h is C++ only: vrserver calls the virtual methods of the IServerTrackedDeviceProvider that
//! HmdDriverFactory returns, and the driver logs through an IVRDriverLog the same way. So these are laid out like those
//! C++ objects (Itanium ABI: a pointer to a vtable of the virtual methods in declaration order, `this` first). An
//! interface's layout is frozen for its version string, so they can't change under us.
use std::ffi::{CStr, CString, c_char, c_void};
use std::ptr::{null, null_mut};
use std::sync::Mutex;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::thread::JoinHandle;

macro_rules! log {
    ($($arg:tt)*) => { $crate::log_line(&format!($($arg)*)) };
}

mod autostart;
mod camserver;
#[allow(dead_code)] // the reader's half is roommate-radar's
#[path = "../shm.rs"]
mod shm;

const PROVIDER_VERSION: &CStr = c"IServerTrackedDeviceProvider_004";
const LOG_VERSION: &CStr = c"IVRDriverLog_001";
const INIT_ERROR_NONE: i32 = 0;
const INIT_ERROR_INTERFACE_NOT_FOUND: i32 = 105;

struct Versions([*const c_char; 13]);
unsafe impl Sync for Versions {}
// vr::k_InterfaceVersions of openvr_driver.h v2.15.6: the interface versions a driver built against it reports.
static INTERFACE_VERSIONS: Versions = Versions([
    c"IVRSettings_003".as_ptr(),
    c"ITrackedDeviceServerDriver_005".as_ptr(),
    c"IVRDisplayComponent_003".as_ptr(),
    c"IVRDriverDirectModeComponent_009".as_ptr(),
    c"IVRCameraComponent_003".as_ptr(),
    PROVIDER_VERSION.as_ptr(),
    c"IVRWatchdogProvider_001".as_ptr(),
    c"IVRVirtualDisplay_002".as_ptr(),
    c"IVRDriverManager_001".as_ptr(),
    c"IVRResources_001".as_ptr(),
    c"IVRCompositorPluginProvider_001".as_ptr(),
    c"IVRIPCResourceManagerClient_003".as_ptr(),
    null(),
]);

// vr::IServerTrackedDeviceProvider
#[repr(C)]
struct Provider {
    vtable: &'static ProviderVtable,
}
#[repr(C)]
struct ProviderVtable {
    init: unsafe extern "C" fn(*mut Provider, *mut DriverContext) -> i32,
    cleanup: unsafe extern "C" fn(*mut Provider),
    get_interface_versions: unsafe extern "C" fn(*mut Provider) -> *const *const c_char,
    run_frame: unsafe extern "C" fn(*mut Provider),
    should_block_standby_mode: unsafe extern "C" fn(*mut Provider) -> bool,
    enter_standby: unsafe extern "C" fn(*mut Provider),
    leave_standby: unsafe extern "C" fn(*mut Provider),
}

// vr::IVRDriverContext (its first method only)
#[repr(C)]
struct DriverContext {
    vtable: *const extern "C" fn(*mut DriverContext, *const c_char, *mut i32) -> *mut c_void, // GetGenericInterface
}

// vr::IVRDriverLog
#[repr(C)]
struct DriverLog {
    vtable: *const extern "C" fn(*mut DriverLog, *const c_char), // Log
}

static PROVIDER: Provider = Provider {
    vtable: &ProviderVtable {
        init,
        cleanup,
        get_interface_versions,
        run_frame: noop,
        should_block_standby_mode,
        enter_standby: noop,
        leave_standby: noop,
    },
};
static DRIVER_LOG: AtomicPtr<DriverLog> = AtomicPtr::new(null_mut());
static THREADS: Mutex<Vec<JoinHandle<()>>> = Mutex::new(Vec::new()); // camserver, autostart

/// To vrserver.txt once vrserver gave us its log, else stderr.
fn log_line(line: &str) {
    let log = DRIVER_LOG.load(Ordering::Acquire);
    if log.is_null() {
        eprint!("{line}");
        return;
    }
    let line = CString::new(line.replace('\0', "")).unwrap();
    unsafe { (*(*log).vtable)(log, line.as_ptr()) };
}

unsafe extern "C" fn init(_: *mut Provider, ctx: *mut DriverContext) -> i32 {
    let mut err = 0;
    let log = unsafe { (*(*ctx).vtable)(ctx, LOG_VERSION.as_ptr(), &mut err) };
    if log.is_null() {
        return INIT_ERROR_INTERFACE_NOT_FOUND;
    }
    DRIVER_LOG.store(log.cast(), Ordering::Release);
    // roommate-radar runs in the Steam Runtime container, which doesn't see /persist: copy the factory calibration next
    // to the frames (calib.rs reads it).
    let _ = std::fs::create_dir_all(shm::CALIB_DIR);
    for f in ["xrservice.json", "device_config.json"] {
        if let Err(e) = std::fs::copy(format!("/persist/{f}"), format!("{}/{f}", shm::CALIB_DIR)) {
            log!("copy /persist/{f}: {e}\n");
        }
    }
    *THREADS.lock().unwrap() = vec![std::thread::spawn(camserver::run), std::thread::spawn(autostart::run)];
    INIT_ERROR_NONE
}

unsafe extern "C" fn cleanup(_: *mut Provider) {
    camserver::stop();
    autostart::stop();
    for t in THREADS.lock().unwrap().drain(..) {
        let _ = t.join();
    }
    DRIVER_LOG.store(null_mut(), Ordering::Release);
}

unsafe extern "C" fn get_interface_versions(_: *mut Provider) -> *const *const c_char {
    INTERFACE_VERSIONS.0.as_ptr()
}

unsafe extern "C" fn should_block_standby_mode(_: *mut Provider) -> bool {
    false
}

unsafe extern "C" fn noop(_: *mut Provider) {}

/// vrserver asks for the provider here.
///
/// # Safety
/// `iface` is a C string, `err` null or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn HmdDriverFactory(iface: *const c_char, err: *mut i32) -> *mut c_void {
    if unsafe { CStr::from_ptr(iface) } == PROVIDER_VERSION {
        return &PROVIDER as *const Provider as *mut c_void;
    }
    if !err.is_null() {
        unsafe { *err = INIT_ERROR_INTERFACE_NOT_FOUND };
    }
    null_mut()
}
