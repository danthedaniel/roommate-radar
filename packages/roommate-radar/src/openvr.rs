//! OpenVR through its C API: VR_InitInternal2, then each interface as a table of function pointers
//! ("FnTable:IVROverlay_028": openvr_capi.h's FnTables, generated into openvr_sys.rs). Wrapped here for the calls
//! roommate-radar makes.
use std::ffi::{CStr, CString, c_char, c_void};
use std::sync::OnceLock;

pub use crate::openvr_sys::*;

unsafe extern "C" {
    fn VR_InitInternal2(err: *mut EVRInitError, ty: EVRApplicationType, startup_info: *const c_char) -> u32;
    fn VR_ShutdownInternal();
    fn VR_GetGenericInterface(version: *const c_char, err: *mut EVRInitError) -> *mut c_void;
    fn VR_GetVRInitErrorAsEnglishDescription(err: EVRInitError) -> *const c_char;
}

// The structs passed below, against openvr.h's layout (the SDK's g++, aarch64).
const _: () = assert!(size_of::<VREvent_t>() == 60);
const _: () = assert!(size_of::<TrackedDevicePose_t>() == 80);
const _: () = assert!(size_of::<VRVulkanTextureData_t>() == 64);
const _: () = assert!(size_of::<Texture_t>() == 16);
const _: () = assert!(size_of::<HmdQuad_t>() == 48);

pub const MAX_TRACKED_DEVICES: usize = 64; // k_unMaxTrackedDeviceCount
const STANDING: ETrackingUniverseOrigin = ETrackingUniverseOrigin_TrackingUniverseStanding;

pub struct Vr {
    system: &'static VR_IVRSystem_FnTable,
    apps: &'static VR_IVRApplications_FnTable,
    overlay: Option<&'static VR_IVROverlay_FnTable>, // not for utility apps (--register)
    compositor: Option<&'static VR_IVRCompositor_FnTable>,
    chaperone: Option<&'static VR_IVRChaperoneSetup_FnTable>,
}

static VR: OnceLock<Vr> = OnceLock::new();

/// The runtime, after init().
pub fn vr() -> &'static Vr {
    VR.get().expect("openvr::init")
}

pub fn init(ty: EVRApplicationType) -> Result<(), String> {
    let mut err = EVRInitError_VRInitError_None;
    unsafe { VR_InitInternal2(&mut err, ty, std::ptr::null()) };
    if err != EVRInitError_VRInitError_None {
        let s = unsafe { CStr::from_ptr(VR_GetVRInitErrorAsEnglishDescription(err)) };
        return Err(s.to_string_lossy().into_owned());
    }
    fn table<T>(version: &str) -> Option<&'static T> {
        let name = CString::new(format!("FnTable:{version}")).unwrap();
        let mut err = 0;
        unsafe { VR_GetGenericInterface(name.as_ptr(), &mut err).cast::<T>().as_ref() }
    }
    let vr = Vr {
        system: table("IVRSystem_026").ok_or("no IVRSystem_026")?,
        apps: table("IVRApplications_008").ok_or("no IVRApplications_008")?,
        overlay: table("IVROverlay_028"),
        compositor: table("IVRCompositor_029"),
        chaperone: table("IVRChaperoneSetup_006"),
    };
    let _ = VR.set(vr);
    Ok(())
}

pub fn shutdown() {
    unsafe { VR_ShutdownInternal() };
}

macro_rules! call {
    ($table:expr, $f:ident $(, $arg:expr)*) => {
        unsafe { ($table.$f.unwrap())($($arg),*) }
    };
}

fn c(s: &str) -> CString {
    CString::new(s).unwrap()
}

impl Vr {
    fn overlay(&self) -> &'static VR_IVROverlay_FnTable {
        self.overlay.expect("IVROverlay_028")
    }

    // IVRSystem

    /// Poses (standing universe) of every tracked device, seconds_from_now (< 0: in the past); [0] is the HMD.
    pub fn poses(&self, seconds_from_now: f32, out: &mut [TrackedDevicePose_t]) {
        call!(
            self.system,
            GetDeviceToAbsoluteTrackingPose,
            STANDING,
            seconds_from_now,
            out.as_mut_ptr(),
            out.len() as u32
        )
    }

    pub fn device_class(&self, i: u32) -> ETrackedDeviceClass {
        call!(self.system, GetTrackedDeviceClass, i)
    }

    pub fn poll_event(&self) -> Option<VREvent_t> {
        let mut ev: VREvent_t = unsafe { std::mem::zeroed() };
        call!(self.system, PollNextEvent, &mut ev, size_of::<VREvent_t>() as u32).then_some(ev)
    }

    pub fn haptic_pulse(&self, device: u32, us: u16) {
        call!(self.system, TriggerHapticPulse, device, 0, us)
    }

    pub fn acknowledge_quit(&self) {
        call!(self.system, AcknowledgeQuit_Exiting)
    }

    /// The VkPhysicalDevice (raw handle) the compositor renders with, for this VkInstance.
    pub fn output_device(&self, instance: u64) -> u64 {
        let mut dev = 0;
        call!(self.system, GetOutputDevice, &mut dev, ETextureType_TextureType_Vulkan, instance as *mut VkInstance_T);
        dev
    }

    // IVRChaperoneSetup

    /// The play area's boundary (standing universe) as its walls' ends on the floor, [x, z]; empty if there's none.
    pub fn play_area_walls(&self) -> Vec<[[f32; 2]; 2]> {
        let Some(t) = self.chaperone else {
            return vec![];
        };
        let mut n = 0;
        call!(t, GetLiveCollisionBoundsInfo, std::ptr::null_mut(), &mut n); // just the count
        let mut quads: Vec<HmdQuad_t> = vec![unsafe { std::mem::zeroed() }; n as usize];
        if !call!(t, GetLiveCollisionBoundsInfo, quads.as_mut_ptr(), &mut n) {
            return vec![];
        }
        quads.truncate(n as usize);
        // a wall is an upright quad: opposite corners 0 and 2 are its two ends, whichever way it's wound
        let xz = |p: &HmdVector3_t| [p.v[0], p.v[2]];
        quads.iter().map(|q| [xz(&q.vCorners[0]), xz(&q.vCorners[2])]).collect()
    }

    // IVRApplications

    pub fn auto_launch(&self, key: &str) -> bool {
        call!(self.apps, GetApplicationAutoLaunch, c(key).as_ptr() as *mut c_char)
    }

    pub fn set_auto_launch(&self, key: &str, on: bool) -> EVRApplicationError {
        call!(self.apps, SetApplicationAutoLaunch, c(key).as_ptr() as *mut c_char, on)
    }

    /// A game is starting, loading or running (its intro counts); passthrough itself isn't reported to apps.
    pub fn game_running(&self) -> bool {
        call!(self.apps, GetSceneApplicationState) != EVRSceneApplicationState_None
    }

    pub fn is_dashboard_overlay(&self, key: &str) -> bool {
        let mut err = 0;
        let prop = EVRApplicationProperty_VRApplicationProperty_IsDashboardOverlay_Bool;
        call!(self.apps, GetApplicationPropertyBool, c(key).as_ptr() as *mut c_char, prop, &mut err)
    }

    pub fn add_manifest(&self, path: &str) -> EVRApplicationError {
        call!(self.apps, AddApplicationManifest, c(path).as_ptr() as *mut c_char, false)
    }

    pub fn app_error_name(&self, e: EVRApplicationError) -> String {
        let name = call!(self.apps, GetApplicationsErrorNameFromEnum, e);
        unsafe { CStr::from_ptr(name) }.to_string_lossy().into()
    }

    // IVROverlay

    pub fn create_overlay(&self, key: &str, name: &str) -> Result<VROverlayHandle_t, EVROverlayError> {
        let mut h = 0;
        let e = call!(self.overlay(), CreateOverlay, c(key).as_ptr() as *mut _, c(name).as_ptr() as *mut _, &mut h);
        if e == 0 { Ok(h) } else { Err(e) }
    }

    /// (the tab, its icon)
    pub fn create_dashboard_overlay(
        &self,
        key: &str,
        name: &str,
    ) -> Result<(VROverlayHandle_t, VROverlayHandle_t), EVROverlayError> {
        let (mut h, mut icon) = (0, 0);
        let (key, name) = (c(key), c(name));
        let e = call!(
            self.overlay(),
            CreateDashboardOverlay,
            key.as_ptr() as *mut _,
            name.as_ptr() as *mut _,
            &mut h,
            &mut icon
        );
        if e == 0 { Ok((h, icon)) } else { Err(e) }
    }

    pub fn set_width(&self, h: VROverlayHandle_t, meters: f32) {
        call!(self.overlay(), SetOverlayWidthInMeters, h, meters);
    }

    /// Placed in the standing universe.
    pub fn set_transform(&self, h: VROverlayHandle_t, m: &HmdMatrix34_t) {
        call!(self.overlay(), SetOverlayTransformAbsolute, h, STANDING, m as *const _ as *mut _);
    }

    pub fn show(&self, h: VROverlayHandle_t) {
        call!(self.overlay(), ShowOverlay, h);
    }

    pub fn hide(&self, h: VROverlayHandle_t) {
        call!(self.overlay(), HideOverlay, h);
    }

    pub fn is_visible(&self, h: VROverlayHandle_t) -> bool {
        call!(self.overlay(), IsOverlayVisible, h)
    }

    pub fn set_texture(&self, h: VROverlayHandle_t, tex: &Texture_t) {
        call!(self.overlay(), SetOverlayTexture, h, tex as *const _ as *mut _);
    }

    /// RGBA pixels, copied.
    pub fn set_raw(&self, h: VROverlayHandle_t, px: &[u8], w: u32, ht: u32) {
        assert!(px.len() >= w as usize * ht as usize * 4);
        call!(self.overlay(), SetOverlayRaw, h, px.as_ptr() as *mut c_void, w, ht, 4);
    }

    /// Mouse events in texture pixels (w x ht).
    pub fn set_mouse_input(&self, h: VROverlayHandle_t, w: f32, ht: f32) {
        call!(self.overlay(), SetOverlayInputMethod, h, VROverlayInputMethod_Mouse);
        call!(self.overlay(), SetOverlayMouseScale, h, &mut HmdVector2_t { v: [w, ht] });
    }

    pub fn poll_overlay_event(&self, h: VROverlayHandle_t) -> Option<VREvent_t> {
        let mut ev: VREvent_t = unsafe { std::mem::zeroed() };
        call!(self.overlay(), PollNextOverlayEvent, h, &mut ev, size_of::<VREvent_t>() as u32).then_some(ev)
    }

    /// Blocks until the compositor's next frame; false if there is none to sync to within timeout_ms.
    pub fn wait_frame_sync(&self, timeout_ms: u32) -> bool {
        call!(self.overlay(), WaitFrameSync, timeout_ms) == 0
    }

    // IVRCompositor

    /// The Vulkan extensions the compositor needs: of the instance (phys = None), or of the device phys.
    pub fn vulkan_extensions(&self, phys: Option<u64>) -> Vec<CString> {
        let mut buf = [0 as c_char; 4096];
        if let Some(cm) = self.compositor {
            match phys {
                None => call!(cm, GetVulkanInstanceExtensionsRequired, buf.as_mut_ptr(), buf.len() as u32),
                Some(p) => call!(
                    cm,
                    GetVulkanDeviceExtensionsRequired,
                    p as *mut VkPhysicalDevice_T,
                    buf.as_mut_ptr(),
                    buf.len() as u32
                ),
            };
        }
        let s = unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned();
        s.split(' ').filter(|e| !e.is_empty()).map(c).collect()
    }
}
