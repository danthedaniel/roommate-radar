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
    chaperone: Option<&'static VR_IVRChaperone_FnTable>,
    chaperone_setup: Option<&'static VR_IVRChaperoneSetup_FnTable>,
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
        chaperone: table("IVRChaperone_004"),
        chaperone_setup: table("IVRChaperoneSetup_006"),
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

fn cstring(s: &str) -> CString {
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

    /// The left hand's controller, if there is one.
    pub fn left_controller(&self) -> Option<u32> {
        let i = call!(
            self.system,
            GetTrackedDeviceIndexForControllerRole,
            ETrackedControllerRole_TrackedControllerRole_LeftHand
        );
        ((i as usize) < MAX_TRACKED_DEVICES).then_some(i)
    }

    pub fn device_class(&self, device_index: u32) -> ETrackedDeviceClass {
        call!(self.system, GetTrackedDeviceClass, device_index)
    }

    pub fn poll_event(&self) -> Option<VREvent_t> {
        let mut ev: VREvent_t = unsafe { std::mem::zeroed() };
        call!(self.system, PollNextEvent, &mut ev, size_of::<VREvent_t>() as u32).then_some(ev)
    }

    pub fn haptic_pulse(&self, device_index: u32, duration_us: u16) {
        call!(self.system, TriggerHapticPulse, device_index, 0, duration_us)
    }

    pub fn acknowledge_quit(&self) {
        call!(self.system, AcknowledgeQuit_Exiting)
    }

    /// The VkPhysicalDevice (raw handle) the compositor renders with, for this VkInstance.
    pub fn vk_physical_device(&self, instance: u64) -> u64 {
        let mut phys = 0;
        call!(self.system, GetOutputDevice, &mut phys, ETextureType_TextureType_Vulkan, instance as *mut VkInstance_T);
        phys
    }

    // IVRChaperone, IVRChaperoneSetup

    /// SteamVR's boundary (the live collision bounds, standing universe; not the play area's rectangle) as its walls'
    /// ends on the floor, [x, z]; empty if there's none.
    /// Reread from disk: this process's copy is loaded when it connects and SteamVR doesn't update it when the
    /// boundary is redrawn (it sends ChaperoneDataHasChanged/UniverseHasChanged instead). Both copies: IVRChaperone's
    /// holds the standing origin the poses use (left stale, a process started before the boundary was drawn put the
    /// head 8 m from where a new one did), IVRChaperoneSetup's the walls.
    pub fn boundary_walls(&self) -> Vec<[[f32; 2]; 2]> {
        let (Some(chaperone), Some(setup)) = (self.chaperone, self.chaperone_setup) else {
            return vec![];
        };
        call!(chaperone, ReloadInfo);
        call!(setup, ReloadFromDisk, EChaperoneConfigFile_Live);
        let mut n = 0;
        call!(setup, GetLiveCollisionBoundsInfo, std::ptr::null_mut(), &mut n); // just the count
        let mut quads: Vec<HmdQuad_t> = vec![unsafe { std::mem::zeroed() }; n as usize];
        if !call!(setup, GetLiveCollisionBoundsInfo, quads.as_mut_ptr(), &mut n) {
            return vec![];
        }
        quads.truncate(n as usize);
        // a wall is an upright quad: opposite corners 0 and 2 are its two ends, whichever way it's wound
        let xz = |p: &HmdVector3_t| [p.v[0], p.v[2]];
        let walls: Vec<_> = quads.iter().map(|q| [xz(&q.vCorners[0]), xz(&q.vCorners[2])]).collect();
        if is_placeholder(&walls) { vec![] } else { walls }
    }

    // IVRApplications

    pub fn auto_launch(&self, key: &str) -> bool {
        call!(self.apps, GetApplicationAutoLaunch, cstring(key).as_ptr() as *mut c_char)
    }

    pub fn set_auto_launch(&self, key: &str, on: bool) -> EVRApplicationError {
        call!(self.apps, SetApplicationAutoLaunch, cstring(key).as_ptr() as *mut c_char, on)
    }

    /// A game is starting, running, waiting or quitting (its intro counts); passthrough itself isn't reported to apps.
    pub fn game_running(&self) -> bool {
        call!(self.apps, GetSceneApplicationState) != EVRSceneApplicationState_None
    }

    pub fn is_dashboard_overlay(&self, key: &str) -> bool {
        let mut err = 0;
        let prop = EVRApplicationProperty_VRApplicationProperty_IsDashboardOverlay_Bool;
        call!(self.apps, GetApplicationPropertyBool, cstring(key).as_ptr() as *mut c_char, prop, &mut err)
    }

    pub fn add_manifest(&self, path: &str) -> EVRApplicationError {
        call!(self.apps, AddApplicationManifest, cstring(path).as_ptr() as *mut c_char, false)
    }

    pub fn app_error_name(&self, e: EVRApplicationError) -> String {
        let name = call!(self.apps, GetApplicationsErrorNameFromEnum, e);
        unsafe { CStr::from_ptr(name) }.to_string_lossy().into()
    }

    // IVROverlay

    pub fn create_overlay(&self, key: &str, name: &str) -> Result<VROverlayHandle_t, EVROverlayError> {
        let mut h = 0;
        let e = call!(
            self.overlay(),
            CreateOverlay,
            cstring(key).as_ptr() as *mut _,
            cstring(name).as_ptr() as *mut _,
            &mut h
        );
        if e == 0 { Ok(h) } else { Err(e) }
    }

    /// (the tab, its icon)
    pub fn create_dashboard_overlay(
        &self,
        key: &str,
        name: &str,
    ) -> Result<(VROverlayHandle_t, VROverlayHandle_t), EVROverlayError> {
        let (mut tab, mut icon) = (0, 0);
        let (key, name) = (cstring(key), cstring(name));
        let e = call!(
            self.overlay(),
            CreateDashboardOverlay,
            key.as_ptr() as *mut _,
            name.as_ptr() as *mut _,
            &mut tab,
            &mut icon
        );
        if e == 0 { Ok((tab, icon)) } else { Err(e) }
    }

    pub fn set_width(&self, h: VROverlayHandle_t, meters: f32) {
        call!(self.overlay(), SetOverlayWidthInMeters, h, meters);
    }

    /// Placed in the standing universe.
    pub fn set_transform(&self, h: VROverlayHandle_t, m: &HmdMatrix34_t) {
        call!(self.overlay(), SetOverlayTransformAbsolute, h, STANDING, m as *const _ as *mut _);
    }

    pub fn set_alpha(&self, h: VROverlayHandle_t, alpha: f32) {
        call!(self.overlay(), SetOverlayAlpha, h, alpha);
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
    pub fn set_raw(&self, h: VROverlayHandle_t, px: &[u8], width: u32, height: u32) {
        assert!(px.len() >= width as usize * height as usize * 4);
        call!(self.overlay(), SetOverlayRaw, h, px.as_ptr() as *mut c_void, width, height, 4);
    }

    /// Mouse events in texture pixels (width x height).
    pub fn set_mouse_input(&self, h: VROverlayHandle_t, width: f32, height: f32) {
        call!(self.overlay(), SetOverlayInputMethod, h, VROverlayInputMethod_Mouse);
        call!(self.overlay(), SetOverlayMouseScale, h, &mut HmdVector2_t { v: [width, height] });
    }

    pub fn poll_overlay_event(&self, h: VROverlayHandle_t) -> Option<VREvent_t> {
        let mut ev: VREvent_t = unsafe { std::mem::zeroed() };
        call!(self.overlay(), PollNextOverlayEvent, h, &mut ev, size_of::<VREvent_t>() as u32).then_some(ev)
    }

    /// Opens the dashboard on the dashboard overlay `key`.
    pub fn show_dashboard(&self, key: &str) {
        call!(self.overlay(), ShowDashboard, cstring(key).as_ptr() as *mut _);
    }

    /// Blocks until the compositor's next frame; false if there is none to sync to within timeout_ms.
    pub fn wait_frame_sync(&self, timeout_ms: u32) -> bool {
        call!(self.overlay(), WaitFrameSync, timeout_ms) == 0
    }

    // IVRCompositor

    /// The Vulkan extensions the compositor needs: of the instance (physical_device = None), or of the device
    /// physical_device (a raw VkPhysicalDevice).
    pub fn vulkan_extensions(&self, physical_device: Option<u64>) -> Vec<CString> {
        let mut buf = [0u8; 4096];
        if let Some(compositor) = self.compositor {
            match physical_device {
                None => {
                    call!(compositor, GetVulkanInstanceExtensionsRequired, buf.as_mut_ptr().cast(), buf.len() as u32)
                }
                Some(phys) => call!(
                    compositor,
                    GetVulkanDeviceExtensionsRequired,
                    phys as *mut VkPhysicalDevice_T,
                    buf.as_mut_ptr().cast(),
                    buf.len() as u32
                ),
            };
        }
        let ext_list = CStr::from_bytes_until_nul(&buf).map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        ext_list.split(' ').filter(|ext| !ext.is_empty()).map(cstring).collect()
    }
}

/// The Frame's "no boundary": when tracking starts in a room it doesn't recognize (after standby, say), SteamVR makes a
/// universe "without bounds" that still reports a 1 x 1 m square around the origin as its walls (calibration state OK),
/// wherever you are. Drawn boundaries are traced, not squares.
fn is_placeholder(walls: &[[[f32; 2]; 2]]) -> bool {
    walls.len() == 4 && walls.iter().flatten().flatten().all(|v| (v.abs() - 0.5).abs() < 1e-3)
}

#[cfg(test)]
mod tests {
    #[test]
    fn placeholder() {
        // as in chaperone_info.vrchap
        let sq = [
            [[-0.5, -0.5], [-0.5, 0.5]],
            [[-0.5, 0.5], [0.5, 0.5]],
            [[0.5, 0.5], [0.5, -0.5]],
            [[0.5, -0.5], [-0.5, -0.5]],
        ];
        assert!(super::is_placeholder(&sq));
        let mut drawn = sq;
        drawn[1][1][0] = 0.6;
        assert!(!super::is_placeholder(&drawn) && !super::is_placeholder(&sq[..3]));
    }
}
