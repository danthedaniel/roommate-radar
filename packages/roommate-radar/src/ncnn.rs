//! ncnn through its C API (c_api.h; third_party/ncnn adds the gpu calls and the winograd63 option).
#![allow(non_camel_case_types)]
use std::ffi::{CString, c_char, c_int, c_void};

type ncnn_net_t = *mut c_void;
type ncnn_option_t = *mut c_void;
type ncnn_extractor_t = *mut c_void;
type ncnn_mat_t = *mut c_void;

const PIXEL_GRAY: c_int = 3; // ncnn::Mat::PIXEL_GRAY

unsafe extern "C" {
    fn ncnn_get_gpu_count() -> c_int;
    fn ncnn_destroy_gpu_instance();
    fn ncnn_net_create() -> ncnn_net_t;
    fn ncnn_net_destroy(net: ncnn_net_t);
    fn ncnn_net_get_option(net: ncnn_net_t) -> ncnn_option_t;
    fn ncnn_option_set_use_vulkan_compute(opt: ncnn_option_t, enable: c_int);
    fn ncnn_option_set_num_threads(opt: ncnn_option_t, n: c_int);
    fn ncnn_option_set_use_sgemm_convolution(opt: ncnn_option_t, enable: c_int);
    fn ncnn_option_set_use_winograd63_convolution(opt: ncnn_option_t, enable: c_int);
    fn ncnn_net_load_param(net: ncnn_net_t, path: *const c_char) -> c_int;
    fn ncnn_net_load_model(net: ncnn_net_t, path: *const c_char) -> c_int;
    fn ncnn_extractor_create(net: ncnn_net_t) -> ncnn_extractor_t;
    fn ncnn_extractor_destroy(ex: ncnn_extractor_t);
    fn ncnn_extractor_input(ex: ncnn_extractor_t, name: *const c_char, mat: ncnn_mat_t) -> c_int;
    fn ncnn_extractor_extract(ex: ncnn_extractor_t, name: *const c_char, mat: *mut ncnn_mat_t) -> c_int;
    fn ncnn_mat_from_pixels(
        px: *const u8,
        ty: c_int,
        w: c_int,
        h: c_int,
        stride: c_int,
        alloc: *mut c_void,
    ) -> ncnn_mat_t;
    fn ncnn_mat_destroy(mat: ncnn_mat_t);
    fn ncnn_mat_get_w(mat: ncnn_mat_t) -> c_int;
    fn ncnn_mat_get_h(mat: ncnn_mat_t) -> c_int;
    fn ncnn_mat_get_data(mat: ncnn_mat_t) -> *mut c_void;
}

/// Before exit, after every Net is gone.
pub fn destroy_gpu_instance() {
    unsafe { ncnn_destroy_gpu_instance() }
}

pub struct Net(ncnn_net_t);

impl Net {
    /// `model`.param and `model`.bin, on the GPU; no GPU is an error, as the CPU is far too slow.
    pub fn load(model: &str) -> Result<Net, String> {
        if unsafe { ncnn_get_gpu_count() } == 0 {
            return Err("ncnn: no Vulkan GPU".into());
        }
        let net = Net(unsafe { ncnn_net_create() });
        unsafe {
            let opt = ncnn_net_get_option(net.0);
            ncnn_option_set_use_vulkan_compute(opt, 1);
            ncnn_option_set_num_threads(opt, 2);
            // ~1 ms faster on the Adreno 750 (Turnip) at every size; ncnn's other Vulkan defaults measured best.
            ncnn_option_set_use_sgemm_convolution(opt, 0);
            ncnn_option_set_use_winograd63_convolution(opt, 0);
        }
        let (param, bin) =
            (CString::new(format!("{model}.param")).unwrap(), CString::new(format!("{model}.bin")).unwrap());
        if unsafe { ncnn_net_load_param(net.0, param.as_ptr()) != 0 || ncnn_net_load_model(net.0, bin.as_ptr()) != 0 } {
            return Err(format!("failed to load model {model} (see prep_model.py)"));
        }
        Ok(net)
    }

    /// Runs the net on a w x h gray image (input "in0"); output "out0", None if extraction failed.
    pub fn run_gray(&self, px: &[u8], w: usize, h: usize) -> Option<Mat> {
        assert!(px.len() >= w * h);
        unsafe {
            let ex = ncnn_extractor_create(self.0);
            let input = Mat(ncnn_mat_from_pixels(
                px.as_ptr(),
                PIXEL_GRAY,
                w as c_int,
                h as c_int,
                w as c_int,
                std::ptr::null_mut(),
            ));
            let mut out = std::ptr::null_mut();
            let ok = ncnn_extractor_input(ex, c"in0".as_ptr(), input.0) == 0
                && ncnn_extractor_extract(ex, c"out0".as_ptr(), &mut out) == 0;
            ncnn_extractor_destroy(ex);
            let out = (!out.is_null()).then_some(Mat(out));
            if ok { out } else { None }
        }
    }
}

impl Drop for Net {
    fn drop(&mut self) {
        unsafe { ncnn_net_destroy(self.0) }
    }
}

/// A 2D float output (w x h).
pub struct Mat(ncnn_mat_t);

impl Mat {
    pub fn w(&self) -> usize {
        unsafe { ncnn_mat_get_w(self.0) as usize }
    }

    pub fn h(&self) -> usize {
        unsafe { ncnn_mat_get_h(self.0) as usize }
    }

    pub fn row(&self, y: usize) -> &[f32] {
        assert!(y < self.h());
        let w = self.w();
        unsafe { std::slice::from_raw_parts(ncnn_mat_get_data(self.0).cast::<f32>().add(y * w), w) }
    }
}

impl Drop for Mat {
    fn drop(&mut self) {
        unsafe { ncnn_mat_destroy(self.0) }
    }
}
