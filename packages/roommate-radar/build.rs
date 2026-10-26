// Links the roommate-radar binary (and its tests) against ncnn (third_party/ncnn-install, built by build.sh with the
// SDK's static glslang) and the OpenVR SDK's libopenvr_api.so, shipped next to the binary. The driver links neither.
fn main() {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
    for arg in [
        &format!("{root}/third_party/ncnn-install/lib/libncnn.a"),
        "-l:libSPIRV.a",
        "-l:libglslang.a",
        "-l:libSPIRV-Tools-opt.a",
        "-l:libSPIRV-Tools.a",
        "-lgomp",
        "-lvulkan",
        "-lstdc++",
        &format!("-L{root}/third_party/openvr/bin/linuxarm64"),
        "-lopenvr_api",
        "-Wl,-rpath,$ORIGIN",
        // what g++ adds after them, as rustc's own -lm -lc come before: libm (and its libmvec), libgcc's outline
        // atomics, glibc
        "-lm",
        "-lgcc",
        "-lc",
    ] {
        println!("cargo::rustc-link-arg-bins={arg}");
    }
    println!("cargo::rerun-if-changed={root}/third_party/ncnn-install/lib/libncnn.a");
}
