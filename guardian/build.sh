#!/bin/sh
# guardian's build, the way a Store depot is built: in the Steam Runtime 4 arm64 SDK. Run by make build, or
# directly; deploy.yml installs the result.
# All on the Mac (arm64, so the SDK image runs natively in Docker Desktop's VM): the inputs (ncnn submodule, models,
# OpenVR SDK pieces, font), then ncnn (into third_party/ncnn-install, where clangd finds it) and guardian built with
# docker in the SDK image (build-sdk/, incremental) and `cmake --install`ed into dist/guardian.
set -e
cd "$(dirname "$0")/.."

# get URL FILE: download unless it's already there
get() {
	[ -f "$2" ] && return
	mkdir -p "$(dirname "$2")"
	curl -fsSL -o "$2.part" "$1"
	mv "$2.part" "$2"
}

# ncnn is a submodule: branch "ncnn" of this repo (upstream snapshot + our patches). Its URL is ./ (this repo),
# which needs file transport allowed for submodules.
[ -f third_party/ncnn/CMakeLists.txt ] || git -c protocol.file.allow=always submodule update --init third_party/ncnn

# Exported on the Mac (ultralytics + torch don't run on the Frame), S x 2S input, adapted with prep_model.py
get https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-grayscale.pt models/yolo11n-grayscale.pt
for s in 320 480 640; do
	[ -f "models/yolo11n-grayscale_${s}_gray.ncnn.param" ] && continue
	(
		cd models
		uvx --from ultralytics --with ncnn --with pnnx yolo export model=yolo11n-grayscale.pt format=ncnn imgsz=$((2 * s)),$s
		mv yolo11n-grayscale_ncnn_model/model.ncnn.param "yolo11n-grayscale_$s.ncnn.param"
		mv yolo11n-grayscale_ncnn_model/model.ncnn.bin "yolo11n-grayscale_$s.ncnn.bin"
		rm -r yolo11n-grayscale_ncnn_model
		python3 ../guardian/prep_model.py "yolo11n-grayscale_$s.ncnn"
	)
done

# The OpenVR SDK pieces guardian builds against and ships (libopenvr_api.so next to the binary)
for f in headers/openvr.h headers/openvr_driver.h bin/linuxarm64/libopenvr_api.so; do
	get "https://raw.githubusercontent.com/ValveSoftware/openvr/v2.15.6/$f" "third_party/openvr/$f"
done

# Label font, shipped with the app (OFL)
font=third_party/fonts/NotoSans-Medium.ttf
sum="635d93d1131d791f2576de90b3bb0f7cdf61929906e8420a61b5f7f8e76420bb  $font"
echo "$sum" | shasum -a 256 -c --status 2>/dev/null || {
	rm -f "$font"
	get https://raw.githubusercontent.com/notofonts/notofonts.github.io/d3abdda968b38bb6eb0ec59f6f9f58b8c2139bca/fonts/NotoSans/hinted/ttf/NotoSans-Medium.ttf "$font"
	echo "$sum" | shasum -a 256 -c --status
}

# ncnn links the SDK's glslang (static); SIMPLEVK=OFF: its stub Vulkan types clash with vulkan.h.
# dist/guardian is rebuilt from scratch so it holds exactly what's installed.
docker run --rm -v "$PWD:/src" -w /src registry.gitlab.steamos.cloud/steamrt/steamrt4/sdk/arm64:latest sh -ec '
cmake -S third_party/ncnn -B build-sdk/ncnn -G Ninja -DNCNN_VULKAN=ON -DNCNN_SIMPLEVK=OFF -DNCNN_SYSTEM_GLSLANG=ON \
	-DGLSLANG_TARGET_DIR=/usr/lib/aarch64-linux-gnu/cmake/glslang -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=OFF \
	-DNCNN_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/src/third_party/ncnn-install >/dev/null
cmake --build build-sdk/ncnn
cmake --install build-sdk/ncnn >/dev/null
cmake -S guardian -B build-sdk/guardian -G Ninja -Dncnn_DIR=/src/third_party/ncnn-install/lib/cmake/ncnn >/dev/null
cmake --build build-sdk/guardian
rm -rf dist/guardian
cmake --install build-sdk/guardian --prefix /src/dist/guardian'
