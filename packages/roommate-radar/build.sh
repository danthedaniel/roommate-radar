#!/bin/sh
# roommate-radar's build, the way a Store depot is built: in the Steam Runtime 4 arm64 SDK. Run by make build, or
# directly; deploy.yml installs the result.
# All on the Mac (arm64, so the SDK image runs natively in Docker Desktop's VM): the inputs (ncnn submodule, models,
# OpenVR SDK pieces, font, rustup), then ncnn (into third_party/ncnn-install, where roommate-radar's build.rs finds it) and
# roommate-radar (cargo, the toolchain in rust-toolchain.toml) built and tested with docker in the SDK image
# (build-sdk/, incremental), and laid out in dist/roommate-radar as Steam installs it:
#   NotoSans-Medium.ttf models/  bin/{roommate-radar,libopenvr_api.so}
#   driver/{driver.vrdrivermanifest,bin/linuxarm64/driver_roommate_radar.so}
set -e
cd "$(dirname "$0")/../.."

# get URL FILE SHA256: download unless FILE is already there with that hash; fails if the download doesn't match
get() {
	echo "$3  $2" | shasum -a 256 -c --status 2>/dev/null && return
	mkdir -p "$(dirname "$2")"
	curl -fsSL -o "$2.part" "$1"
	echo "$3  $2.part" | shasum -a 256 -c --status || {
		rm "$2.part"
		echo "$2: wrong sha256 from $1" >&2
		exit 1
	}
	mv "$2.part" "$2"
}

# ncnn is a submodule: github.com/danthedaniel/ncnn (upstream snapshot + our patches)
[ -f third_party/ncnn/CMakeLists.txt ] || git submodule update --init third_party/ncnn

# Exported on the Mac (ultralytics + torch don't run on the Frame), S x 2S input, adapted with prep_model.py
get https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-grayscale.pt models/yolo11n-grayscale.pt \
	b81f01d60918e3194df21c305d9bd2275ab33cfddc79b26ca145a9190d1932f5
for s in 320 480 640; do
	[ -f "models/yolo11n-grayscale_${s}_gray.ncnn.param" ] && continue
	(
		cd models
		uvx --from ultralytics --with ncnn --with pnnx yolo export model=yolo11n-grayscale.pt format=ncnn imgsz=$((2 * s)),$s
		mv yolo11n-grayscale_ncnn_model/model.ncnn.param "yolo11n-grayscale_$s.ncnn.param"
		mv yolo11n-grayscale_ncnn_model/model.ncnn.bin "yolo11n-grayscale_$s.ncnn.bin"
		rm -r yolo11n-grayscale_ncnn_model
		python3 ../packages/roommate-radar/prep_model.py "yolo11n-grayscale_$s.ncnn"
	)
done

# The OpenVR SDK's client library, which roommate-radar links and ships next to the binary (its C API: src/openvr_sys.rs)
get https://raw.githubusercontent.com/ValveSoftware/openvr/v2.15.6/bin/linuxarm64/libopenvr_api.so \
	third_party/openvr/bin/linuxarm64/libopenvr_api.so \
	cc3671d24dd23fb61494e8a8327cf4fe7f638a60508ff7225984a03f2f5947e9

# Label font, shipped with the app (OFL)
get https://raw.githubusercontent.com/notofonts/notofonts.github.io/d3abdda968b38bb6eb0ec59f6f9f58b8c2139bca/fonts/NotoSans/hinted/ttf/NotoSans-Medium.ttf \
	third_party/fonts/NotoSans-Medium.ttf 635d93d1131d791f2576de90b3bb0f7cdf61929906e8420a61b5f7f8e76420bb

# rustup, which installs the toolchain into build-sdk/rust
get https://static.rust-lang.org/rustup/archive/1.29.2/aarch64-unknown-linux-gnu/rustup-init build-sdk/rust/rustup-init \
	a9c168dbea81a84671c6b7a5c1795cb0eb699020110d107ddfca427bd797e697

# ncnn links the SDK's glslang (static); SIMPLEVK=OFF: its stub Vulkan types clash with vulkan.h. The tests load
# libopenvr_api.so (roommate-radar links it) from the SDK pieces. dist/roommate-radar is rebuilt from scratch so it holds exactly
# what's installed.
docker run --rm -v "$PWD:/src" -w /src -e RUSTUP_HOME=/src/build-sdk/rust/rustup -e CARGO_HOME=/src/build-sdk/rust/cargo \
	-e CARGO_TARGET_DIR=/src/build-sdk/roommate-radar registry.gitlab.steamos.cloud/steamrt/steamrt4/sdk/arm64:latest sh -ec '
cmake -S third_party/ncnn -B build-sdk/ncnn -G Ninja -DNCNN_VULKAN=ON -DNCNN_SIMPLEVK=OFF -DNCNN_SYSTEM_GLSLANG=ON \
	-DGLSLANG_TARGET_DIR=/usr/lib/aarch64-linux-gnu/cmake/glslang -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=OFF \
	-DNCNN_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/src/third_party/ncnn-install >/dev/null
cmake --build build-sdk/ncnn
cmake --install build-sdk/ncnn >/dev/null
[ -x "$CARGO_HOME/bin/cargo" ] || { chmod +x build-sdk/rust/rustup-init
	build-sdk/rust/rustup-init -y -q --no-modify-path --profile minimal --default-toolchain none; }
export PATH="$CARGO_HOME/bin:$PATH"
cd packages/roommate-radar
rustup -q toolchain install
cargo build --release
LD_LIBRARY_PATH=/src/third_party/openvr/bin/linuxarm64 cargo test --release -q
d=/src/dist/roommate-radar
rm -rf $d
mkdir -p $d/bin $d/driver/bin/linuxarm64 $d/models
cp /src/third_party/fonts/NotoSans-Medium.ttf $d/
cp /src/models/*_gray.ncnn.* $d/models/
cp $CARGO_TARGET_DIR/release/roommate-radar /src/third_party/openvr/bin/linuxarm64/libopenvr_api.so $d/bin/
cp driver.vrdrivermanifest $d/driver/
cp $CARGO_TARGET_DIR/release/libdriver_roommate_radar.so $d/driver/bin/linuxarm64/driver_roommate_radar.so'
