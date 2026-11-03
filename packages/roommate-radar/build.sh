#!/bin/sh
# roommate-radar's build, the way a Store depot is built: in the Steam Runtime 4 arm64 SDK. Run by make build, or
# directly; deploy.yml installs the result.
# All on the Mac (arm64, so the SDK image runs natively in Docker Desktop's VM): the inputs (ncnn submodule, models,
# int8 calibration images, OpenVR SDK pieces, font, rustup), then ncnn (into third_party/ncnn-install, where
# roommate-radar's build.rs finds it), the mixed-precision models (ncnn's quantize tools) and roommate-radar (cargo, the
# toolchain in rust-toolchain.toml) built and tested with docker in the SDK image (build-sdk/, incremental), and laid
# out in dist/roommate-radar as Steam installs it:
#   NotoSans-Medium.ttf models/  bin/{roommate-radar,libopenvr_api.so}
#   driver/{driver.vrdrivermanifest,bin/linuxarm64/driver_roommate_radar.so}
# build.sh --check (CI): build and test only, without the models and dist/.
# build.sh --developer (make deploy): with the developer feature (YOLO time in the tab's status, confidence on the labels).
set -e
cd "$(dirname "$0")/../.."
check=
features=
[ "$1" = --check ] && check=1
[ "$1" = --developer ] && features=developer

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

# ncnn's build is skipped when third_party/ncnn-install was built from this commit of it, without local changes
ncnn=$(git -C third_party/ncnn describe --always --dirty --abbrev=40)
case $ncnn in *-dirty) ncnn= ;; esac

# Exported on the Mac (ultralytics + torch don't run on the Frame), S x 2S input, adapted with prep_model.py
[ "$check" ] || get https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-grayscale.pt \
	models/yolo11n-grayscale.pt b81f01d60918e3194df21c305d9bd2275ab33cfddc79b26ca145a9190d1932f5
[ "$check" ] || for s in 320 480 640; do
	[ "models/yolo11n-grayscale_${s}_rr.ncnn.param" -nt packages/roommate-radar/prep_model.py ] && continue
	(
		cd models
		if [ ! -f "yolo11n-grayscale_$s.ncnn.param" ]; then
			uvx --from ultralytics --with ncnn --with pnnx yolo export model=yolo11n-grayscale.pt format=ncnn imgsz=$((2 * s)),$s
			mv yolo11n-grayscale_ncnn_model/model.ncnn.param "yolo11n-grayscale_$s.ncnn.param"
			mv yolo11n-grayscale_ncnn_model/model.ncnn.bin "yolo11n-grayscale_$s.ncnn.bin"
			rm -r yolo11n-grayscale_ncnn_model
		fi
		python3 ../packages/roommate-radar/prep_model.py "yolo11n-grayscale_$s.ncnn"
	)
done

# int8 calibration input (calib_prep.py): calib_images.txt's COCO val2017 images (people, cats, dogs) as the app's input
if [ ! "$check" ]; then
	while read -r f h; do get "http://images.cocodataset.org/val2017/$f" "models/coco/$f" "$h"; done \
		<packages/roommate-radar/calib_images.txt
	[ models/calib/list.txt -nt packages/roommate-radar/calib_prep.py ] &&
		[ models/calib/list.txt -nt packages/roommate-radar/calib_images.txt ] ||
		uvx --with pillow python packages/roommate-radar/calib_prep.py models/coco models/calib
fi

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
	-e CARGO_TARGET_DIR=/src/build-sdk/roommate-radar -e "ncnn=$ncnn" -e "check=$check" -e "features=$features" \
	registry.gitlab.steamos.cloud/steamrt/steamrt4/sdk/arm64:latest sh -ec '
built=third_party/ncnn-install/commit
if [ -z "$ncnn" ] || [ "$(cat $built 2>/dev/null)" != "$ncnn" ]; then
	rm -f $built
	cmake -S third_party/ncnn -B build-sdk/ncnn -G Ninja -DNCNN_VULKAN=ON -DNCNN_SIMPLEVK=OFF -DNCNN_SYSTEM_GLSLANG=ON \
		-DGLSLANG_TARGET_DIR=/usr/lib/aarch64-linux-gnu/cmake/glslang -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=ON \
		-DNCNN_SIMPLEOCV=ON -DNCNN_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/src/third_party/ncnn-install >/dev/null
	cmake --build build-sdk/ncnn
	cmake --install build-sdk/ncnn >/dev/null
	[ -z "$ncnn" ] || echo "$ncnn" > $built
fi
# The mixed-precision models: int8 on int8_convs.txt'"'"'s convs for each size (faster in int8 on the Frame), scales
# calibrated by ncnn2table (percentile; kl loses far more accuracy) on models/calib, the rest fp16
q=/src/build-sdk/ncnn/tools/quantize
[ -n "$check" ] || for s in 320 480 640; do
	m=/src/models/yolo11n-grayscale_$s
	for dep in ${m}_rr.ncnn.param models/calib/list.txt packages/roommate-radar/int8_convs.txt $q/ncnn2int8; do
		[ ${m}_mixed.ncnn.param -nt $dep ] || { rm -f ${m}_mixed.ncnn.param; break; }
	done
	[ -f ${m}_mixed.ncnn.param ] && continue
	(cd models/calib && $q/ncnn2table ${m}_rr.ncnn.param ${m}_rr.ncnn.bin list.txt $m.table "mean=[0]" "norm=[1]" \
		"shape=[$s,$((2 * s)),1]" pixel=GRAY method=percentile thread=8 >/dev/null 2>&1) ||
		{ echo "ncnn2table $s failed" >&2; exit 1; }
	for c in $(sed -n "s/^$s //p" packages/roommate-radar/int8_convs.txt); do
		grep -E "^${c}(_param_0)? " $m.table
	done >$m.int8.table
	$q/ncnn2int8 ${m}_rr.ncnn.param ${m}_rr.ncnn.bin ${m}_mixed.ncnn.param ${m}_mixed.ncnn.bin $m.int8.table \
		>/dev/null 2>&1 || { echo "ncnn2int8 $s failed" >&2; exit 1; }
	echo "quantized $s: $(grep -c " 8=" ${m}_mixed.ncnn.param) convs int8"
done
[ -x "$CARGO_HOME/bin/cargo" ] || { chmod +x build-sdk/rust/rustup-init
	build-sdk/rust/rustup-init -y -q --no-modify-path --profile minimal --default-toolchain none; }
export PATH="$CARGO_HOME/bin:$PATH"
cd packages/roommate-radar
rustup -q toolchain install
cargo build --release --features "$features"
LD_LIBRARY_PATH=/src/third_party/openvr/bin/linuxarm64 cargo test --release -q --features "$features"
[ -z "$check" ] || exit 0
d=/src/dist/roommate-radar
rm -rf $d
mkdir -p $d/bin $d/driver/bin/linuxarm64 $d/models
cp /src/third_party/fonts/NotoSans-Medium.ttf $d/
cp /src/models/*_mixed.ncnn.* $d/models/
cp $CARGO_TARGET_DIR/release/roommate-radar /src/third_party/openvr/bin/linuxarm64/libopenvr_api.so $d/bin/
cp driver.vrdrivermanifest $d/driver/
cp $CARGO_TARGET_DIR/release/libdriver_roommate_radar.so $d/driver/bin/linuxarm64/driver_roommate_radar.so'
