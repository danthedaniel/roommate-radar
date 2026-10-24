# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: deploy build logs stream mirror cams ssh sysroot

# Everything: mirrorserver service, guardian (ship, build). See ansible/site.yml
deploy:
	cd ansible && ansible-playbook site.yml

# Ship and build guardian only
build:
	cd ansible && ansible-playbook site.yml --tags guardian

# guardian output (started by SteamVR)
logs:
	ssh $(FRAME) 'tail -f /tmp/guardian.log'

# Headset view over UDP (mirrorserver service, always on)
stream:
	scripts/udpmirror.py

# Headset view over ssh
mirror:
	scripts/mirror.sh

# Raw tracking cameras
cams:
	scripts/stream.sh

ssh:
	ssh $(FRAME)

# Frame headers into .sysroot/ so clangd on the Mac resolves includes (guardian/compile_flags.txt). Rerun after make build
sysroot:
	mkdir -p .sysroot/usr/lib/gcc
	rsync -a $(FRAME):/usr/include .sysroot/usr/
	rsync -a --include='*/' --include='include/***' --include='crt*.o' --exclude='*' $(FRAME):/usr/lib/gcc/aarch64-unknown-linux-gnu .sysroot/usr/lib/gcc/
	rsync -a $(FRAME):steamframe/third_party/ncnn-install/include/ncnn/ .sysroot/ncnn/
	rsync -a $(FRAME):/opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers/ .sysroot/openvr/
