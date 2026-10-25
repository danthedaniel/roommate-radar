# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: deploy deploy-mirror deploy-guardian logs stream cams ssh sysroot

# Everything: pair the ssh key, then mirror/deploy.yml and guardian/deploy.yml (via ansible/deploy.yml)
deploy:
	cd ansible && ansible-playbook deploy.yml

# One project: mirrorserver service / ship and build guardian
deploy-mirror deploy-guardian: deploy-%:
	cd ansible && ansible-playbook deploy.yml --tags $*

# guardian output (started by SteamVR)
logs:
	ssh $(FRAME) 'tail -f /tmp/guardian.log'

# Headset view over UDP (mirrorserver service, always on)
stream:
	mirror/udpmirror.py

# Raw tracking cameras
cams:
	scripts/stream.sh

ssh:
	ssh $(FRAME)

# Frame headers into .sysroot/ so clangd on the Mac resolves includes (guardian/compile_flags.txt). Rerun after make deploy-guardian
sysroot:
	mkdir -p .sysroot/usr/lib/gcc
	rsync -a $(FRAME):/usr/include .sysroot/usr/
	rsync -a --include='*/' --include='include/***' --include='crt*.o' --exclude='*' $(FRAME):/usr/lib/gcc/aarch64-unknown-linux-gnu .sysroot/usr/lib/gcc/
	rsync -a $(FRAME):steamframe/third_party/ncnn-install/include/ncnn/ .sysroot/ncnn/
	rsync -a $(FRAME):/opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers/ .sysroot/openvr/
