# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: build deploy deploy-mirror deploy-guardian logs stream cams ssh sysroot

# Build guardian on the Mac (Steam Runtime SDK, docker) into dist/guardian (ansible/build.yml)
build:
	cd ansible && ansible-playbook build.yml

# Everything: pair the ssh key, then mirror/deploy.yml and guardian/deploy.yml (installs dist/guardian; make build first)
deploy:
	cd ansible && ansible-playbook deploy.yml

# One project: mirrorserver service / install guardian
deploy-mirror deploy-guardian: deploy-%:
	cd ansible && ansible-playbook deploy.yml --tags $*

# guardian output (started through Steam)
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

# Frame headers into .sysroot/ so clangd on the Mac resolves includes (guardian/compile_flags.txt; ncnn and OpenVR
# come from make build, in third_party/)
sysroot:
	mkdir -p .sysroot/usr/lib/gcc
	rsync -a $(FRAME):/usr/include .sysroot/usr/
	rsync -a --include='*/' --include='include/***' --include='crt*.o' --exclude='*' $(FRAME):/usr/lib/gcc/aarch64-unknown-linux-gnu .sysroot/usr/lib/gcc/
