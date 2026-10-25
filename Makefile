# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: build deploy deploy-mirror deploy-guardian logs stream cams ssh

# Build guardian on the Mac (Steam Runtime SDK, docker) into dist/guardian
build:
	guardian/build.sh

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
