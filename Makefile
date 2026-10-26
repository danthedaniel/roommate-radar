# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: build deploy deploy-mirror deploy-roommate-radar logs stream cams ssh

# Build roommate-radar on the Mac (Steam Runtime SDK, docker) into dist/roommate-radar
build:
	packages/roommate-radar/build.sh

# Everything: pair the ssh key, then packages/{mirror,roommate-radar}/deploy.yml (installs dist/roommate-radar; make build first)
deploy:
	cd ansible && ansible-playbook deploy.yml

# One project: mirrorserver service / install roommate-radar
deploy-mirror deploy-roommate-radar: deploy-%:
	cd ansible && ansible-playbook deploy.yml --tags $*

# roommate-radar output (started through Steam)
logs:
	ssh $(FRAME) 'tail -f /tmp/roommate-radar.log'

# Headset view over UDP (mirrorserver service, always on)
stream:
	packages/mirror/udpmirror.py

# Raw tracking cameras
cams:
	scripts/stream.sh

ssh:
	ssh $(FRAME)
