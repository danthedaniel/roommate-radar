# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: build deploy deploy-all deploy-mirror logs stream cams ssh

# Build roommate-radar on the Mac (Steam Runtime SDK, docker) into dist/roommate-radar, as released
build:
	packages/roommate-radar/build.sh

# Developer build (YOLO time in the status, confidence on the labels) of roommate-radar, installed; pairs the ssh key first
deploy:
	packages/roommate-radar/build.sh --developer
	cd ansible && ansible-playbook deploy.yml --tags roommate-radar

# Everything: pair the ssh key, then packages/{mirror,roommate-radar}/deploy.yml (roommate-radar's developer build)
deploy-all:
	packages/roommate-radar/build.sh --developer
	cd ansible && ansible-playbook deploy.yml

# The mirrorserver service only
deploy-mirror:
	cd ansible && ansible-playbook deploy.yml --tags mirror

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
