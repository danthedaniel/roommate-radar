# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: build deploy deploy-all deploy-mirror logs stream cams ssh

# Build roommate-radar on the Mac (Steam Runtime SDK, docker) into dist/roommate-radar
build:
	packages/roommate-radar/build.sh

# Install roommate-radar (dist/roommate-radar; make build first), pairing the ssh key first
deploy:
	cd ansible && ansible-playbook deploy.yml --tags roommate-radar

# Everything: pair the ssh key, then packages/{mirror,roommate-radar}/deploy.yml
deploy-all:
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
