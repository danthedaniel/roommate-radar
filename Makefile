# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: deploy build reload launch stop logs stream mirror cams ssh

# Everything: mirrorserver service, yoloxr (ship, build, Steam library entry). See ansible/site.yml
deploy:
	cd ansible && ansible-playbook site.yml

# Ship and build yoloxr only
build:
	cd ansible && ansible-playbook site.yml --tags yoloxr

# Rebuild and restart yoloxr inside the running run.sh
reload: build
	ssh $(FRAME) 'pkill -USR1 -x run.sh'

# Start yoloxr the way the Steam library entry does
launch:
	ssh $(FRAME) 'python3 ~/steamos-devkit/client/devkit-utils/steam-devkit-rpc run-game gameid=yoloxr'

stop:
	ssh $(FRAME) 'pkill -TERM -x run.sh'

# yoloxr output when launched from Steam
logs:
	ssh $(FRAME) 'tail -f ~/.local/share/Steam/logs/steam_output.log'

# Headset view over UDP (mirrorserver service, always on)
stream:
	./udpmirror.py

# Headset view over ssh (no yoloxr needed)
mirror:
	./mirror.sh

# Raw tracking cameras
cams:
	./stream.sh

ssh:
	ssh $(FRAME)
