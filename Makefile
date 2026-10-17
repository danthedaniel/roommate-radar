# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: deploy build reload launch stop logs view mirror cams ssh

# Copy yoloxr/ to ~/steamframe on the Frame
deploy:
	COPYFILE_DISABLE=1 tar cf - --no-xattrs --exclude build yoloxr | ssh $(FRAME) 'cd ~/steamframe && tar xf -'

build: deploy
	ssh $(FRAME) 'cd ~/steamframe/yoloxr && cmake -S . -B build -G Ninja >/dev/null && cmake --build build'

# Rebuild and restart yoloxr inside the running run.sh
reload: build
	ssh $(FRAME) 'pkill -USR1 -x run.sh'

# Start yoloxr the way the Steam library entry does
launch:
	ssh $(FRAME) 'python3 ~/devkit-utils/steam-devkit-rpc run-game gameid=yoloxr'

stop:
	ssh $(FRAME) 'pkill -TERM -x run.sh'

# yoloxr output when launched from Steam
logs:
	ssh $(FRAME) 'tail -f ~/.local/share/Steam/logs/steam_output.log'

# Headset view over UDP (yoloxr must be running)
view:
	./udpmirror.py

# Headset view over ssh (no yoloxr needed)
mirror:
	./mirror.sh

# Raw tracking cameras
cams:
	./stream.sh

ssh:
	ssh $(FRAME)
