# Run from the Mac. See README for what each piece does.
FRAME ?= steamos@frame.local

.PHONY: deploy build reload launch stop logs stream mirror cams ssh sysroot

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

# Frame headers into .sysroot/ so clangd on the Mac resolves includes (yoloxr/compile_flags.txt). Rerun after make build
sysroot:
	mkdir -p .sysroot/usr/lib/gcc .sysroot/gen
	rsync -a $(FRAME):/usr/include .sysroot/usr/
	rsync -a --include='*/' --include='include/***' --include='crt*.o' --exclude='*' $(FRAME):/usr/lib/gcc/aarch64-unknown-linux-gnu .sysroot/usr/lib/gcc/
	rsync -a $(FRAME):steamframe/third_party/ncnn-install/include/ncnn/ .sysroot/ncnn/
	rsync -a $(FRAME):/opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers/ .sysroot/openvr/
	rsync -a '$(FRAME):steamframe/yoloxr/build/*.inc' .sysroot/gen/
