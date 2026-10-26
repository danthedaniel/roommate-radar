# mirror: headset view over UDP

`mirrorserver.py` runs on the Frame as a systemd user service (`mirrorserver`, UDP 5600, H.264 MPEG-TS, libx264
only while a client sends keepalives), independent of roommate-radar. Logs: `journalctl --user -u mirrorserver -f`.
Deploy: `make deploy-mirror` (ships `mirrorserver.py` to `~/steamframe/`, installs and enables the service).

```sh
packages/mirror/udpmirror.py                         # headset view (/dev/video99) from the mirrorserver service; black/~1 fps when not worn
packages/mirror/udpmirror.py > out.ts                # same, saved
packages/mirror/udpmirror.py --record out.mp4        # watch and save to MP4 (stream copied, not re-encoded)
```

`make stream` runs the first one.

## Dead ends

- Hardware H.264 encoder (`v4l2h264enc`, `h264_v4l2m2m`) fails here; mirrorserver uses libx264.
