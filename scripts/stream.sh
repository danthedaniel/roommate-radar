#!/bin/sh
# shellcheck disable=SC2029  # $remote is built here on purpose, then run by ssh on the Frame
# Live stream all 4 raw tracking cameras to this machine. SteamVR keeps running:
# rawcam.py reads the frames out of XRService's capture buffers (needs NOPASSWD sudo).
# Usage: scripts/stream.sh [rawcam args, e.g. --fps 15 --exposure controller]   -> ffplay
#        scripts/stream.sh > out.ts                                              -> save H.264 MPEG-TS
cd "$(dirname "$0")" || exit
host=${FRAME_HOST:-steamos@frame.local}
fps=30
remote="sudo -n python3 - --fps $fps $* | ffmpeg -hide_banner -loglevel error \
  -f rawvideo -pix_fmt gray -s 2112x1504 -framerate $fps -i - -pix_fmt yuv420p \
  -c:v libx264 -preset ultrafast -tune zerolatency -b:v 10M -g $fps -f mpegts -"

if [ -t 1 ]; then
  ssh "$host" "$remote" < rawcam.py | ffplay -loglevel error -fflags nobuffer -flags low_delay -framedrop -window_title "Steam Frame cameras" -
else
  ssh "$host" "$remote" < rawcam.py
fi
