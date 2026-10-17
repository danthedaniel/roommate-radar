#!/bin/sh
# Live stream the headset view (SteamVR's /dev/video99 mirror, 1080p30) to this machine.
# Non-disruptive: SteamVR keeps running. With passthrough on, this is the camera view.
# Usage: ./mirror.sh            -> plays in ffplay
#        ./mirror.sh > out.ts   -> saves the H.264 MPEG-TS stream
host=${FRAME_HOST:-steamos@frame.local}
# libx264: the hardware encoder (v4l2h264enc / h264_v4l2m2m) fails on this input
remote='ffmpeg -hide_banner -loglevel error -f v4l2 -i /dev/video99 -pix_fmt yuv420p \
  -c:v libx264 -preset ultrafast -tune zerolatency -b:v 8M -g 30 -f mpegts -'

if [ -t 1 ]; then
  ssh "$host" "$remote" | ffplay -loglevel error -fflags nobuffer -flags low_delay -framedrop -window_title "Steam Frame" -
else
  ssh "$host" "$remote"
fi
