#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Kullanım:"
  echo "  ./rpi_stream_to_mac.sh MAC_IP [PORT] [WIDTH] [HEIGHT] [FPS] [BITRATE]"
  echo
  echo "Örnek:"
  echo "  ./rpi_stream_to_mac.sh 172.20.10.6"
  echo "  ./rpi_stream_to_mac.sh 172.20.10.6 5000 1280 720 30 2000000"
  exit 1
fi

MAC_IP="$1"
PORT="${2:-9003}"
WIDTH="${3:-640}"
HEIGHT="${4:-480}"
FPS="${5:-30}"
BITRATE="${6:-1000000}"

echo "[RPi] Kamera yayını başlatılıyor..."
echo "[RPi] Hedef Mac IP : ${MAC_IP}"
echo "[RPi] Port         : ${PORT}"
echo "[RPi] Çözünürlük   : ${WIDTH}x${HEIGHT}"
echo "[RPi] FPS          : ${FPS}"
echo "[RPi] Bitrate      : ${BITRATE}"
echo
echo "Durdurmak için CTRL+C"
echo

rpicam-vid -t 0 \
  --nopreview \
  --camera 0 \
  --width "${WIDTH}" --height "${HEIGHT}" --framerate "${FPS}" \
  --codec h264 --inline \
  --libav-format h264 \
  --bitrate "${BITRATE}" \
  -o - | gst-launch-1.0 -v \
  fdsrc \
  ! h264parse \
  ! rtph264pay config-interval=1 pt=96 \
  ! udpsink host="${MAC_IP}" port="${PORT}" sync=false async=false
