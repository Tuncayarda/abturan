#!/usr/bin/env bash
# Single entry point for the robot bringup. Starts the host-side camera streamer
# (data plane: shares the video via SRT in LISTENER mode — no destination IP, the
# single client connects to srt://<this_pi_ip>:PORT) and the ROS 2 control plane
# container (rosbridge, camera control, IMU, joystick/thruster/STM32 link).
#
# Actuators live on the STM32 (6 ESC PWM, stepper, LED) and are reached over
# UART; the IMU hangs on the Pi's I2C bus. Both need one-time host setup —
# run ./setup_hardware.sh once if /dev/ttyAMA0 or /dev/i2c-1 is missing.
#
# Usage:
#   ./start.sh [PORT WIDTH HEIGHT FPS BITRATE]
#
# Env:
#   STM32_PORT (default /dev/ttyAMA0), STM32_BAUD (default 115200)
#
# Examples:
#   ./start.sh
#   ./start.sh 9003 1280 720 30 2000000
#   STM32_PORT=/dev/ttyUSB0 ./start.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTRL_DIR="${SCRIPT_DIR}/cam_ctrl"
mkdir -p "${CTRL_DIR}"

# Use plain `docker` if the current session already has the docker group
# active; otherwise fall back to `sg docker` (needed until the next login
# after `usermod -aG docker`).
if docker info >/dev/null 2>&1; then
  compose() { docker compose "$@"; }
else
  compose() { sg docker -c "docker compose $*"; }
fi

export CAM_PORT="${1:-9003}"
export CAM_WIDTH="${2:-}"
export CAM_HEIGHT="${3:-}"
export CAM_FPS="${4:-}"
export CAM_BITRATE="${5:-}"

export STM32_PORT="${STM32_PORT:-/dev/ttyAMA0}"
export STM32_BAUD="${STM32_BAUD:-115200}"

PI_IP="$(hostname -I | awk '{print $1}')"
echo "[start] Video (SRT listener): srt://${PI_IP}:${CAM_PORT}  (client buna bağlanır)"
echo "[start] rosbridge           : ws://${PI_IP}:9090"
echo "[start] STM32 UART          : ${STM32_PORT} @ ${STM32_BAUD}"

# Eksik aygitlar sessizce yayin/kontrol kaybina donusmesin diye uyariyoruz;
# ikisi de olmadan sistem yine kalkiyor (koprular yeniden denemeye devam eder).
[ -e "${STM32_PORT}" ] || echo "[start] UYARI: ${STM32_PORT} yok — ./setup_hardware.sh calistirdin mi? (STM32 komutlari gitmeyecek)"
[ -e /dev/i2c-1 ]      || echo "[start] UYARI: /dev/i2c-1 yok — IMU okunamayacak (./setup_hardware.sh)"
[ -n "$CAM_WIDTH" ]   && echo "[start] width   : ${CAM_WIDTH}"
[ -n "$CAM_HEIGHT" ]  && echo "[start] height  : ${CAM_HEIGHT}"
[ -n "$CAM_FPS" ]     && echo "[start] fps     : ${CAM_FPS}"
[ -n "$CAM_BITRATE" ] && echo "[start] bitrate : ${CAM_BITRATE}"
echo

cleanup() {
  echo
  echo "[start] stopping..."
  [ -n "${STREAMER_PID:-}" ] && kill "${STREAMER_PID}" 2>/dev/null || true
  (cd "${SCRIPT_DIR}" && compose down) || true
}
trap cleanup EXIT INT TERM

echo "[start] launching host camera streamer (rpi_cam_streamer.py)..."
python3 "${SCRIPT_DIR}/src/rpi_cam_bridge/scripts/rpi_cam_streamer.py" \
  --ctrl-dir "${CTRL_DIR}" &
STREAMER_PID=$!

echo "[start] launching ROS 2 control plane (docker compose)..."
cd "${SCRIPT_DIR}"
compose up --build
