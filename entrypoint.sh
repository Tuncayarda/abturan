#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ ! -f install/setup.bash ]; then
  echo "[entrypoint] install/ not found — building workspace with colcon..."
  colcon build --symlink-install
fi

source install/setup.bash

# When the default CMD (ros2 launch up_bringup bringup.launch.py) is used,
# append any optional overrides from the environment — this is how start.sh
# forwards [port w h fps bitrate] plus the STM32 serial port. No IP is needed
# for the camera (SRT listener).
if [ "$1" = "ros2" ] && [ "$2" = "launch" ]; then
  launch_args=()
  [ -n "${CAM_PORT:-}" ]    && launch_args+=("port:=${CAM_PORT}")
  [ -n "${CAM_WIDTH:-}" ]   && launch_args+=("width:=${CAM_WIDTH}")
  [ -n "${CAM_HEIGHT:-}" ]  && launch_args+=("height:=${CAM_HEIGHT}")
  [ -n "${CAM_FPS:-}" ]     && launch_args+=("fps:=${CAM_FPS}")
  [ -n "${CAM_BITRATE:-}" ] && launch_args+=("bitrate:=${CAM_BITRATE}")
  [ -n "${STM32_PORT:-}" ]  && launch_args+=("stm32_port:=${STM32_PORT}")
  [ -n "${STM32_BAUD:-}" ]  && launch_args+=("stm32_baud:=${STM32_BAUD}")
  exec "$@" "${launch_args[@]}"
fi

exec "$@"
