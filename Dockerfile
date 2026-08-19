FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    python3-numpy \
    python3-serial \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "up_bringup", "bringup.launch.py"]
