#!/usr/bin/env bash
# Open an interactive shell inside the running ROS2 Humble container with the environment sourced.
docker exec -it ros2_humble bash -c "source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; exec bash"
