#!/bin/bash
# Hackathon setup — local smoke testing on humble at this repo path.
# Source this in every terminal:  source ~/hack_2026/unitree_ros2/setup_hackathon.sh
source /opt/ros/humble/setup.bash
if [ -f "$HOME/hack_2026/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  source "$HOME/hack_2026/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi
if [ -f "$HOME/hack_2026/unitree_ros2/example/install/setup.bash" ]; then
  source "$HOME/hack_2026/unitree_ros2/example/install/setup.bash"
fi
# For local smoke tests we stay on the default fastrtps RMW (no robot needed).
# When you actually connect to the Go2, switch to cyclonedds and set the iface:
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="<your-iface>" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
echo "[hackathon] env ready (humble, fastrtps RMW)"
