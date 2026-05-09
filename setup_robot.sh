#!/bin/bash
# Source this in every terminal that needs to talk to the Go2:
#   source ~/hack_2026/unitree_ros2/setup_robot.sh
#
# Switches the ROS2 middleware to CycloneDDS and pins it to the USB-Ethernet
# adapter that's wired to the robot (192.168.123.99/24).
source /opt/ros/humble/setup.bash
source "$HOME/hack_2026/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/hack_2026/unitree_ros2/example/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                          <NetworkInterface name="enx50a03003bae4" priority="default" multicast="default" />
                       </Interfaces></General></Domain></CycloneDDS>'
echo "[robot] env ready (humble, cyclonedds, iface=enx50a03003bae4)"
