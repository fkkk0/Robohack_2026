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
# AllowMulticast=spdp mirrors the robot's own cyclonedds.xml: use multicast
# only for participant discovery, switch to unicast for actual data.
# MinimumSocketReceiveBufferSize asks the kernel for a 10 MB UDP receive
# buffer so large multi-fragment messages (H.264 I-frames in /frontvideostream)
# don't get dropped under burst load. Pair this with:
#   sudo sysctl -w net.core.rmem_max=10485760
# (the kernel won't grant more than rmem_max, regardless of what we request).
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                          <NetworkInterface name="enx50a03003bae4" priority="default" multicast="default" />
                       </Interfaces><AllowMulticast>spdp</AllowMulticast></General><Internal><SocketReceiveBufferSize min="10MB"/></Internal></Domain></CycloneDDS>'
echo "[robot] env ready (humble, cyclonedds, iface=enx50a03003bae4)"
