# Working pipeline: laptop ↔ Go2

## A. Hardware bring-up (one-time per session)

1. **Power on the Go2.** Wait for it to finish its self-stand routine (it'll be standing on all four legs, looking around).
2. **Plug Ethernet** from the Go2's network port to the USB-Ethernet adapter on your laptop.
3. **Verify the link**:
   ```bash
   ip addr show enx50a03003bae4 | grep "inet "
   ```
   Should show `192.168.123.99/24`. If not, re-run the static IP command:
   ```bash
   sudo nmcli connection up go2-wired
   ```
4. **Ping the robot**:
   ```bash
   ping -c 2 192.168.123.161
   ```
   Replies = good. No replies = check cable / robot power.

## B. Open three terminals, source the env in each

In **every** new terminal, first thing:
```bash
source ~/hack_2026/unitree_ros2/setup_robot.sh
```
And in any terminal that runs the C++ controller binary, also:
```bash
GO2BIN=~/hack_2026/unitree_ros2/example/install/unitree_ros2_example/bin
```

## C. Sanity checks (terminal 1, before starting anything)

```bash
ros2 topic list | head -20             # should show /api/sport/request, /lf/sportmodestate, etc.
ros2 topic hz /lf/sportmodestate       # ~50 Hz incoming = robot reachable & alive
ros2 topic info /follow/target         # Publisher count = 0 → safe to start controller
```
If `Publisher count >= 1`, kill stale publishers before continuing:
```bash
pkill -f go2_perception
pkill -f "ros2 topic pub.*follow"
```

## D. Start the controller (terminal 1)

```bash
$GO2BIN/go2_follow_controller
```
The controller starts ticking at 10 Hz. With nothing publishing `/follow/target` it sends `StopMove` every 100 ms — the robot stays still.

## E. Verify what the controller is sending (terminal 2)

```bash
ros2 topic echo /api/sport/request
```
You should see steady `Request` messages. While idle, the `parameter` field will encode `StopMove`. This window stays open as your read-out throughout the session.

## F. Choose a perception source (terminal 3)

### F.1 — Test mode: fake target by hand
```bash
# centered, 3 m away → robot walks forward
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 3.0}"
```
Other useful one-liners:
```bash
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.3, y: 1.5}"   # turn right in place
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 1.5}"   # idle (at follow distance)
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 0.5}"   # back up (too close)
```
**Ctrl-C the publisher to stop motion.** Watchdog kicks in after 0.5 s and controller goes back to `StopMove`.

### F.2 — Real mode: live perception from the Go2's front camera
```bash
ros2 run unitree_ros2_example go2_perception
```
This subscribes to `Go2FrontVideoData` (default topic `/frontvideostream`), runs YOLO, and publishes `/follow/target` based on whoever the camera sees. **Robot will follow you in real time.** If `/frontvideostream` is the wrong name, find it with `ros2 topic list | grep -i video` and override:
```bash
ros2 run unitree_ros2_example go2_perception --ros-args -p camera_topic:=<actual>
```

## G. Stop everything

In each terminal: **Ctrl-C** (or **Ctrl-\\** if Ctrl-C is being eaten by log spam).
- Stopping the publisher (F) → controller goes idle within 0.5 s.
- Stopping the controller (D) → no more commands, robot does whatever its high-level controller decides (typically holds still).
- Closing terminals does NOT power down the robot — use the remote/app/power button for that.

## Troubleshooting

### `ros2 topic list` hangs even though `ping` works

The `ros2` CLI talks to a long-lived `_ros2_daemon` process that caches discovery state. If that daemon was started under a *different* RMW (e.g. fastrtps default before we sourced cyclonedds), every `ros2 topic *` call hangs because the daemon and your shell disagree about the bus.

```bash
pkill -9 -f _ros2_daemon          # nuke the stale daemon
pgrep -af ros2_daemon              # confirm it's gone (no output)
ros2 topic list --no-daemon        # discover in-process, bypasses any daemon
```

After it works once, subsequent calls auto-start a fresh daemon under the right RMW and are fast again.

### Adapter disconnect / reconnect mid-session

If the USB-Ethernet was unplugged and plugged back, do all of:
```bash
sudo nmcli connection up go2-wired      # re-apply static IP
ping -c 1 192.168.123.161               # confirm wire
pkill -9 -f _ros2_daemon                # nuke the stale daemon (it cached the old peer state)
```
Then re-source `setup_robot.sh` in any running terminals (they keep the env, but the daemon they spawn will be fresh).

## Full reference card

| What | Where | Command |
|---|---|---|
| Source env | every terminal | `source ~/hack_2026/unitree_ros2/setup_robot.sh` |
| Verify link | once per session | `ping -c 2 192.168.123.161` |
| Verify ROS bridge | once per session | `ros2 topic hz /lf/sportmodestate` |
| Check no stale publishers | before controller | `ros2 topic info /follow/target` |
| Run controller | T1 | `$GO2BIN/go2_follow_controller` |
| Watch outgoing | T2 | `ros2 topic echo /api/sport/request` |
| Fake target | T3 (test) | `ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 3.0}"` |
| Live perception | T3 (real) | `ros2 run unitree_ros2_example go2_perception` |
