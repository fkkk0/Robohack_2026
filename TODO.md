# Follow-person on Go2 — implementation plan

The Go2 already exposes a front camera (`Go2FrontVideoData`, H.264 720p/360p/180p) and lidar (`/utlidar/cloud`) over DDS. The high-level locomotion API in `example/src/include/common/ros2_sport_client.h:144` gives `SportClient::Move(req, vx, vy, vyaw)` — published on `/api/sport/request` — which is exactly what a follow controller needs.

**Only the controller node exists in this repo. Perception nodes must be written.**

## Node 1 — Camera publisher (already running on the robot)

Nothing to write. The Go2's onboard process publishes `Go2FrontVideoData` (defined in `cyclonedds_ws/src/unitree/unitree_go/msg/Go2FrontVideoData.msg`). The perception node just subscribes.

If a USB camera is preferred instead, run a stock `v4l2_camera` or `image_publisher` node — not part of this repo.

## Node 2 — Perception node (NEW — to write)

New file, e.g. `example/src/src/go2/follow_perception.cpp`, registered in `example/src/CMakeLists.txt` next to the other `add_executable(...)` lines. Python (`rclpy`) is also a sane choice here — much less code for vision work.

- **Subscribes:** `Go2FrontVideoData` (topic name confirmable with `ros2 topic list` once connected — typically `/frontvideostream` on Go2)
- **Optional second subscribe:** `/utlidar/cloud` (`sensor_msgs/PointCloud2`) for depth fusion
- **Inside:** decode H.264 (FFmpeg / `libav`), run YOLO or MediaPipe person detector + a tracker (e.g. ByteTrack) for ID stability
- **Publishes:** small custom topic, e.g. `/follow/target` as `geometry_msgs/PointStamped` (bearing in pixels + estimated distance). Reuse `geometry_msgs` rather than defining a new `.msg`.

### Distance estimation

Two options:
- **Lidar fusion** — project `/utlidar/cloud` into the image, read depth at the person's bbox pixels. Most robust.
- **Monocular** — estimate from bbox height + known person height. Cheap, less reliable.

## Node 3 — Follow controller (MODIFY `example/src/src/go2/go2_sport_client.cpp`)

The existing file is ~90% of what's needed. Concrete edits:

- **Add a subscriber** alongside `suber_` (line 122) for `/follow/target` — call its callback `TargetHandler`.
- **Drop the `test_mode` switch** (lines 56–98). Replace with a single control-loop body:
  ```cpp
  vyaw = -kp_yaw  * bearing_error;          // px from frame center
  vx   =  kp_dist * (distance - target_distance);
  sport_client_.Move(req_, vx, 0.0, vyaw);
  req_puber_->publish(req_);                // see note below
  ```
- **Bug to fix in the template:** the example builds `req_` but never publishes it. Add a `rclcpp::Publisher<unitree_api::msg::Request>` on `/api/sport/request` (topic shown in `README.md:297`) and call `publish(req_)` after each `sport_client_.Move(...)`. `SportClient` only fills the message; it doesn't transmit.
- **Safety:** if `now() - last_target_stamp > 0.5s` → call `sport_client_.StopMove(req_)` instead of `Move` (API at `ros2_sport_client.h:113`).
- **Keep:** the `HighStateHandler` on `lf/sportmodestate` (line 42) — useful for logging/safety (e.g., abort if `state_.mode` becomes `lieDown` / `damping`).

The `Move` API is declared at `ros2_sport_client.h:144` and demonstrated at `go2_sport_client.cpp:64,91`.

## Wiring summary

```
[Go2 onboard]                 [Perception node (new)]              [Modified go2_sport_client]
 publishes:                    subscribes Go2FrontVideoData  →
   /frontvideostream           subscribes /utlidar/cloud
   /utlidar/cloud              publishes  /follow/target      →    subscribes /follow/target
   lf/sportmodestate                                               subscribes lf/sportmodestate
                                                                   publishes  /api/sport/request  → [Go2 onboard]
```

All three live on the same DDS domain — no bridging or launch-file orchestration beyond `source ~/unitree_ros2/setup.sh` in each terminal.

## Where to run perception (key tradeoff)

- **Onboard the Go2's Jetson** — self-contained, but limits you to small/quantized models.
- **On a laptop on the same DDS network** (set `CYCLONEDDS_URI` to the Ethernet interface, like `setup.sh` does) — bigger models, no code change. Topics look identical from either side.

## Suggested order of work

1. **Verify control path.** Modify `go2_sport_client.cpp` to add the `/api/sport/request` publisher and have it send `Move(0.2, 0, 0)` for 2 s then `StopMove`. Confirms the publish path works (the missing piece in the template).
2. **Bring up perception in Python**, publishing a fake `/follow/target` from a laptop. Tune `kp_yaw` / `kp_dist` and target distance against the fake target.
3. **Plug in the real detector** (YOLOv8 + ByteTrack), monocular distance first.
4. **Add lidar-fusion depth** for robustness.
5. **Safety pass:** target-loss timeout, max velocity clamps, abort on bad `sportmodestate.mode`.
