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


##### ------------------------

# control part

> **Correction to plan above:** `SportClient` already creates its own publisher on `/api/sport/request` in its constructor (`ros2_sport_client.h:71`), and `Move` / `StopMove` publish internally (`ros2_sport_client.cpp:18,55`). The earlier note about a "missing publisher bug" was wrong — the original `go2_sport_client.cpp` does send commands. The controller therefore does not need its own `req_puber_`; calling `sport_client_.Move(req_, vx, 0, vyaw)` is enough.

## Built: `example/src/src/go2/go2_follow_controller.cpp`

A new node — does NOT modify `go2_sport_client.cpp`. The original example stays as a reference.

### Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sub | `/follow/target` | `geometry_msgs/Vector3` | `x` = bearing (rad, +ve = person to the right), `y` = distance (m), `z` ignored |
| sub | `lf/sportmodestate` | `unitree_go/SportModeState` | only used for safety (`mode == 5` lieDown / `7` damping → stop) |
| pub | `/api/sport/request` | `unitree_api/Request` | sent indirectly via `SportClient::Move` / `StopMove` |

### Control loop (10 Hz)

```
if mode in {lieDown, damping}: StopMove; return
if no fresh /follow/target within 0.5s: StopMove; return

dist_err = distance - desired_distance
vx   = clamp(Kp_dist * dist_err, vx_min, vx_max)        if |dist_err| > deadband else 0
vyaw = clamp(-Kp_yaw  * bearing,  -vyaw_max, vyaw_max)  if |bearing|  > deadband else 0

if distance < min_safe_dist and vx > 0: vx = 0          # never push closer than min_safe_dist
if |bearing| > 0.4 rad:                 vx = min(vx, 0.1) # turn before walking

Move(vx, 0, vyaw)
```

### Constants (hardcoded for hackathon, tune in source)

| Name | Value | Meaning |
|---|---|---|
| `kDesiredDistance` | 1.5 m | follow distance |
| `kDistDeadband` | 0.15 m | ignore tiny distance errors |
| `kBearingDeadband` | 0.05 rad ≈ 3° | ignore tiny bearing errors |
| `kKpDist` | 0.8 | forward gain |
| `kKpYaw` | 1.5 | yaw gain |
| `kVxMax` / `kVxMin` | +0.6 / −0.3 m/s | forward clamp (asymmetric: cautious back-up) |
| `kVyawMax` | 1.2 rad/s | yaw clamp |
| `kMinSafeDist` | 0.6 m | hard floor — no forward motion below this |
| `kTargetTimeout` | 0.5 s | watchdog |

### Build wiring (already added)

- `example/src/CMakeLists.txt`: `geometry_msgs` added to `find_package` and `DEPENDENCY_LIST`; new `add_executable` / `target_compile_features` (cxx_std_20) / `ament_target_dependencies` / `install` for `go2_follow_controller`.
- `example/src/package.xml`: `<depend>geometry_msgs</depend>` added.

### Smoke test (no perception needed yet)

```bash
source ~/unitree_ros2/setup.sh
cd ~/hack_2026/unitree_ros2/example
colcon build --packages-select unitree_ros2_example
source install/setup.bash

##### new smoke test
# Terminal A — controller
source ~/hack_2026/unitree_ros2/setup_hackathon.sh
~/hack_2026/unitree_ros2/example/install/unitree_ros2_example/bin/go2_follow_controller

# Terminal B — fake the perception output (try each line one at a time)
source ~/hack_2026/unitree_ros2/setup_hackathon.sh
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 1.5}"   # idle (centered, at follow distance)
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.3, y: 1.5}"   # turn right
ros2 topic pub -r 10 /follow/target geometry_msgs/Vector3 "{x: 0.0, y: 3.0}"   # walk forward

# Terminal C — watch what the controller is sending to the (absent) robot
source ~/hack_2026/unitree_ros2/setup_hackathon.sh
ros2 topic echo /api/sport/request

--
# camera problem
 The camera problem in plain language

  How ROS2 sends messages

  When the Go2 publishes a camera frame, what travels over the wire
  is just a byte stream — say, 0x4F 0x82 0x1A 0x00 …. The bytes have
   no labels attached to them. To make sense of those bytes, the
  receiver needs to know the exact recipe — the schema — for how to
  chop them up:

  ▎ "First 8 bytes = a timestamp. Then 4 bytes = how long the next 
  ▎ array is, call that N. Then N bytes = the array data. Then 4 
  ▎ more bytes = the next array's length…"

  That recipe is the .msg file. Our copy says:

  uint64 time_frame       (8 bytes)
  uint8[] video720p       (4-byte length + data)
  uint8[] video360p       (4-byte length + data)
  uint8[] video180p       (4-byte length + data)

  The Go2 firmware has its own copy of this recipe baked in. As long
   as both sides have the same recipe, deserialization works. If
  they disagree by even one field, the receiver applies the wrong
  recipe to the bytes and gets garbage.

  What's actually happening on your robot

  We confirmed our recipe matches Unitree's official ROS2 GitHub
  repo — they're identical. But when we read one frame off the wire,
   the lengths came back nonsensical:

  video720p: length 360                      ← believable
  video360p: length 1,535,818,905 (1.5 GB)   ← physically impossible

  A 1.5 GB array can't exist in a single ROS message. So our
  deserializer didn't actually find a real "length 1.5 GB array" —
  it read random H.264 video bytes from the actual frame data and 
  interpreted them as a length number, because it had drifted
  off-position.

  Why it drifts off-position

  Imagine the recipe says "skip 8 bytes for the timestamp" but the
  firmware actually wrote 12 bytes (because in its recipe there's a
  4-byte frame_id field after the timestamp). The receiver skips 8,
  reads the next 4 as "length of video720p" — but those 4 bytes are
  actually the second half of the frame_id. Now the receiver thinks
  the video720p array starts where frame_id's last bytes ended.
  Everything is offset by 4 bytes from then on, and every subsequent
   length-and-data pair is computed from H.264 garbage.

  We don't know exactly what the firmware's recipe is — could be an
  extra field, a renamed field, a different type, fields in a
  different order. We just know it doesn't match ours.

  Why this happens with Unitree specifically

  Unitree maintains two separate SDKs for talking to the Go2:

  - unitree_sdk2 (C++) — what their official apps use
  - unitree_ros2 (this repo) — the ROS2 mirror

  They generate from independent IDL files. When Unitree updates the
   C++ SDK to add a field to Go2FrontVideoData, they sometimes ship
  that change to the firmware before updating the ROS2 side. The
  ROS2 GitHub stays one or two versions behind the firmware. That's
  the gap you're hitting.

  This isn't a bug in your setup — it's a versioning mismatch
  between two pieces of Unitree's stack that you don't control.
  Anyone running the latest firmware on a Go2 with the latest
  unitree_ros2 from GitHub will hit the same wall on
  will hit the same wall on Go2FrontVideoData.

  What to do about it

  Three real options:

  1. Find the firmware's actual schema by reading the C++ SDK. Clone
  unitree_sdk2, find their Go2FrontVideoData.idl or equivalent, port the
   differences into your local Go2FrontVideoData.msg, rebuild
  cyclonedds_ws. Most "correct" fix. Probably 30–60 min.
  We confirmed our recipe matches Unitree's official ROS2 GitHub
  repo — they're identical. But when we read one frame off the wire,
   the lengths came back nonsensical:

  video720p: length 360                      ← believable
  video360p: length 1,535,818,905 (1.5 GB)   ← physically impossible

  A 1.5 GB array can't exist in a single ROS message. So our
  deserializer didn't actually find a real "length 1.5 GB array" —
  it read random H.264 video bytes from the actual frame data and
  interpreted them as a length number, because it had drifted
  off-position.

  Why it drifts off-position

  Imagine the recipe says "skip 8 bytes for the timestamp" but the
  firmware actually wrote 12 bytes (because in its recipe there's a
  4-byte frame_id field after the timestamp). The receiver skips 8,
  reads the next 4 as "length of video720p" — but those 4 bytes are
  actually the second half of the frame_id. Now the receiver thinks
  the video720p array starts where frame_id's last bytes ended.
  Everything is offset by 4 bytes from then on, and every subsequent
   length-and-data pair is computed from H.264 garbage.

  We don't know exactly what the firmware's recipe is — could be an
  extra field, a renamed field, a different type, fields in a
  different order. We just know it doesn't match ours.

  Why this happens with Unitree specifically

  Unitree maintains two separate SDKs for talking to the Go2:

  - unitree_sdk2 (C++) — what their official apps use
  - unitree_ros2 (this repo) — the ROS2 mirror

  They generate from independent IDL files. When Unitree updates the
   C++ SDK to add a field to Go2FrontVideoData, they sometimes ship
  that change to the firmware before updating the ROS2 side. The
  ROS2 GitHub stays one or two versions behind the firmware. That's
  the gap you're hitting.

  This isn't a bug in your setup — it's a versioning mismatch
  between two pieces of Unitree's stack that you don't control.
  Anyone running the latest firmware on a Go2 with the latest
  unitree_ros2 from GitHub will hit the same wall on
  Go2FrontVideoData.

  What to do about it

  Three real options:

  1. Find the firmware's actual schema by reading the C++ SDK. Clone
   unitree_sdk2, find their Go2FrontVideoData.idl or equivalent,
  port the differences into your local Go2FrontVideoData.msg,
  rebuild cyclonedds_ws. Most "correct" fix. Probably 30–60 min.
  2. Capture the raw bytes and reverse-engineer the layout.
  Subscribe in raw mode, dump the first 64 bytes hex, look at where
  the timestamp ends and the H.264 frame starts (an H.264 frame
  always begins with the start code 00 00 00 01 or 00 00 01). Then
  the .msg file is whatever fields fit between the timestamp and
  that start code. Faster — maybe 15 min.
  3. Use a different camera interface entirely. The Go2's
  /api/videohub/* topics are a service-style API that might publish
  images via a different path. Or there's a WebRTC endpoint the
  Unitree app uses. These are less documented and potentially a
  deeper rabbit hole.

  If you want to pursue this, option 2 is fastest. Want me to write
  the raw-byte subscriber?


----------------------------------------------------------------------
geometry_msgs : x = bearing(angle in rad) y = distance (m) # or maybe 
# x of the camera and size of bounding box
api/sport/request : x = vx (m/s), y = vy (m/s), z=yaw rate (rad/s)
calibrate k_distance 

## commands
source ~/hack_2026/unitree_ros2/setup_robot.sh

-----------------------
#debug camera for perception input

-----
# connect to the robot
A. Log into the Go2

  The Go2's onboard computer (Jetson) is reachable over the same
  Ethernet. Common defaults — try in order until one works:

  # typical Jetson IP on Go2
  ssh unitree@192.168.123.18

  # alt IP that some firmwares use
  ssh unitree@192.168.123.161

  # alt user name / IP
  ssh pi@192.168.123.10

  Default passwords are usually one of: 123, unitree, pi. If none of
   these work, the password may have been set by Unitree at the
  factory — check the sticker on the robot or the Unitree app.

  If SSH fails with "host key verification failed" (because you've
  used this IP before for something else), ssh -o 
  StrictHostKeyChecking=no unitree@192.168.123.18.

  B. Find the message definition

  Once logged in:

  find / -name "Go2FrontVideoData*" 2>/dev/null

  You'll likely get hits in /opt/, /usr/, or ~/unitree_ros2/. The
  .msg is human-readable; the .idl (if present) is the canonical IDL
   form.

  # whatever path the find returned for the .msg:
  cat /path/to/Go2FrontVideoData.msg

  # also worth checking the IDL if present:
  cat /path/to/Go2FrontVideoData.idl

  C. Bonus checks while you're in

  Get the firmware version and SDK version installed on the robot:
  cat /etc/os-release          # Jetson OS info
  ls -la ~/unitree_ros2/       # the SDK install (if mirrored)
  ls /opt/unitree* 2>/dev/null # alternate install paths
  ros2 pkg prefix unitree_go 2>/dev/null
  
  D. Send me the output

  Paste:
  1. The result of find / -name "Go2FrontVideoData*"
  2. The contents of the .msg (and .idl if present)
  3. Anything from the bonus checks that returned interesting paths

  Then I can diff that against our local
  cyclonedds_ws/src/unitree/unitree_go/msg/Go2FrontVideoData.msg,
  patch ours to match, rebuild, and the perception node should
  actually decode.