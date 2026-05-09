#!/usr/bin/env python3
"""go2_perception — front-camera person detector for the follow controller.

Subscribes:
  Go2FrontVideoData  (unitree_go/msg/Go2FrontVideoData)  H.264 packets

Publishes:
  /follow/target     (geometry_msgs/msg/Vector3)
    x = normalized bearing in [-1, 1]   (0 = centered, +ve = person to the right)
    y = monocular distance estimate (m) (calibrate `k_distance` for your camera)
    z = unused

Run:
  ros2 run unitree_ros2_example go2_perception
  ros2 run unitree_ros2_example go2_perception --ros-args \
      -p camera_topic:=/frontvideostream -p resolution:=360p -p show_window:=true
"""
import sys

import av
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ultralytics import YOLO

from unitree_go.msg import Go2FrontVideoData


_FIELD_BY_RESOLUTION = {
    "720p": "video720p",
    "360p": "video360p",
    "180p": "video180p",
}


class Go2Perception(Node):
    def __init__(self) -> None:
        super().__init__("go2_perception")

        self.declare_parameter("camera_topic", "/frontvideostream")
        self.declare_parameter("resolution", "360p")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cuda")  # override with "cpu" or "cuda:0"
        self.declare_parameter("conf_threshold", 0.4)
        # distance(m) ≈ k_distance / bbox_height_px — calibrate at a known distance.
        self.declare_parameter("k_distance", 1700.0)
        self.declare_parameter("show_window", False)

        topic = self.get_parameter("camera_topic").value
        resolution = self.get_parameter("resolution").value
        if resolution not in _FIELD_BY_RESOLUTION:
            raise ValueError(
                f"resolution must be one of {list(_FIELD_BY_RESOLUTION)}, got {resolution!r}"
            )
        self.video_field = _FIELD_BY_RESOLUTION[resolution]
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.k_dist = float(self.get_parameter("k_distance").value)
        self.show = bool(self.get_parameter("show_window").value)
        self.device = str(self.get_parameter("device").value)

        model_path = self.get_parameter("model_path").value
        self.get_logger().info(f"loading YOLO model: {model_path} on {self.device}")
        self.model = YOLO(model_path)

        self.codec = av.CodecContext.create("h264", "r")

        self.pub = self.create_publisher(Vector3, "/follow/target", 10)
        self.sub = self.create_subscription(
            Go2FrontVideoData, topic, self.on_video, 10
        )
        self.frame_count = 0
        self.get_logger().info(
            f"subscribed to {topic} ({resolution}); publishing /follow/target"
        )

    def on_video(self, msg: Go2FrontVideoData) -> None:
        payload = bytes(getattr(msg, self.video_field))
        if not payload:
            return
        try:
            packets = self.codec.parse(payload)
        except Exception as e:  # PyAV raises a variety of FFmpeg-wrapped errors
            self.get_logger().warn(f"H.264 parse failed: {e}")
            return

        for packet in packets:
            try:
                frames = self.codec.decode(packet)
            except Exception as e:
                self.get_logger().warn(f"H.264 decode failed: {e}")
                continue
            for frame in frames:
                self.process_frame(frame.to_ndarray(format="bgr24"))

    def process_frame(self, img: np.ndarray) -> None:
        h, w = img.shape[:2]
        # predict() avoids ByteTrack (which pulls scipy and crashes on numpy 2.x).
        # Swap to model.track(persist=True) once tracker stability is needed.
        results = self.model.predict(
            img, classes=[0], conf=self.conf, device=self.device, verbose=False
        )
        if not results:
            return
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self._maybe_show(img, None)
            return

        # Pick the largest bbox (closest person). Swap for track-id persistence later.
        xywh = boxes.xywh.cpu().numpy()  # (N, 4): cx, cy, w, h
        idx = int(np.argmax(xywh[:, 2] * xywh[:, 3]))
        cx, _cy, _bw, bh = xywh[idx]

        bearing = float((cx - w / 2.0) / (w / 2.0))
        distance = float(self.k_dist / max(bh, 1.0))

        msg = Vector3(x=bearing, y=distance, z=0.0)
        self.pub.publish(msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"target: bearing={bearing:+.2f}  distance={distance:.2f} m  "
                f"({len(boxes)} ppl, bbox_h={bh:.0f}px)"
            )

        self._maybe_show(img, boxes.xyxy[idx].cpu().numpy().astype(int))

    def _maybe_show(self, img: np.ndarray, xyxy) -> None:
        if not self.show:
            return
        if xyxy is not None:
            x1, y1, x2, y2 = xyxy
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow("go2_perception", img)
        cv2.waitKey(1)


def main() -> int:
    rclpy.init()
    node = Go2Perception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
