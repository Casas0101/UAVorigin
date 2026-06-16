#!/usr/bin/env python3
"""Bootstrap ROS 2 node for publishing TargetObservationFrame JSON frames.

This node is intentionally usable before the project has a frozen custom
TargetObservationFrame.msg. It receives the Gazebo-facing inputs, packages a
schema-stable JSON frame, and publishes it as std_msgs/String.

Expected input topics by default:
  /camera/image_raw        sensor_msgs/msg/Image
  /camera/camera_info      sensor_msgs/msg/CameraInfo
  /uav/odom                nav_msgs/msg/Odometry
  /target/detection        std_msgs/msg/String containing JSON
  /target/odom_gt          nav_msgs/msg/Odometry, simulation/evaluation only

Output topic by default:
  /reacquisition/target_observation_frame  std_msgs/msg/String containing JSON
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Mapping

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - exercised only without ROS 2.
    rclpy = None
    Node = object
    Odometry = object
    CameraInfo = object
    Image = object
    String = object
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


REJECT_OK = 0
REJECT_NO_DETECTION = 1
REJECT_STALE_INPUT = 2
REJECT_BBOX_INVALID = 3
REJECT_UNCERTAINTY_UNAVAILABLE = 18

REJECT_REASON_NAMES = {
    REJECT_OK: "OK",
    REJECT_NO_DETECTION: "NO_DETECTION",
    REJECT_STALE_INPUT: "STALE_INPUT",
    REJECT_BBOX_INVALID: "BBOX_INVALID",
    REJECT_UNCERTAINTY_UNAVAILABLE: "UNCERTAINTY_UNAVAILABLE",
}

NS_PER_SEC = 1_000_000_000
NS_PER_MS = 1_000_000
CLOCK_DOMAIN_LIMIT_NS = 3600 * NS_PER_SEC


@dataclass
class CachedMessage:
    msg: Any
    stamp_ns: int | None
    arrival_ns: int
    payload: Mapping[str, Any] | None = None


def _stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * NS_PER_SEC + int(nanosec)


def _stamp_to_dict(stamp: Any) -> dict[str, int] | None:
    stamp_ns = _stamp_to_ns(stamp)
    if stamp_ns is None:
        return None
    return {
        "sec": int(getattr(stamp, "sec")),
        "nanosec": int(getattr(stamp, "nanosec")),
    }


def _header_stamp_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    return _stamp_to_ns(getattr(header, "stamp", None))


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _int_or_none(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _bool_or_none(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "y"}:
            return True
        if lowered in {"false", "0", "no", "n"}:
            return False
    return None


def _first_value(payload: Mapping[str, Any] | None, names: tuple[str, ...]) -> Any:
    if not payload:
        return None
    for name in names:
        if name in payload:
            return payload[name]
    return None


def _first_float(payload: Mapping[str, Any] | None, names: tuple[str, ...]) -> float | None:
    return _float_or_none(_first_value(payload, names))


def _point_to_dict(point: Any) -> dict[str, float]:
    return {
        "x": float(point.x),
        "y": float(point.y),
        "z": float(point.z),
    }


def _quaternion_to_dict(quaternion: Any) -> dict[str, float]:
    return {
        "x": float(quaternion.x),
        "y": float(quaternion.y),
        "z": float(quaternion.z),
        "w": float(quaternion.w),
    }


def _vector3_to_dict(vector: Any) -> dict[str, float]:
    return {
        "x": float(vector.x),
        "y": float(vector.y),
        "z": float(vector.z),
    }


def _odom_pose_to_dict(msg: Any) -> dict[str, Any]:
    pose = msg.pose.pose
    return {
        "position": _point_to_dict(pose.position),
        "orientation": _quaternion_to_dict(pose.orientation),
    }


def _odom_twist_to_dict(msg: Any) -> dict[str, Any]:
    twist = msg.twist.twist
    return {
        "linear": _vector3_to_dict(twist.linear),
        "angular": _vector3_to_dict(twist.angular),
    }


def _duration_ms(reference_ns: int | None, sample_ns: int | None) -> float | None:
    if reference_ns is None or sample_ns is None:
        return None
    delta_ns = reference_ns - sample_ns
    if abs(delta_ns) > CLOCK_DOMAIN_LIMIT_NS:
        return None
    return delta_ns / NS_PER_MS


def _message_stamp_dict(stamp: Any) -> dict[str, int]:
    stamp_dict = _stamp_to_dict(stamp)
    if stamp_dict is None:
        return {"sec": 0, "nanosec": 0}
    return stamp_dict


def _message_float(value: Any, default: float = 0.0) -> float:
    parsed = _float_or_none(value)
    return default if parsed is None else parsed


def _message_duration_ms(value: Any) -> float:
    return _message_float(value, -1.0)


def _message_point(value: Mapping[str, Any] | None) -> dict[str, float]:
    if not isinstance(value, Mapping):
        return {"x": 0.0, "y": 0.0, "z": 0.0}
    return {
        "x": _message_float(value.get("x")),
        "y": _message_float(value.get("y")),
        "z": _message_float(value.get("z")),
    }


def _message_pose(value: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        value = {}
    orientation = value.get("orientation")
    if not isinstance(orientation, Mapping):
        orientation = {}
    return {
        "position": _message_point(value.get("position")),
        "orientation": {
            "x": _message_float(orientation.get("x")),
            "y": _message_float(orientation.get("y")),
            "z": _message_float(orientation.get("z")),
            "w": _message_float(orientation.get("w"), 1.0),
        },
    }


def _message_twist(value: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        value = {}
    return {
        "linear": _message_point(value.get("linear")),
        "angular": _message_point(value.get("angular")),
    }


def _message_covariance_xy(value: list[float] | None) -> list[float]:
    if not value or len(value) < 4:
        return [0.0, 0.0, 0.0, 0.0]
    return [_message_float(item) for item in value[:4]]


def _payload_stamp_ns(payload: Mapping[str, Any] | None) -> int | None:
    if not payload:
        return None

    stamp_ns = _int_or_none(payload.get("stamp_ns"))
    if stamp_ns is not None:
        return stamp_ns

    stamp = payload.get("stamp")
    if isinstance(stamp, Mapping):
        sec = _int_or_none(stamp.get("sec"))
        nanosec = _int_or_none(stamp.get("nanosec"))
        if sec is not None and nanosec is not None:
            return sec * NS_PER_SEC + nanosec

    header = payload.get("header")
    if isinstance(header, Mapping):
        nested_stamp = header.get("stamp")
        if isinstance(nested_stamp, Mapping):
            sec = _int_or_none(nested_stamp.get("sec"))
            nanosec = _int_or_none(nested_stamp.get("nanosec"))
            if sec is not None and nanosec is not None:
                return sec * NS_PER_SEC + nanosec

    return None


def _extract_bbox(payload: Mapping[str, Any] | None) -> dict[str, float | None]:
    bbox_source: Mapping[str, Any] | None = None
    if payload and isinstance(payload.get("bbox"), Mapping):
        bbox_source = payload["bbox"]
    elif payload:
        bbox_source = payload

    left = _first_float(bbox_source, ("bbox_left", "left", "x_min", "xmin", "x1"))
    top = _first_float(bbox_source, ("bbox_top", "top", "y_min", "ymin", "y1"))
    right = _first_float(bbox_source, ("bbox_right", "right", "x_max", "xmax", "x2"))
    bottom = _first_float(bbox_source, ("bbox_bottom", "bottom", "y_max", "ymax", "y2"))

    x = _first_float(bbox_source, ("x", "bbox_x"))
    y = _first_float(bbox_source, ("y", "bbox_y"))
    width = _first_float(bbox_source, ("width", "w", "bbox_width"))
    height = _first_float(bbox_source, ("height", "h", "bbox_height"))

    if left is None and x is not None:
        left = x
    if top is None and y is not None:
        top = y
    if right is None and left is not None and width is not None:
        right = left + width
    if bottom is None and top is not None and height is not None:
        bottom = top + height

    if payload and isinstance(payload.get("bbox"), list) and len(payload["bbox"]) >= 4:
        raw_bbox = payload["bbox"]
        left = _float_or_none(raw_bbox[0])
        top = _float_or_none(raw_bbox[1])
        right = _float_or_none(raw_bbox[2])
        bottom = _float_or_none(raw_bbox[3])

    width = None
    height = None
    if left is not None and right is not None:
        width = right - left
    if top is not None and bottom is not None:
        height = bottom - top

    center_u = _first_float(payload, ("center_u", "u", "cx"))
    center_v = _first_float(payload, ("center_v", "v", "cy"))
    if center_u is None and left is not None and width is not None:
        center_u = left + width * 0.5
    if center_v is None and top is not None and height is not None:
        center_v = top + height * 0.5

    foot_u = _first_float(payload, ("foot_u", "bottom_center_u"))
    foot_v = _first_float(payload, ("foot_v", "bottom_center_v"))
    if foot_u is None:
        foot_u = center_u
    if foot_v is None:
        foot_v = bottom

    pixel_area = _first_float(payload, ("pixel_area", "area", "area_pix", "A_pix"))
    min_pixel_size = _first_float(payload, ("min_pixel_size", "s_pix"))
    if pixel_area is None and width is not None and height is not None:
        pixel_area = max(0.0, width) * max(0.0, height)
    if min_pixel_size is None and width is not None and height is not None:
        min_pixel_size = min(max(0.0, width), max(0.0, height))

    return {
        "bbox_left": left,
        "bbox_top": top,
        "bbox_right": right,
        "bbox_bottom": bottom,
        "center_u": center_u,
        "center_v": center_v,
        "foot_u": foot_u,
        "foot_v": foot_v,
        "bbox_width": width,
        "bbox_height": height,
        "pixel_area": pixel_area,
        "min_pixel_size": min_pixel_size,
    }


def _bbox_is_valid(bbox: Mapping[str, float | None]) -> bool:
    required = ("bbox_left", "bbox_top", "bbox_right", "bbox_bottom")
    if any(bbox.get(name) is None for name in required):
        return False
    width = bbox.get("bbox_width")
    height = bbox.get("bbox_height")
    return width is not None and height is not None and width > 0.0 and height > 0.0


def _extract_ground_position(payload: Mapping[str, Any] | None) -> dict[str, float] | None:
    if not payload:
        return None

    position = payload.get("ground_position")
    if isinstance(position, Mapping):
        x = _float_or_none(position.get("x"))
        y = _float_or_none(position.get("y"))
        z = _float_or_none(position.get("z"))
        if x is not None and y is not None:
            return {"x": x, "y": y, "z": 0.0 if z is None else z}

    x = _first_float(payload, ("ground_x", "x_world", "world_x", "target_x"))
    y = _first_float(payload, ("ground_y", "y_world", "world_y", "target_y"))
    z = _first_float(payload, ("ground_z", "z_world", "world_z", "target_z"))
    if x is not None and y is not None:
        return {"x": x, "y": y, "z": 0.0 if z is None else z}

    return None


def _extract_covariance_xy(payload: Mapping[str, Any] | None) -> list[float] | None:
    if not payload:
        return None

    raw = payload.get("ground_covariance_xy")
    if raw is None:
        raw = payload.get("covariance_xy")
    if isinstance(raw, list) and len(raw) >= 4:
        values = [_float_or_none(value) for value in raw[:4]]
        if all(value is not None for value in values):
            return [float(value) for value in values if value is not None]

    xx = _first_float(payload, ("cov_xx", "ground_cov_xx"))
    xy = _first_float(payload, ("cov_xy", "ground_cov_xy"))
    yx = _first_float(payload, ("cov_yx", "ground_cov_yx"))
    yy = _first_float(payload, ("cov_yy", "ground_cov_yy"))
    if xx is not None and xy is not None and yx is not None and yy is not None:
        return [xx, xy, yx, yy]

    return None


class TargetObservationFrameNode(Node):
    """Receive first-ring inputs and publish TargetObservationFrame JSON."""

    def __init__(self) -> None:
        super().__init__("target_observation_frame_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("uav_odom_topic", "/uav/odom")
        self.declare_parameter("detection_topic", "/target/detection")
        self.declare_parameter("target_gt_topic", "/target/odom_gt")
        self.declare_parameter("output_topic", "/reacquisition/target_observation_frame")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("config_revision", 0)
        self.declare_parameter("max_detection_age_sec", 0.5)
        self.declare_parameter("max_odom_age_sec", 0.5)
        self.declare_parameter("max_gt_age_sec", 0.5)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.uav_odom_topic = self.get_parameter("uav_odom_topic").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.target_gt_topic = self.get_parameter("target_gt_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.config_revision = int(self.get_parameter("config_revision").value)
        self.max_detection_age_sec = float(self.get_parameter("max_detection_age_sec").value)
        self.max_odom_age_sec = float(self.get_parameter("max_odom_age_sec").value)
        self.max_gt_age_sec = float(self.get_parameter("max_gt_age_sec").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        default_qos = QoSProfile(depth=10)

        self.publisher = self.create_publisher(String, self.output_topic, default_qos)

        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(Odometry, self.uav_odom_topic, self._on_uav_odom, default_qos)
        self.create_subscription(String, self.detection_topic, self._on_detection, default_qos)
        self.create_subscription(Odometry, self.target_gt_topic, self._on_target_gt, default_qos)

        self._camera_info: CachedMessage | None = None
        self._uav_odom: CachedMessage | None = None
        self._detection: CachedMessage | None = None
        self._target_gt: CachedMessage | None = None
        self._source_sequence = 0

        self.get_logger().info(
            "TargetObservationFrameNode publishing JSON frames on "
            f"{self.output_topic}"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_camera_info(self, msg: Any) -> None:
        self._camera_info = CachedMessage(
            msg=msg,
            stamp_ns=_header_stamp_ns(msg),
            arrival_ns=self._now_ns(),
        )

    def _on_uav_odom(self, msg: Any) -> None:
        self._uav_odom = CachedMessage(
            msg=msg,
            stamp_ns=_header_stamp_ns(msg),
            arrival_ns=self._now_ns(),
        )

    def _on_target_gt(self, msg: Any) -> None:
        self._target_gt = CachedMessage(
            msg=msg,
            stamp_ns=_header_stamp_ns(msg),
            arrival_ns=self._now_ns(),
        )

    def _on_detection(self, msg: Any) -> None:
        arrival_ns = self._now_ns()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"Ignoring non-JSON detection message on {self.detection_topic}"
            )
            return

        if not isinstance(payload, Mapping):
            self.get_logger().warn(
                f"Ignoring detection JSON that is not an object on {self.detection_topic}"
            )
            return

        self._detection = CachedMessage(
            msg=msg,
            stamp_ns=_payload_stamp_ns(payload),
            arrival_ns=arrival_ns,
            payload=payload,
        )

    def _on_image(self, msg: Any) -> None:
        start_perf_ns = time.perf_counter_ns()
        frame = self._build_frame(msg, start_perf_ns)
        output = String()
        output.data = json.dumps(frame, ensure_ascii=True, allow_nan=False, separators=(",", ":"))
        self.publisher.publish(output)

    def _cache_is_fresh(
        self,
        cache: CachedMessage | None,
        image_stamp_ns: int | None,
        now_ns: int,
        max_age_sec: float,
    ) -> bool:
        if cache is None:
            return False
        if max_age_sec < 0.0:
            return True

        max_age_ns = int(max_age_sec * NS_PER_SEC)
        if image_stamp_ns is not None and cache.stamp_ns is not None:
            stamp_delta = abs(image_stamp_ns - cache.stamp_ns)
            if stamp_delta <= CLOCK_DOMAIN_LIMIT_NS:
                return stamp_delta <= max_age_ns

        arrival_delta = abs(now_ns - cache.arrival_ns)
        if arrival_delta > CLOCK_DOMAIN_LIMIT_NS:
            return True
        return arrival_delta <= max_age_ns

    def _build_frame(self, image_msg: Any, start_perf_ns: int) -> dict[str, Any]:
        self._source_sequence += 1

        now_ns = self._now_ns()
        image_header = getattr(image_msg, "header", None)
        image_stamp = getattr(image_header, "stamp", None)
        image_stamp_ns = _stamp_to_ns(image_stamp)

        detection_cache = self._detection
        detection_fresh = self._cache_is_fresh(
            detection_cache,
            image_stamp_ns,
            now_ns,
            self.max_detection_age_sec,
        )
        detection_payload = detection_cache.payload if detection_fresh and detection_cache else None

        bbox = _extract_bbox(detection_payload)
        bbox_valid = _bbox_is_valid(bbox)
        detected_hint = _bool_or_none(
            _first_value(detection_payload, ("detected", "has_detection"))
        )
        detected = bool(detection_payload) and (
            bbox_valid if detected_hint is None else detected_hint
        )

        ground_position = _extract_ground_position(detection_payload)
        ground_covariance_xy = _extract_covariance_xy(detection_payload)

        valid_from_payload = _bool_or_none(_first_value(detection_payload, ("valid", "is_valid")))
        has_world_observation = ground_position is not None and ground_covariance_xy is not None
        if not detection_fresh and detection_cache is not None:
            valid = False
            reject_reason = REJECT_STALE_INPUT
        elif not detected:
            valid = False
            reject_reason = REJECT_NO_DETECTION
        elif not bbox_valid:
            valid = False
            reject_reason = REJECT_BBOX_INVALID
        elif valid_from_payload is not None:
            valid = valid_from_payload and has_world_observation
            reject_reason = (
                REJECT_OK
                if valid
                else _int_or_none(_first_value(detection_payload, ("reject_reason", "reject_code")))
                or REJECT_UNCERTAINTY_UNAVAILABLE
            )
        elif has_world_observation:
            valid = True
            reject_reason = REJECT_OK
        else:
            valid = False
            reject_reason = REJECT_UNCERTAINTY_UNAVAILABLE

        reliability = _first_float(detection_payload, ("reliability", "confidence", "score", "c_det"))
        blur_risk = _first_float(detection_payload, ("blur_risk", "q_blur"))
        edge_distance_px = self._edge_distance_px(image_msg, bbox)
        if edge_distance_px is None:
            edge_distance_px = _first_float(detection_payload, ("edge_distance_px", "d_edge"))

        uav_fresh = self._cache_is_fresh(
            self._uav_odom,
            image_stamp_ns,
            now_ns,
            self.max_odom_age_sec,
        )
        target_gt_fresh = self._cache_is_fresh(
            self._target_gt,
            image_stamp_ns,
            now_ns,
            self.max_gt_age_sec,
        )

        uav_pose = _odom_pose_to_dict(self._uav_odom.msg) if self._uav_odom and uav_fresh else None
        uav_twist = _odom_twist_to_dict(self._uav_odom.msg) if self._uav_odom and uav_fresh else None
        target_gt_position = (
            _point_to_dict(self._target_gt.msg.pose.pose.position)
            if self._target_gt and target_gt_fresh
            else None
        )
        target_gt_twist = (
            _odom_twist_to_dict(self._target_gt.msg)
            if self._target_gt and target_gt_fresh
            else None
        )

        compute_duration_ms = (time.perf_counter_ns() - start_perf_ns) / NS_PER_MS
        observation_age_ms = _duration_ms(now_ns, image_stamp_ns)
        processing_latency_ms = observation_age_ms

        frame = {
            "header": {
                "stamp": _message_stamp_dict(image_stamp),
                "frame_id": str(self.output_frame_id),
            },
            "source_sequence": int(self._source_sequence),
            "config_revision": int(self.config_revision),
            "detected": bool(detected),
            "valid": bool(valid),
            "reject_reason": int(reject_reason),
            "bbox_left": _message_float(bbox["bbox_left"]),
            "bbox_top": _message_float(bbox["bbox_top"]),
            "bbox_right": _message_float(bbox["bbox_right"]),
            "bbox_bottom": _message_float(bbox["bbox_bottom"]),
            "center_u": _message_float(bbox["center_u"]),
            "center_v": _message_float(bbox["center_v"]),
            "foot_u": _message_float(bbox["foot_u"]),
            "foot_v": _message_float(bbox["foot_v"]),
            "pixel_area": _message_float(bbox["pixel_area"]),
            "min_pixel_size": _message_float(bbox["min_pixel_size"]),
            "reliability": _message_float(reliability),
            "edge_distance_px": _message_float(edge_distance_px),
            "blur_risk": _message_float(blur_risk),
            "ground_position": _message_point(ground_position),
            "ground_covariance_xy": _message_covariance_xy(ground_covariance_xy),
            "uav_pose": _message_pose(uav_pose),
            "uav_twist": _message_twist(uav_twist),
            "interpolation_gap_ms": -1.0,
            "processing_latency_ms": _message_duration_ms(processing_latency_ms),
            "observation_age_ms": _message_duration_ms(observation_age_ms),
            "compute_duration_ms": _message_duration_ms(compute_duration_ms),
            "has_target_gt": bool(self._target_gt and target_gt_fresh),
            "target_gt_position": _message_point(target_gt_position),
            "target_gt_twist": _message_twist(target_gt_twist),
        }
        return frame

    def _edge_distance_px(
        self,
        image_msg: Any,
        bbox: Mapping[str, float | None],
    ) -> float | None:
        width = _float_or_none(getattr(image_msg, "width", None))
        height = _float_or_none(getattr(image_msg, "height", None))
        left = bbox.get("bbox_left")
        top = bbox.get("bbox_top")
        right = bbox.get("bbox_right")
        bottom = bbox.get("bbox_bottom")
        if None in (width, height, left, top, right, bottom):
            return None
        return min(float(left), float(top), width - float(right), height - float(bottom))

    def _camera_info_to_dict(self, msg: Any) -> dict[str, Any]:
        return {
            "width": int(msg.width),
            "height": int(msg.height),
            "distortion_model": str(msg.distortion_model),
            "k": [float(value) for value in msg.k],
            "d": [float(value) for value in msg.d],
            "r": [float(value) for value in msg.r],
            "p": [float(value) for value in msg.p],
        }


def main(args: list[str] | None = None) -> None:
    if ROS_IMPORT_ERROR is not None:
        raise SystemExit(
            "ROS 2 Python dependencies are unavailable. Run this inside a "
            "ROS 2 Humble environment with rclpy, sensor_msgs, nav_msgs, and "
            f"std_msgs installed. Original import error: {ROS_IMPORT_ERROR}"
        )

    rclpy.init(args=args)
    node = TargetObservationFrameNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
