#!/usr/bin/env python3
"""ROS 2 adapter for publishing FirstRingRawFrame JSON.

This node keeps ROS 2/Gazebo/PX4 integration outside the pure Python raw
middleware package. It subscribes to raw simulation/runtime inputs, packages
them into the schema-0.1 FirstRingRawFrame dataclasses, validates the frame,
and publishes JSON on /reacquisition/first_ring/raw_frame.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


_DEV_RAW_PARENT = Path(__file__).resolve().parent.parent / "reacquisition_first_ring_raw"
if _DEV_RAW_PARENT.exists() and str(_DEV_RAW_PARENT) not in sys.path:
    sys.path.insert(0, str(_DEV_RAW_PARENT))

try:
    from reacquisition_first_ring_raw import raw_frame, schema, validators
except ImportError as exc:  # pragma: no cover - only when package install is broken.
    raw_frame = None
    schema = None
    validators = None
    RAW_IMPORT_ERROR = exc
else:
    RAW_IMPORT_ERROR = None

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

    try:
        from px4_msgs.msg import VehicleOdometry
    except ImportError:  # pragma: no cover - px4_msgs is optional.
        VehicleOdometry = None
except ImportError as exc:  # pragma: no cover - exercised outside ROS 2.
    rclpy = None
    Node = object
    Odometry = object
    CameraInfo = object
    Image = object
    String = object
    VehicleOdometry = None
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


NS_PER_SEC = 1_000_000_000
NS_PER_US = 1_000
CLOCK_DOMAIN_LIMIT_NS = 3600 * NS_PER_SEC

UAV_MSG_ODOM = "nav_msgs/Odometry"
UAV_MSG_PX4_ODOM = "px4_msgs/VehicleOdometry"


@dataclass
class CachedMessage:
    msg: Any
    stamp_ns: int | None
    arrival_ns: int


def _stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * NS_PER_SEC + int(nanosec)


def _stamp_from_ns(total_ns: int) -> Any:
    if raw_frame is None:
        raise RuntimeError("reacquisition_first_ring_raw is unavailable")
    return raw_frame.stamp_from_ns(max(0, int(total_ns)))


def _stamp_from_ros_stamp(stamp: Any) -> Any:
    stamp_ns = _stamp_to_ns(stamp)
    if stamp_ns is None:
        return _stamp_from_ns(0)
    return _stamp_from_ns(stamp_ns)


def _stamp_from_px4_us(timestamp_us: Any) -> Any:
    try:
        timestamp_ns = int(timestamp_us) * NS_PER_US
    except (TypeError, ValueError):
        timestamp_ns = 0
    return _stamp_from_ns(timestamp_ns)


def _header_stamp_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    return _stamp_to_ns(getattr(header, "stamp", None))


def _header_frame_id(msg: Any, fallback: str) -> str:
    header = getattr(msg, "header", None)
    frame_id = getattr(header, "frame_id", None)
    return str(frame_id) if frame_id else fallback


def _vector3(value: Any) -> Any:
    return raw_frame.Vector3(float(value.x), float(value.y), float(value.z))


def _point(value: Any) -> Any:
    return raw_frame.Vector3(float(value.x), float(value.y), float(value.z))


def _quaternion(value: Any) -> Any:
    return raw_frame.Quaternion(
        float(value.x),
        float(value.y),
        float(value.z),
        float(value.w),
    )


def _array_vector3(values: Any) -> Any:
    return raw_frame.Vector3(float(values[0]), float(values[1]), float(values[2]))


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("true", "1", "yes", "y", "on"):
            return True
        if lowered in ("false", "0", "no", "n", "off"):
            return False
    return bool(value)


def _px4_quaternion(q: Any) -> Any:
    # px4_msgs/VehicleOdometry.q is [w, x, y, z].
    return raw_frame.Quaternion(float(q[1]), float(q[2]), float(q[3]), float(q[0]))


def odometry_to_uav_state(
    msg: Any,
    *,
    source: str,
    raw_frame_convention: str,
    fallback_frame_id: str,
) -> Any:
    pose = msg.pose.pose
    twist = msg.twist.twist
    return raw_frame.UavStateRaw(
        stamp=_stamp_from_ros_stamp(msg.header.stamp),
        frame_id=_header_frame_id(msg, fallback_frame_id),
        raw_frame_convention=raw_frame_convention,
        position=_point(pose.position),
        orientation=_quaternion(pose.orientation),
        linear_velocity=_vector3(twist.linear),
        angular_velocity=_vector3(twist.angular),
        source=source,
    )


def px4_vehicle_odometry_to_uav_state(
    msg: Any,
    *,
    source: str,
    raw_frame_convention: str,
    fallback_frame_id: str,
) -> Any:
    timestamp_us = getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0)
    return raw_frame.UavStateRaw(
        stamp=_stamp_from_px4_us(timestamp_us),
        frame_id=fallback_frame_id,
        raw_frame_convention=raw_frame_convention,
        position=_array_vector3(msg.position),
        orientation=_px4_quaternion(msg.q),
        linear_velocity=_array_vector3(msg.velocity),
        angular_velocity=_array_vector3(msg.angular_velocity),
        source=source,
    )


def odometry_to_target_state(
    msg: Any,
    *,
    source: str,
    target_id: str,
    fallback_frame_id: str,
) -> Any:
    pose = msg.pose.pose
    twist = msg.twist.twist
    return raw_frame.TargetStateRaw(
        has_target_state=True,
        target_id=target_id,
        stamp=_stamp_from_ros_stamp(msg.header.stamp),
        frame_id=_header_frame_id(msg, fallback_frame_id),
        position=_point(pose.position),
        orientation=_quaternion(pose.orientation),
        linear_velocity=_vector3(twist.linear),
        angular_velocity=_vector3(twist.angular),
        source=source,
    )


def unavailable_target_state() -> Any:
    return raw_frame.TargetStateRaw(
        has_target_state=False,
        target_id="",
        stamp=None,
        frame_id=None,
        position=None,
        orientation=None,
        linear_velocity=None,
        angular_velocity=None,
        source=schema.SRC_UNAVAILABLE,
    )


def image_to_meta(msg: Any, *, topic: str, image_sequence: int, data_transport: str) -> Any:
    return raw_frame.ImageMetaRaw(
        header=raw_frame.Header(
            stamp=_stamp_from_ros_stamp(msg.header.stamp),
            frame_id=_header_frame_id(msg, schema.DEFAULT_CAMERA_OPTICAL_FRAME_ID),
        ),
        image_topic=topic,
        image_sequence=int(image_sequence),
        height=int(msg.height),
        width=int(msg.width),
        encoding=str(msg.encoding),
        is_bigendian=int(msg.is_bigendian),
        step=int(msg.step),
        data_transport=data_transport,
        data_base64=None,
    )


def camera_info_to_raw(msg: Any, *, topic: str) -> Any:
    return raw_frame.CameraInfoRaw(
        header=raw_frame.Header(
            stamp=_stamp_from_ros_stamp(msg.header.stamp),
            frame_id=_header_frame_id(msg, schema.DEFAULT_CAMERA_OPTICAL_FRAME_ID),
        ),
        camera_info_topic=topic,
        height=int(msg.height),
        width=int(msg.width),
        distortion_model=str(msg.distortion_model),
        d=[float(value) for value in msg.d],
        k=[float(value) for value in msg.k],
        r=[float(value) for value in msg.r],
        p=[float(value) for value in msg.p],
    )


def cache_is_fresh(
    cache: CachedMessage | None,
    sample_stamp_ns: int | None,
    now_ns: int,
    max_age_sec: float,
) -> bool:
    if cache is None:
        return False
    if max_age_sec < 0.0:
        return True

    max_age_ns = int(max_age_sec * NS_PER_SEC)
    if sample_stamp_ns is not None and cache.stamp_ns is not None:
        delta = abs(sample_stamp_ns - cache.stamp_ns)
        if delta <= CLOCK_DOMAIN_LIMIT_NS:
            return delta <= max_age_ns

    arrival_delta = abs(now_ns - cache.arrival_ns)
    if arrival_delta > CLOCK_DOMAIN_LIMIT_NS:
        return True
    return arrival_delta <= max_age_ns


def _uav_cache_stamp_ns(msg: Any, uav_state_msg_type: str) -> int | None:
    if uav_state_msg_type == UAV_MSG_PX4_ODOM:
        # PX4 VehicleOdometry timestamps are PX4 internal microsecond clocks, not
        # necessarily comparable with Gazebo image header stamps. Use arrival
        # time for freshness while preserving the PX4 stamp in the raw frame.
        return None
    return _header_stamp_ns(msg)


class RawFramePublisherNode(Node):
    """Publish FirstRingRawFrame JSON from ROS 2 input topics."""

    def __init__(self) -> None:
        super().__init__("first_ring_raw_frame_node")

        self.declare_parameter("image_topic", schema.DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("camera_info_topic", schema.DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter("uav_state_topic", "/uav/odom")
        self.declare_parameter("uav_state_msg_type", UAV_MSG_ODOM)
        self.declare_parameter("target_state_topic", "/target/odom_gt")
        self.declare_parameter("target_state_enabled", True)
        self.declare_parameter("output_topic", schema.RAW_FRAME_TOPIC)
        self.declare_parameter("config_revision", 0)
        self.declare_parameter("raw_frame_convention", schema.COORD_CONVENTION_ENU_FLU)
        self.declare_parameter("uav_frame_id", schema.DEFAULT_GLOBAL_FRAME_ID)
        self.declare_parameter("target_frame_id", schema.DEFAULT_GLOBAL_FRAME_ID)
        self.declare_parameter("target_id", "target_0")
        self.declare_parameter("primary_time_source", schema.PRIMARY_TIME_SOURCE_IMAGE)
        self.declare_parameter("uav_source", schema.SRC_GAZEBO_MODEL_STATE)
        self.declare_parameter("target_source", schema.SRC_GAZEBO_GROUND_TRUTH)
        self.declare_parameter("producer", "ros2_raw_frame_publisher")
        self.declare_parameter("environment", "wsl2_ros2_gazebo_px4")
        self.declare_parameter("max_camera_info_age_sec", 1.0)
        self.declare_parameter("max_uav_state_age_sec", 0.5)
        self.declare_parameter("max_target_state_age_sec", 0.5)
        self.declare_parameter("validate_before_publish", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.uav_state_topic = str(self.get_parameter("uav_state_topic").value)
        self.uav_state_msg_type = str(self.get_parameter("uav_state_msg_type").value)
        self.target_state_topic = str(self.get_parameter("target_state_topic").value)
        self.target_state_enabled = _as_bool(self.get_parameter("target_state_enabled").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.config_revision = int(self.get_parameter("config_revision").value)
        self.raw_frame_convention = str(self.get_parameter("raw_frame_convention").value)
        self.uav_frame_id = str(self.get_parameter("uav_frame_id").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.target_id = str(self.get_parameter("target_id").value)
        self.primary_time_source = str(self.get_parameter("primary_time_source").value)
        self.uav_source = str(self.get_parameter("uav_source").value)
        self.target_source = str(self.get_parameter("target_source").value)
        self.producer = str(self.get_parameter("producer").value)
        self.environment = str(self.get_parameter("environment").value)
        self.max_camera_info_age_sec = float(self.get_parameter("max_camera_info_age_sec").value)
        self.max_uav_state_age_sec = float(self.get_parameter("max_uav_state_age_sec").value)
        self.max_target_state_age_sec = float(self.get_parameter("max_target_state_age_sec").value)
        self.validate_before_publish = _as_bool(self.get_parameter("validate_before_publish").value)

        self._validate_parameters()

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

        if self.uav_state_msg_type == UAV_MSG_PX4_ODOM:
            if VehicleOdometry is None:
                raise RuntimeError("uav_state_msg_type=px4_msgs/VehicleOdometry requires px4_msgs")
            self.create_subscription(VehicleOdometry, self.uav_state_topic, self._on_uav_state, default_qos)
        else:
            self.create_subscription(Odometry, self.uav_state_topic, self._on_uav_state, default_qos)

        if self.target_state_enabled:
            self.create_subscription(Odometry, self.target_state_topic, self._on_target_state, default_qos)

        self._camera_info: CachedMessage | None = None
        self._uav_state: CachedMessage | None = None
        self._target_state: CachedMessage | None = None
        self._source_sequence = 0
        self._last_missing_warn_ns = 0

        self.get_logger().info(
            f"Publishing FirstRingRawFrame JSON on {self.output_topic}"
        )

    def _validate_parameters(self) -> None:
        if self.uav_state_msg_type not in (UAV_MSG_ODOM, UAV_MSG_PX4_ODOM):
            raise ValueError(f"unsupported uav_state_msg_type: {self.uav_state_msg_type}")
        if self.raw_frame_convention not in schema.ALLOWED_COORD_CONVENTIONS:
            raise ValueError(f"unsupported raw_frame_convention: {self.raw_frame_convention}")
        if self.primary_time_source not in schema.ALLOWED_PRIMARY_TIME_SOURCES:
            raise ValueError(f"unsupported primary_time_source: {self.primary_time_source}")
        if self.uav_source not in schema.ALLOWED_UAV_SOURCES:
            raise ValueError(f"unsupported uav_source: {self.uav_source}")
        if self.target_source not in schema.ALLOWED_TARGET_SOURCES:
            raise ValueError(f"unsupported target_source: {self.target_source}")

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_camera_info(self, msg: Any) -> None:
        self._camera_info = CachedMessage(
            msg=msg,
            stamp_ns=_header_stamp_ns(msg),
            arrival_ns=self._now_ns(),
        )

    def _on_uav_state(self, msg: Any) -> None:
        self._uav_state = CachedMessage(
            msg=msg,
            stamp_ns=_uav_cache_stamp_ns(msg, self.uav_state_msg_type),
            arrival_ns=self._now_ns(),
        )

    def _on_target_state(self, msg: Any) -> None:
        self._target_state = CachedMessage(
            msg=msg,
            stamp_ns=_header_stamp_ns(msg),
            arrival_ns=self._now_ns(),
        )

    def _on_image(self, msg: Any) -> None:
        now_ns = self._now_ns()
        image_stamp_ns = _header_stamp_ns(msg)

        if not cache_is_fresh(
            self._camera_info,
            image_stamp_ns,
            now_ns,
            self.max_camera_info_age_sec,
        ):
            self._warn_missing("Skipping raw frame: camera_info is unavailable or stale", now_ns)
            return

        if not cache_is_fresh(
            self._uav_state,
            image_stamp_ns,
            now_ns,
            self.max_uav_state_age_sec,
        ):
            self._warn_missing("Skipping raw frame: UAV state is unavailable or stale", now_ns)
            return

        frame = self._build_frame(msg, now_ns)
        if self.validate_before_publish:
            result = validators.validate_frame(frame)
            if not result.ok:
                self.get_logger().error(
                    "Skipping invalid raw frame: " + "; ".join(result.errors)
                )
                return

        output = String()
        output.data = raw_frame.to_json(frame)
        self.publisher.publish(output)

    def _warn_missing(self, message: str, now_ns: int) -> None:
        if now_ns - self._last_missing_warn_ns >= 2 * NS_PER_SEC:
            self.get_logger().warn(message)
            self._last_missing_warn_ns = now_ns

    def _build_frame(self, image_msg: Any, now_ns: int) -> Any:
        self._source_sequence += 1

        image_meta = image_to_meta(
            image_msg,
            topic=self.image_topic,
            image_sequence=self._source_sequence,
            data_transport=schema.DATA_TRANSPORT_ROS_TOPIC_REFERENCE,
        )
        camera_info = camera_info_to_raw(self._camera_info.msg, topic=self.camera_info_topic)

        if self.uav_state_msg_type == UAV_MSG_PX4_ODOM:
            uav_state = px4_vehicle_odometry_to_uav_state(
                self._uav_state.msg,
                source=self.uav_source,
                raw_frame_convention=self.raw_frame_convention,
                fallback_frame_id=self.uav_frame_id,
            )
        else:
            uav_state = odometry_to_uav_state(
                self._uav_state.msg,
                source=self.uav_source,
                raw_frame_convention=self.raw_frame_convention,
                fallback_frame_id=self.uav_frame_id,
            )

        target_fresh = (
            self.target_state_enabled
            and cache_is_fresh(
                self._target_state,
                _header_stamp_ns(image_msg),
                now_ns,
                self.max_target_state_age_sec,
            )
        )
        target_state = (
            odometry_to_target_state(
                self._target_state.msg,
                source=self.target_source,
                target_id=self.target_id,
                fallback_frame_id=self.target_frame_id,
            )
            if target_fresh
            else unavailable_target_state()
        )

        middleware_stamp = _stamp_from_ns(now_ns)
        middleware = raw_frame.MiddlewareMeta(
            primary_time_source=self.primary_time_source,
            producer=self.producer,
            environment=self.environment,
            stamp=(
                middleware_stamp
                if self.primary_time_source == schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
                else None
            ),
        )
        top_stamp = (
            image_meta.header.stamp
            if self.primary_time_source == schema.PRIMARY_TIME_SOURCE_IMAGE
            else middleware_stamp
        )

        return raw_frame.FirstRingRawFrame(
            schema_version=schema.RAW_FRAME_SCHEMA_VERSION,
            header=raw_frame.Header(stamp=top_stamp, frame_id=image_meta.header.frame_id),
            source_sequence=self._source_sequence,
            config_revision=self.config_revision,
            uav_state=uav_state,
            target_state=target_state,
            image=image_meta,
            camera_info=camera_info,
            middleware=middleware,
        )


def main(args: list[str] | None = None) -> None:
    if RAW_IMPORT_ERROR is not None:
        raise SystemExit(
            "reacquisition_first_ring_raw is unavailable. Build/source the "
            f"workspace first. Original import error: {RAW_IMPORT_ERROR}"
        )
    if ROS_IMPORT_ERROR is not None:
        raise SystemExit(
            "ROS 2 Python dependencies are unavailable. Run inside a ROS 2 "
            f"environment. Original import error: {ROS_IMPORT_ERROR}"
        )

    rclpy.init(args=args)
    node = RawFramePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
