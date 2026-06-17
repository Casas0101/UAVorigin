#!/usr/bin/env python3
"""Publish minimal odometry inputs for the first-ring Gazebo scene."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ImportError as exc:  # pragma: no cover - exercised outside ROS 2.
    rclpy = None
    Node = object
    Odometry = object
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


@dataclass(frozen=True)
class PlanarState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


def target_state_at(
    elapsed_s: float,
    *,
    motion: str = "static",
    center_x: float = 2.0,
    center_y: float = 1.0,
    z: float = 0.04,
    radius: float = 1.5,
    angular_rate: float = 0.2,
    line_vx: float = 0.2,
    line_vy: float = 0.0,
) -> PlanarState:
    if motion == "static":
        return PlanarState(center_x, center_y, z, 0.0, 0.0, 0.0)
    if motion == "line":
        return PlanarState(
            center_x + line_vx * elapsed_s,
            center_y + line_vy * elapsed_s,
            z,
            line_vx,
            line_vy,
            0.0,
        )
    if motion == "circle":
        theta = angular_rate * elapsed_s
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        vx = -radius * angular_rate * math.sin(theta)
        vy = radius * angular_rate * math.cos(theta)
        return PlanarState(x, y, z, vx, vy, 0.0)
    raise ValueError(f"unsupported target_motion: {motion}")


def fill_odometry(
    msg: Any,
    *,
    stamp: Any,
    frame_id: str,
    child_frame_id: str,
    state: PlanarState,
) -> Any:
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = state.x
    msg.pose.pose.position.y = state.y
    msg.pose.pose.position.z = state.z
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = state.vx
    msg.twist.twist.linear.y = state.vy
    msg.twist.twist.linear.z = state.vz
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = 0.0
    return msg


class GazeboStatePublisherNode(Node):
    """Publish synthetic scene ground-truth odometry for the raw frame node."""

    def __init__(self) -> None:
        super().__init__("first_ring_sim_state_node")

        self.declare_parameter("uav_odom_topic", "/uav/odom")
        self.declare_parameter("target_odom_topic", "/target/odom_gt")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("uav_child_frame_id", "downward_uav/base_link")
        self.declare_parameter("target_child_frame_id", "ground_target/base_link")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("uav_x", 0.0)
        self.declare_parameter("uav_y", 0.0)
        self.declare_parameter("uav_z", 10.0)
        self.declare_parameter("target_motion", "static")
        self.declare_parameter("target_x", 2.0)
        self.declare_parameter("target_y", 1.0)
        self.declare_parameter("target_z", 0.04)
        self.declare_parameter("target_radius", 1.5)
        self.declare_parameter("target_angular_rate", 0.2)
        self.declare_parameter("target_line_vx", 0.2)
        self.declare_parameter("target_line_vy", 0.0)

        self.uav_pub = self.create_publisher(
            Odometry, str(self.get_parameter("uav_odom_topic").value), 10
        )
        self.target_pub = self.create_publisher(
            Odometry, str(self.get_parameter("target_odom_topic").value), 10
        )

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.uav_child_frame_id = str(self.get_parameter("uav_child_frame_id").value)
        self.target_child_frame_id = str(self.get_parameter("target_child_frame_id").value)
        self.uav_state = PlanarState(
            float(self.get_parameter("uav_x").value),
            float(self.get_parameter("uav_y").value),
            float(self.get_parameter("uav_z").value),
            0.0,
            0.0,
            0.0,
        )
        self.target_motion = str(self.get_parameter("target_motion").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.target_radius = float(self.get_parameter("target_radius").value)
        self.target_angular_rate = float(self.get_parameter("target_angular_rate").value)
        self.target_line_vx = float(self.get_parameter("target_line_vx").value)
        self.target_line_vy = float(self.get_parameter("target_line_vy").value)

        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.start_time_ns = int(self.get_clock().now().nanoseconds)
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        elapsed_s = max(0.0, (int(now.nanoseconds) - self.start_time_ns) / 1_000_000_000)

        uav_msg = fill_odometry(
            Odometry(),
            stamp=stamp,
            frame_id=self.frame_id,
            child_frame_id=self.uav_child_frame_id,
            state=self.uav_state,
        )
        target_msg = fill_odometry(
            Odometry(),
            stamp=stamp,
            frame_id=self.frame_id,
            child_frame_id=self.target_child_frame_id,
            state=target_state_at(
                elapsed_s,
                motion=self.target_motion,
                center_x=self.target_x,
                center_y=self.target_y,
                z=self.target_z,
                radius=self.target_radius,
                angular_rate=self.target_angular_rate,
                line_vx=self.target_line_vx,
                line_vy=self.target_line_vy,
            ),
        )
        self.uav_pub.publish(uav_msg)
        self.target_pub.publish(target_msg)


def main(args: list[str] | None = None) -> None:
    if ROS_IMPORT_ERROR is not None:
        raise SystemExit(
            "ROS 2 Python dependencies are unavailable. Run inside a ROS 2 "
            f"environment. Original import error: {ROS_IMPORT_ERROR}"
        )

    rclpy.init(args=args)
    node = GazeboStatePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
