#!/usr/bin/env python3
"""Record FirstRingRawFrame std_msgs/String messages to JSONL."""

from __future__ import annotations

import json
import sys
from pathlib import Path


_DEV_RAW_PARENT = Path(__file__).resolve().parent.parent / "reacquisition_first_ring_raw"
if _DEV_RAW_PARENT.exists() and str(_DEV_RAW_PARENT) not in sys.path:
    sys.path.insert(0, str(_DEV_RAW_PARENT))

try:
    from reacquisition_first_ring_raw import validators
except ImportError as exc:  # pragma: no cover - only when package install is broken.
    validators = None
    RAW_IMPORT_ERROR = exc
else:
    RAW_IMPORT_ERROR = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - exercised outside ROS 2.
    rclpy = None
    Node = object
    String = object
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("true", "1", "yes", "y", "on"):
            return True
        if lowered in ("false", "0", "no", "n", "off"):
            return False
    return bool(value)


class RawFrameJsonlRecorderNode(Node):
    """Subscribe to raw frame JSON strings and append valid frames to JSONL."""

    def __init__(self) -> None:
        super().__init__("raw_frame_jsonl_recorder")

        self.declare_parameter("input_topic", "/reacquisition/first_ring/raw_frame")
        self.declare_parameter("output_path", "outputs/raw_frames/gazebo_raw_frames.jsonl")
        self.declare_parameter("overwrite", False)
        self.declare_parameter("validate", True)
        self.declare_parameter("flush_every", 1)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.overwrite = _as_bool(self.get_parameter("overwrite").value)
        self.validate = _as_bool(self.get_parameter("validate").value)
        self.flush_every = max(1, int(self.get_parameter("flush_every").value))
        self.count = 0

        if self.output_path.parent:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "w" if self.overwrite else "x"
        try:
            self.fp = self.output_path.open(mode, encoding="utf-8", newline="\n")
        except FileExistsError as exc:
            raise SystemExit(
                f"output file already exists: {self.output_path}. "
                "Relaunch with overwrite:=true or choose another output_path."
            ) from exc

        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self._on_msg,
            10,
        )
        self.get_logger().info(
            f"Recording {self.input_topic} to {self.output_path}"
        )

    def _on_msg(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Skipping non-JSON raw frame: {exc}")
            return

        if self.validate:
            result = validators.validate_frame_dict(payload)
            if not result.ok:
                self.get_logger().warn(
                    "Skipping invalid raw frame: " + "; ".join(result.errors)
                )
                return

        self.fp.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        self.fp.write("\n")
        self.count += 1
        if self.count % self.flush_every == 0:
            self.fp.flush()

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "fp") and not self.fp.closed:
                self.fp.flush()
                self.fp.close()
        finally:
            return super().destroy_node()


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
    node = RawFrameJsonlRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
