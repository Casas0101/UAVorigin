"""test_schema.py

schema.py 只导出常量, 不应 import ROS / PX4 / Gazebo / OpenCV / numpy / C++ 模块.
"""

from __future__ import annotations

import sys


_FORBIDDEN_MODULES = (
    "rclpy",
    "rcl",
    "px4_msgs",
    "px4",
    "gazebo",
    "cv2",
    "numpy",
    "Cython",
    "setuptools._distutils",  # C++ 编译相关
)


def test_schema_version_constant():
    from reacquisition_first_ring_raw import schema
    assert schema.RAW_FRAME_SCHEMA_VERSION == "0.1"


def test_topic_and_ros_type_constants():
    from reacquisition_first_ring_raw import schema
    assert schema.RAW_FRAME_TOPIC == "/reacquisition/first_ring/raw_frame"
    assert schema.RAW_FRAME_ROS_TYPE == "std_msgs/msg/String"
    assert schema.RAW_FRAME_ENCODING == "JSON"


def test_coord_conventions_contains_expected_values():
    from reacquisition_first_ring_raw import schema
    assert schema.COORD_CONVENTION_ENU_FLU in schema.ALLOWED_COORD_CONVENTIONS
    assert schema.COORD_CONVENTION_NED_FRD in schema.ALLOWED_COORD_CONVENTIONS
    assert schema.COORD_CONVENTION_UNKNOWN in schema.ALLOWED_COORD_CONVENTIONS


def test_data_sources_are_present():
    from reacquisition_first_ring_raw import schema
    for s in (
        schema.SRC_MOCK_UAV_STATE,
        schema.SRC_MOCK_TARGET_STATE,
        schema.SRC_MOCK_IMAGE,
        schema.SRC_MOCK_CAMERA_INFO,
        schema.SRC_PX4_VEHICLE_ODOMETRY,
        schema.SRC_GAZEBO_GROUND_TRUTH,
        schema.SRC_UNAVAILABLE,
    ):
        assert isinstance(s, str) and s


def test_required_top_level_fields_contains_documented_keys():
    from reacquisition_first_ring_raw import schema
    for k in (
        "schema_version",
        "header",
        "source_sequence",
        "config_revision",
        "uav_state",
        "target_state",
        "image",
        "camera_info",
        "middleware",
    ):
        assert k in schema.REQUIRED_TOP_LEVEL_FIELDS


def test_forbidden_output_fields_match_guide():
    from reacquisition_first_ring_raw import schema
    expected = {
        "bbox_left", "bbox_top", "bbox_right", "bbox_bottom",
        "center_u", "center_v", "foot_u", "foot_v",
        "ground_position", "ground_covariance_xy",
        "reliability", "blur_risk", "edge_distance_px",
    }
    assert set(schema.FORBIDDEN_OUTPUT_FIELDS) == expected


def test_schema_module_has_no_forbidden_imports():
    """schema.py 不得触发任何禁用模块的导入."""

    for mod in _FORBIDDEN_MODULES:
        assert mod not in sys.modules, (
            f"schema indirectly imported forbidden module: {mod}"
        )


def test_package_root_has_no_forbidden_imports():
    """import 包根时不应触发任何禁用模块."""

    from reacquisition_first_ring_raw import schema, raw_frame, mock_sources, jsonl_writer, validators  # noqa: F401
    for mod in _FORBIDDEN_MODULES:
        assert mod not in sys.modules, (
            f"package root indirectly imported forbidden module: {mod}"
        )
