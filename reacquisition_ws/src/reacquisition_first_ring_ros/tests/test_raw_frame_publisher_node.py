from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


ROS_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROS_PACKAGE_ROOT.parent
RAW_PACKAGE_ROOT = SRC_ROOT / "reacquisition_first_ring_raw"
for path in (ROS_PACKAGE_ROOT, RAW_PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import raw_frame_publisher_node as node
from reacquisition_first_ring_raw import raw_frame, schema, validators


def _stamp(sec: int = 1, nanosec: int = 0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _header(frame_id: str = "map", sec: int = 1, nanosec: int = 0):
    return SimpleNamespace(stamp=_stamp(sec, nanosec), frame_id=frame_id)


def _vec(x: float, y: float, z: float):
    return SimpleNamespace(x=x, y=y, z=z)


def _quat(x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
    return SimpleNamespace(x=x, y=y, z=z, w=w)


def _odom(frame_id: str = "map"):
    return SimpleNamespace(
        header=_header(frame_id),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=_vec(1.0, 2.0, 3.0),
                orientation=_quat(),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=_vec(0.1, 0.2, 0.3),
                angular=_vec(0.01, 0.02, 0.03),
            )
        ),
    )


def _image():
    return SimpleNamespace(
        header=_header("camera_optical_frame"),
        height=720,
        width=1280,
        encoding="rgb8",
        is_bigendian=0,
        step=3840,
    )


def _camera_info():
    return SimpleNamespace(
        header=_header("camera_optical_frame"),
        height=720,
        width=1280,
        distortion_model="plumb_bob",
        d=[],
        k=[500.0, 0.0, 640.0, 0.0, 500.0, 360.0, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[500.0, 0.0, 640.0, 0.0, 0.0, 500.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


def test_ros_helpers_build_valid_raw_frame():
    image_meta = node.image_to_meta(
        _image(),
        topic="/camera/image_raw",
        image_sequence=1,
        data_transport=schema.DATA_TRANSPORT_ROS_TOPIC_REFERENCE,
    )
    camera_info = node.camera_info_to_raw(_camera_info(), topic="/camera/camera_info")
    uav_state = node.odometry_to_uav_state(
        _odom(),
        source=schema.SRC_GAZEBO_MODEL_STATE,
        raw_frame_convention=schema.COORD_CONVENTION_ENU_FLU,
        fallback_frame_id=schema.DEFAULT_GLOBAL_FRAME_ID,
    )
    target_state = node.odometry_to_target_state(
        _odom(),
        source=schema.SRC_GAZEBO_GROUND_TRUTH,
        target_id="target_0",
        fallback_frame_id=schema.DEFAULT_GLOBAL_FRAME_ID,
    )
    middleware = raw_frame.MiddlewareMeta(
        primary_time_source=schema.PRIMARY_TIME_SOURCE_IMAGE,
        producer="ros2_raw_frame_publisher",
        environment="wsl2_ros2_gazebo_px4",
    )
    frame = raw_frame.FirstRingRawFrame(
        schema_version=schema.RAW_FRAME_SCHEMA_VERSION,
        header=raw_frame.Header(stamp=image_meta.header.stamp, frame_id=image_meta.header.frame_id),
        source_sequence=1,
        config_revision=0,
        uav_state=uav_state,
        target_state=target_state,
        image=image_meta,
        camera_info=camera_info,
        middleware=middleware,
    )

    result = validators.validate_frame(frame)
    assert result.ok, result.errors


def test_px4_vehicle_odometry_mapping_preserves_raw_convention():
    msg = SimpleNamespace(
        timestamp=2_000_000,
        timestamp_sample=1_000_000,
        position=[1.0, 2.0, -3.0],
        q=[1.0, 0.0, 0.0, 0.0],
        velocity=[0.1, 0.2, 0.3],
        angular_velocity=[0.01, 0.02, 0.03],
    )
    uav_state = node.px4_vehicle_odometry_to_uav_state(
        msg,
        source=schema.SRC_PX4_VEHICLE_ODOMETRY,
        raw_frame_convention=schema.COORD_CONVENTION_NED_FRD,
        fallback_frame_id="px4_ned",
    )

    assert uav_state.stamp.sec == 1
    assert uav_state.stamp.nanosec == 0
    assert uav_state.frame_id == "px4_ned"
    assert uav_state.raw_frame_convention == schema.COORD_CONVENTION_NED_FRD
    assert uav_state.position.z == -3.0
    assert uav_state.orientation.w == 1.0


def test_cache_freshness_accepts_arrival_time_when_stamp_domains_differ():
    cache = node.CachedMessage(msg=object(), stamp_ns=10, arrival_ns=1_000)
    assert node.cache_is_fresh(cache, sample_stamp_ns=10_000_000_000_000, now_ns=1_100, max_age_sec=1.0)


def test_px4_uav_cache_uses_arrival_time_for_freshness():
    msg = SimpleNamespace(timestamp=20_000_000, timestamp_sample=10_000_000)
    assert node._uav_cache_stamp_ns(msg, node.UAV_MSG_PX4_ODOM) is None


def test_launch_bool_strings_are_parsed_explicitly():
    assert node._as_bool("true") is True
    assert node._as_bool("false") is False
    assert node._as_bool("0") is False
