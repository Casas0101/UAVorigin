from __future__ import annotations

import json
import math
import sys
from pathlib import Path


SIM_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(SIM_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(SIM_PACKAGE_ROOT))

import gazebo_state_publisher_node as state_node
import raw_frame_jsonl_recorder_node as recorder_node


class _Nested:
    pass


def _odom_like():
    msg = _Nested()
    msg.header = _Nested()
    msg.pose = _Nested()
    msg.pose.pose = _Nested()
    msg.pose.pose.position = _Nested()
    msg.pose.pose.orientation = _Nested()
    msg.twist = _Nested()
    msg.twist.twist = _Nested()
    msg.twist.twist.linear = _Nested()
    msg.twist.twist.angular = _Nested()
    return msg


def test_static_target_state():
    state = state_node.target_state_at(10.0, motion="static")
    assert state.x == 2.0
    assert state.y == 1.0
    assert state.vx == 0.0
    assert state.vy == 0.0


def test_circle_target_state_has_tangent_velocity():
    state = state_node.target_state_at(
        math.pi / 2.0,
        motion="circle",
        center_x=0.0,
        center_y=0.0,
        radius=2.0,
        angular_rate=1.0,
    )
    assert abs(state.x) < 1e-9
    assert abs(state.y - 2.0) < 1e-9
    assert abs(state.vx + 2.0) < 1e-9
    assert abs(state.vy) < 1e-9


def test_line_target_state_moves_at_constant_velocity():
    state = state_node.target_state_at(
        5.0,
        motion="line",
        center_x=2.0,
        center_y=1.0,
        line_vx=0.2,
        line_vy=-0.1,
    )
    assert state.x == 3.0
    assert state.y == 0.5
    assert state.vx == 0.2
    assert state.vy == -0.1


def test_fill_odometry_sets_pose_and_twist():
    msg = _odom_like()
    stamp = object()
    filled = state_node.fill_odometry(
        msg,
        stamp=stamp,
        frame_id="map",
        child_frame_id="target",
        state=state_node.PlanarState(1.0, 2.0, 0.04, 0.1, 0.2, 0.0),
    )
    assert filled.header.stamp is stamp
    assert filled.header.frame_id == "map"
    assert filled.child_frame_id == "target"
    assert filled.pose.pose.position.x == 1.0
    assert filled.pose.pose.orientation.w == 1.0
    assert filled.twist.twist.linear.y == 0.2


def test_recorder_bool_strings_are_parsed_explicitly():
    assert recorder_node._as_bool("true") is True
    assert recorder_node._as_bool("false") is False
    assert recorder_node._as_bool("0") is False


def test_qgc_rectangle_mission_plan_is_valid_json():
    plan_path = SIM_PACKAGE_ROOT / "missions" / "px4_rectangle_10m_qgc.plan"
    payload = json.loads(plan_path.read_text(encoding="utf-8"))
    assert payload["fileType"] == "Plan"
    items = payload["mission"]["items"]
    assert len(items) == 6
    assert items[0]["command"] == 22
    assert all(item["params"][6] == 10 for item in items)
