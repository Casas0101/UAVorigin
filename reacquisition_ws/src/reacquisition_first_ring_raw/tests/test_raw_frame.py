"""test_raw_frame.py

覆盖:
- FirstRingRawFrame.to_json() 可生成标准 JSON
- from_json(to_json(frame)) 字段保持一致
- 不出现禁止字段
"""

from __future__ import annotations

import json

import pytest

from reacquisition_first_ring_raw import (
    FirstRingRawFrame,
    mock_sources,
    raw_frame,
    schema,
)


def test_to_json_generates_valid_json():
    frame = mock_sources.make_raw_frame(sequence=0)
    text = raw_frame.to_json(frame)
    parsed = json.loads(text)  # 标准 JSON 解析
    assert isinstance(parsed, dict)


def test_roundtrip_preserves_all_top_level_fields():
    frame = mock_sources.make_raw_frame(sequence=42, config_revision=7)
    text = raw_frame.to_json(frame)
    restored = raw_frame.from_json(text)
    restored_dict = raw_frame.to_dict(restored)
    original_dict = raw_frame.to_dict(frame)

    assert restored_dict == original_dict
    assert restored.schema_version == schema.RAW_FRAME_SCHEMA_VERSION
    assert restored.source_sequence == 42
    assert restored.config_revision == 7
    assert restored.uav_state.frame_id == schema.DEFAULT_GLOBAL_FRAME_ID
    assert restored.image.image_sequence == 42
    assert restored.camera_info.distortion_model == schema.DISTORTION_MODEL_PLUMB_BOB


def test_to_json_does_not_contain_forbidden_fields():
    frame = mock_sources.make_raw_frame(sequence=1)
    text = raw_frame.to_json(frame)
    payload = json.loads(text)

    for forbidden in schema.FORBIDDEN_OUTPUT_FIELDS:
        assert forbidden not in payload, f"forbidden field leaked: {forbidden}"
        # 嵌套字段也不应出现
        for sub in (
            payload.get("uav_state", {}),
            payload.get("target_state", {}),
            payload.get("image", {}),
            payload.get("camera_info", {}),
            payload.get("middleware", {}),
        ):
            assert forbidden not in sub


def test_roundtrip_via_dict_helper():
    frame = mock_sources.make_raw_frame(sequence=5)
    d = raw_frame.to_dict(frame)
    restored = raw_frame.from_dict(d)
    assert raw_frame.to_dict(restored) == d


def test_no_target_state_does_not_carry_position():
    frame = mock_sources.make_raw_frame(sequence=2, has_target_state=False)
    text = raw_frame.to_json(frame)
    payload = json.loads(text)
    tgt = payload["target_state"]

    assert tgt["has_target_state"] is False
    assert tgt.get("position") is None
    assert tgt.get("orientation") is None
    assert tgt.get("linear_velocity") is None
    assert tgt.get("angular_velocity") is None
    assert tgt.get("stamp") is None
    assert tgt.get("frame_id") is None
    assert tgt.get("source") == schema.SRC_UNAVAILABLE


def test_roundtrip_preserves_no_target_state():
    frame = mock_sources.make_raw_frame(sequence=3, has_target_state=False)
    restored = raw_frame.from_json(raw_frame.to_json(frame))
    assert restored.target_state.has_target_state is False
    assert restored.target_state.position is None
    assert restored.target_state.linear_velocity is None
    assert restored.target_state.source == schema.SRC_UNAVAILABLE


def test_top_header_stamp_uses_image_time_by_default():
    frame = mock_sources.make_raw_frame(sequence=10)
    assert frame.header.stamp == frame.image.header.stamp


def test_primary_time_source_image_yields_image_top_header():
    frame = mock_sources.make_raw_frame(sequence=0)
    assert frame.middleware.primary_time_source == schema.PRIMARY_TIME_SOURCE_IMAGE
    assert frame.header.stamp == frame.image.header.stamp


def test_primary_time_source_middleware_overrides_top_header():
    frames = mock_sources.make_sequence(
        count=3, primary_time_source=schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
    )
    for i, f in enumerate(frames):
        assert f.middleware.primary_time_source == schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
        assert f.middleware.stamp is not None
        assert f.header.stamp == f.middleware.stamp
        # 严格递增
        if i > 0:
            prev_ns = (
                frames[i - 1].middleware.stamp.sec * 1_000_000_000
                + frames[i - 1].middleware.stamp.nanosec
            )
            cur_ns = f.middleware.stamp.sec * 1_000_000_000 + f.middleware.stamp.nanosec
            assert cur_ns > prev_ns


def test_make_raw_frame_respects_explicit_middleware_time_source():
    mw_stamp = raw_frame.Stamp(sec=99, nanosec=123)
    middleware = mock_sources.make_middleware(
        primary_time_source=schema.PRIMARY_TIME_SOURCE_MIDDLEWARE,
        stamp=mw_stamp,
    )
    frame = mock_sources.make_raw_frame(sequence=0, middleware=middleware)
    assert frame.header.stamp == mw_stamp
    assert frame.middleware.stamp == mw_stamp


def test_make_sequence_rejects_unknown_primary_time_source():
    with pytest.raises(ValueError):
        mock_sources.make_sequence(count=1, primary_time_source="bad_source")


def test_make_camera_info_rejects_unsupported_source_argument():
    with pytest.raises(ValueError):
        mock_sources.make_camera_info(source="custom_camera_info")


def test_stamp_from_ns_round_trip():
    s = raw_frame.stamp_from_ns(2_500_000_000 + 123_456_789)
    assert s.sec == 2
    assert s.nanosec == 623_456_789


def test_stamp_from_ns_rejects_negative():
    with pytest.raises(ValueError):
        raw_frame.stamp_from_ns(-1)
