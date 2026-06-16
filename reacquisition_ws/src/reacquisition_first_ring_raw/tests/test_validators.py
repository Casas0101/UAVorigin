"""test_validators.py

覆盖 validators 的格式级校验:
- 缺少顶层必需字段时返回失败
- has_target_state=false 时不伪造目标位置
- JSONL 写入后每一行可被 validate_frame 通过
"""

from __future__ import annotations

import json

import pytest

from reacquisition_first_ring_raw import (
    jsonl_writer,
    mock_sources,
    raw_frame,
    schema,
    validators,
)


def test_validate_frame_accepts_default_mock():
    frame = mock_sources.make_raw_frame(sequence=0)
    result = validators.validate_frame(frame)
    assert result.ok, result.errors


def test_validate_frame_rejects_missing_top_level_field():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    del payload["uav_state"]
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("uav_state" in e for e in result.errors)


def test_validate_frame_rejects_forbidden_field():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    payload["bbox_left"] = 1
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("forbidden field" in e for e in result.errors)


def test_validate_frame_rejects_nested_forbidden_field():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    payload["target_state"]["ground_position"] = {"x": 1.0, "y": 2.0, "z": 0.0}
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("target_state.ground_position" in e for e in result.errors)


def test_validate_frame_rejects_primary_time_source_mismatch():
    frame = mock_sources.make_sequence(
        count=1, primary_time_source=schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
    )[0]
    payload = raw_frame.to_dict(frame)
    payload["header"]["stamp"] = {"sec": 123, "nanosec": 456}
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("middleware.stamp" in e for e in result.errors)


def test_validate_no_target_state_does_not_carry_position():
    frame = mock_sources.make_raw_frame(sequence=0, has_target_state=False)
    result = validators.validate_frame(frame)
    assert result.ok, result.errors


def test_validate_no_target_state_with_position_fails():
    frame = mock_sources.make_raw_frame(sequence=0, has_target_state=False)
    payload = raw_frame.to_dict(frame)
    # 模拟人为在 has_target_state=false 时塞了伪造 position
    payload["target_state"]["position"] = {"x": 1.0, "y": 2.0, "z": 3.0}
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("has_target_state=false" in e for e in result.errors)


def test_validate_rejects_wrong_schema_version():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    payload["schema_version"] = "9.9"
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("schema_version" in e for e in result.errors)


def test_validate_rejects_non_int_stamp():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    payload["header"]["stamp"]["sec"] = 1.5
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("stamp.sec" in e for e in result.errors)


def test_validate_rejects_empty_frame_id():
    frame = mock_sources.make_raw_frame(sequence=0)
    payload = raw_frame.to_dict(frame)
    payload["uav_state"]["frame_id"] = ""
    result = validators.validate_frame_dict(payload)
    assert not result.ok
    assert any("uav_state.frame_id" in e for e in result.errors)


def test_validate_sequence_passes_for_monotonic_mock(tmp_path: Path):
    frames = mock_sources.make_sequence(count=5)
    result = validators.validate_sequence(frames)
    assert result.ok, result.errors

    out = tmp_path / "frames.jsonl"
    jsonl_writer.write_jsonl(frames, out)
    payloads = jsonl_writer.read_jsonl(out)
    result2 = validators.validate_sequence(payloads)
    assert result2.ok, result2.errors


def test_validate_sequence_detects_non_monotonic_source_sequence():
    frames = mock_sources.make_sequence(count=3)
    payloads = [raw_frame.to_dict(f) for f in frames]
    payloads[1]["source_sequence"] = payloads[0]["source_sequence"]  # 故意制造非递增
    result = validators.validate_sequence(payloads)
    assert not result.ok
    assert any("source_sequence" in e for e in result.errors)


def test_validate_rejects_invalid_json_string():
    result = validators.validate_frame("not-json")
    assert not result.ok
    assert any("invalid JSON" in e for e in result.errors)
