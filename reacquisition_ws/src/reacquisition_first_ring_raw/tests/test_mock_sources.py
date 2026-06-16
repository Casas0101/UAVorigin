"""test_mock_sources.py

覆盖:
- 单帧 mock
- 多帧 mock, source_sequence 单调递增, 时间戳递增
- 无目标状态场景
- 图像元数据变化场景
- 固定可复现
"""

from __future__ import annotations

from reacquisition_first_ring_raw import (
    mock_sources,
    raw_frame,
    schema,
)


def _ts_ns(stamp: raw_frame.Stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def test_make_raw_frame_defaults_are_valid():
    frame = mock_sources.make_raw_frame(sequence=0)
    assert isinstance(frame, raw_frame.FirstRingRawFrame)
    assert frame.schema_version == schema.RAW_FRAME_SCHEMA_VERSION
    assert frame.source_sequence == 0
    assert frame.uav_state.source == schema.SRC_MOCK_UAV_STATE
    assert frame.target_state.source == schema.SRC_MOCK_TARGET_STATE


def test_make_sequence_strictly_monotonic():
    frames = mock_sources.make_sequence(count=10)
    seqs = [f.source_sequence for f in frames]
    assert seqs == list(range(10))

    # image header.stamp 单调递增
    stamps = [_ts_ns(f.image.header.stamp) for f in frames]
    assert all(stamps[i] < stamps[i + 1] for i in range(len(stamps) - 1))

    # uav_state.stamp 单调递增
    uav_stamps = [_ts_ns(f.uav_state.stamp) for f in frames]
    assert all(uav_stamps[i] < uav_stamps[i + 1] for i in range(len(uav_stamps) - 1))


def test_make_sequence_with_no_target():
    frames = mock_sources.make_sequence(count=4, has_target=False)
    for f in frames:
        assert f.target_state.has_target_state is False
        assert f.target_state.position is None
        assert f.target_state.linear_velocity is None
        assert f.target_state.source == schema.SRC_UNAVAILABLE


def test_make_sequence_varying_image_meta():
    frames = mock_sources.make_sequence(count=3)
    # 默认 mock 会在生成函数里给每个 frame 注入不同的 image_sequence
    image_seqs = [f.image.image_sequence for f in frames]
    assert image_seqs == [0, 1, 2]
    # 图像尺寸保持不变
    for f in frames:
        assert f.image.height == schema.DEFAULT_IMAGE_HEIGHT
        assert f.image.width == schema.DEFAULT_IMAGE_WIDTH


def test_make_raw_frame_is_deterministic_for_fixed_seed_like_input():
    """mock 数据应当可复现: 相同 sequence 得到相同的 UAV/目标位置."""

    a = mock_sources.make_raw_frame(sequence=5)
    b = mock_sources.make_raw_frame(sequence=5)
    assert raw_frame.to_dict(a) == raw_frame.to_dict(b)


def test_make_sequence_with_start_sequence():
    frames = mock_sources.make_sequence(count=3, start_sequence=100)
    assert [f.source_sequence for f in frames] == [100, 101, 102]


def test_make_sequence_negative_count_raises():
    import pytest
    with pytest.raises(ValueError):
        mock_sources.make_sequence(count=-1)


def test_make_uav_state_rejects_unknown_coord_convention():
    import pytest
    with pytest.raises(ValueError):
        mock_sources.make_uav_state(coord_convention="MAGIC_FRAME")


def test_make_camera_info_rejects_unknown_distortion_model():
    import pytest
    with pytest.raises(ValueError):
        mock_sources.make_camera_info(distortion_model="fisheye_extreme")


def test_image_meta_step_matches_default_rgb8_layout():
    img = mock_sources.make_image_meta(sequence=0)
    assert img.step == schema.DEFAULT_IMAGE_STEP
    assert img.encoding == schema.IMAGE_ENCODING_RGB8
    assert img.is_bigendian == 0
