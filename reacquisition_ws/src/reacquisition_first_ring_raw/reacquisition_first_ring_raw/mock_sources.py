"""mock_sources.py

只生成固定、可复现的 mock 原始数据.

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md

禁止:
- 读取真实相机;
- 调用 Gazebo / PX4 / ROS 2;
- 引入 numpy / opencv 等重型依赖;
- 进行坐标变换或时间插值.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

from . import raw_frame, schema


# --- 固定基线数据 --------------------------------------------------------

# UAV 在地面坐标系 (map) 中悬停在 (0, 0, 10) 的简单基线.
_UAV_BASELINE_POSITION = (0.0, 0.0, 10.0)
_UAV_BASELINE_ORIENTATION = (0.0, 0.0, 0.0, 1.0)
_UAV_BASELINE_LINEAR_VEL = (0.0, 0.0, 0.0)
_UAV_BASELINE_ANGULAR_VEL = (0.0, 0.0, 0.0)

# 目标在地面上方 (0, 0) 缓慢移动, 仅用于联调, 不参与算法.
_TARGET_BASELINE_POSITION = (2.0, 1.0, 0.0)
_TARGET_BASELINE_LINEAR_VEL = (0.5, 0.0, 0.0)

# 相机内参基线 (与生成指南 § 5.1 示例一致).
_CAMERA_BASELINE_K = (
    500.0, 0.0, 640.0,
    0.0, 500.0, 360.0,
    0.0, 0.0, 1.0,
)
_CAMERA_BASELINE_R = (
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
)
_CAMERA_BASELINE_P = (
    500.0, 0.0, 640.0, 0.0,
    0.0, 500.0, 360.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
)

# 默认 mock 时间起点 (相对任意固定基准, 不依赖 wall clock).
_DEFAULT_BASE_NS = 1_000_000_000  # 1 秒, 便于人阅读


# --- 子结构构造 ----------------------------------------------------------


def make_uav_state(
    *,
    sequence: int = 0,
    stamp: Optional[raw_frame.Stamp] = None,
    position: Sequence[float] = _UAV_BASELINE_POSITION,
    orientation: Sequence[float] = _UAV_BASELINE_ORIENTATION,
    linear_velocity: Sequence[float] = _UAV_BASELINE_LINEAR_VEL,
    angular_velocity: Sequence[float] = _UAV_BASELINE_ANGULAR_VEL,
    frame_id: str = schema.DEFAULT_GLOBAL_FRAME_ID,
    coord_convention: str = schema.COORD_CONVENTION_ENU_FLU,
    source: str = schema.SRC_MOCK_UAV_STATE,
) -> raw_frame.UavStateRaw:
    """构造一个固定的 UAV 原始状态."""

    if stamp is None:
        # 默认从 sequence 推导一个递增时间戳, 保证单调.
        stamp = raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + sequence * 33_000_000)  # ~30Hz

    if coord_convention not in schema.ALLOWED_COORD_CONVENTIONS:
        raise ValueError(f"unsupported coord_convention: {coord_convention}")

    return raw_frame.UavStateRaw(
        stamp=stamp,
        frame_id=frame_id,
        raw_frame_convention=coord_convention,
        position=raw_frame.Vector3(float(position[0]), float(position[1]), float(position[2])),
        orientation=raw_frame.Quaternion(
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
            float(orientation[3]),
        ),
        linear_velocity=raw_frame.Vector3(
            float(linear_velocity[0]),
            float(linear_velocity[1]),
            float(linear_velocity[2]),
        ),
        angular_velocity=raw_frame.Vector3(
            float(angular_velocity[0]),
            float(angular_velocity[1]),
            float(angular_velocity[2]),
        ),
        source=source,
    )


def make_target_state(
    *,
    sequence: int = 0,
    has_target_state: bool = True,
    stamp: Optional[raw_frame.Stamp] = None,
    position: Sequence[float] = _TARGET_BASELINE_POSITION,
    linear_velocity: Sequence[float] = _TARGET_BASELINE_LINEAR_VEL,
    target_id: str = "target_0",
    frame_id: str = schema.DEFAULT_GLOBAL_FRAME_ID,
    source: str = schema.SRC_MOCK_TARGET_STATE,
) -> raw_frame.TargetStateRaw:
    """构造一个固定的目标原始状态.

    has_target_state=False 时, 位置/速度等字段必须全部为 None, 不得伪造.
    """

    if not has_target_state:
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

    if stamp is None:
        stamp = raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + sequence * 33_000_000)

    return raw_frame.TargetStateRaw(
        has_target_state=True,
        target_id=target_id,
        stamp=stamp,
        frame_id=frame_id,
        position=raw_frame.Vector3(
            float(position[0]),
            float(position[1]),
            float(position[2]),
        ),
        orientation=raw_frame.Quaternion(0.0, 0.0, 0.0, 1.0),
        linear_velocity=raw_frame.Vector3(
            float(linear_velocity[0]),
            float(linear_velocity[1]),
            float(linear_velocity[2]),
        ),
        angular_velocity=raw_frame.Vector3(0.0, 0.0, 0.0),
        source=source,
    )


def make_image_meta(
    *,
    sequence: int = 0,
    stamp: Optional[raw_frame.Stamp] = None,
    image_sequence: Optional[int] = None,
    height: int = schema.DEFAULT_IMAGE_HEIGHT,
    width: int = schema.DEFAULT_IMAGE_WIDTH,
    encoding: str = schema.IMAGE_ENCODING_RGB8,
    is_bigendian: int = 0,
    step: int = schema.DEFAULT_IMAGE_STEP,
    image_topic: str = schema.DEFAULT_IMAGE_TOPIC,
    camera_frame_id: str = schema.DEFAULT_CAMERA_OPTICAL_FRAME_ID,
    data_transport: str = schema.DATA_TRANSPORT_METADATA_ONLY,
    data_base64: Optional[str] = None,
) -> raw_frame.ImageMetaRaw:
    """构造图像元数据. 默认不携带像素二进制."""

    if image_sequence is None:
        image_sequence = sequence

    if stamp is None:
        # 图像时间与 UAV 时间对齐, 方便主时间戳直接取自图像.
        stamp = raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + sequence * 33_000_000)

    return raw_frame.ImageMetaRaw(
        header=raw_frame.Header(stamp=stamp, frame_id=camera_frame_id),
        image_topic=image_topic,
        image_sequence=int(image_sequence),
        height=int(height),
        width=int(width),
        encoding=encoding,
        is_bigendian=int(is_bigendian),
        step=int(step),
        data_transport=data_transport,
        data_base64=data_base64,
    )


def make_camera_info(
    *,
    sequence: int = 0,
    stamp: Optional[raw_frame.Stamp] = None,
    height: int = schema.DEFAULT_IMAGE_HEIGHT,
    width: int = schema.DEFAULT_IMAGE_WIDTH,
    distortion_model: str = schema.DISTORTION_MODEL_PLUMB_BOB,
    d: Optional[Sequence[float]] = None,
    k: Sequence[float] = _CAMERA_BASELINE_K,
    r: Sequence[float] = _CAMERA_BASELINE_R,
    p: Sequence[float] = _CAMERA_BASELINE_P,
    camera_info_topic: str = schema.DEFAULT_CAMERA_INFO_TOPIC,
    camera_frame_id: str = schema.DEFAULT_CAMERA_OPTICAL_FRAME_ID,
    source: Optional[str] = None,
) -> raw_frame.CameraInfoRaw:
    """构造 CameraInfo 元数据. 默认参数与生成指南 § 5.1 示例一致."""

    if d is None:
        d = ()

    if source is not None and source != schema.SRC_MOCK_CAMERA_INFO:
        raise ValueError("CameraInfoRaw does not carry a source field in schema 0.1")

    if stamp is None:
        stamp = raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + sequence * 33_000_000)

    if distortion_model not in schema.ALLOWED_DISTORTION_MODELS:
        raise ValueError(f"unsupported distortion_model: {distortion_model}")

    return raw_frame.CameraInfoRaw(
        header=raw_frame.Header(stamp=stamp, frame_id=camera_frame_id),
        camera_info_topic=camera_info_topic,
        height=int(height),
        width=int(width),
        distortion_model=distortion_model,
        d=[float(x) for x in d],
        k=[float(x) for x in k],
        r=[float(x) for x in r],
        p=[float(x) for x in p],
    )


def make_middleware(
    *,
    primary_time_source: str = schema.PRIMARY_TIME_SOURCE_IMAGE,
    stamp: Optional[raw_frame.Stamp] = None,
    producer: str = schema.PRODUCER_WINDOWS_MOCK,
    environment: str = schema.ENVIRONMENT_WINDOWS_NO_SIM_NO_CPP,
) -> raw_frame.MiddlewareMeta:
    """构造中间件自身元数据."""

    if primary_time_source not in schema.ALLOWED_PRIMARY_TIME_SOURCES:
        raise ValueError(f"unsupported primary_time_source: {primary_time_source}")

    return raw_frame.MiddlewareMeta(
        primary_time_source=primary_time_source,
        producer=producer,
        environment=environment,
        stamp=stamp,
    )


# --- 帧级构造 ------------------------------------------------------------


def make_raw_frame(
    *,
    sequence: int = 0,
    config_revision: int = 0,
    has_target_state: bool = True,
    target_state: Optional[raw_frame.TargetStateRaw] = None,
    uav_state: Optional[raw_frame.UavStateRaw] = None,
    image: Optional[raw_frame.ImageMetaRaw] = None,
    camera_info: Optional[raw_frame.CameraInfoRaw] = None,
    middleware: Optional[raw_frame.MiddlewareMeta] = None,
) -> raw_frame.FirstRingRawFrame:
    """构造一帧完整的 FirstRingRawFrame.

    主时间戳默认取自 image.header.stamp; 若该帧没有图像, 则取自 middleware.stamp.
    """

    if image is None:
        image = make_image_meta(sequence=sequence)
    if uav_state is None:
        uav_state = make_uav_state(sequence=sequence)
    if camera_info is None:
        camera_info = make_camera_info(sequence=sequence)
    if target_state is None:
        target_state = make_target_state(sequence=sequence, has_target_state=has_target_state)

    if middleware is None:
        middleware = make_middleware()
    elif middleware.primary_time_source not in schema.ALLOWED_PRIMARY_TIME_SOURCES:
        raise ValueError(f"unsupported primary_time_source: {middleware.primary_time_source}")

    if middleware.primary_time_source == schema.PRIMARY_TIME_SOURCE_IMAGE:
        top_stamp = image.header.stamp
    else:
        if middleware.stamp is None:
            middleware = raw_frame.MiddlewareMeta(
                primary_time_source=middleware.primary_time_source,
                producer=middleware.producer,
                environment=middleware.environment,
                stamp=raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + sequence * 33_000_000),
            )
        top_stamp = middleware.stamp

    top_header = raw_frame.Header(stamp=top_stamp, frame_id=image.header.frame_id)

    return raw_frame.FirstRingRawFrame(
        schema_version=schema.RAW_FRAME_SCHEMA_VERSION,
        header=top_header,
        source_sequence=int(sequence),
        config_revision=int(config_revision),
        uav_state=uav_state,
        target_state=target_state,
        image=image,
        camera_info=camera_info,
        middleware=middleware,
    )


def make_sequence(
    count: int,
    *,
    start_sequence: int = 0,
    config_revision: int = 0,
    has_target: bool = True,
    primary_time_source: str = schema.PRIMARY_TIME_SOURCE_IMAGE,
) -> List[raw_frame.FirstRingRawFrame]:
    """连续生成 count 帧, source_sequence 单调递增, 时间戳单调递增."""

    if count < 0:
        raise ValueError("count must be non-negative")
    if primary_time_source not in schema.ALLOWED_PRIMARY_TIME_SOURCES:
        raise ValueError(f"unsupported primary_time_source: {primary_time_source}")

    frames: List[raw_frame.FirstRingRawFrame] = []
    for i in range(count):
        seq = start_sequence + i
        mw_stamp = (
            raw_frame.stamp_from_ns(_DEFAULT_BASE_NS + seq * 33_000_000)
            if primary_time_source == schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
            else None
        )
        frame = make_raw_frame(
            sequence=seq,
            config_revision=config_revision,
            has_target_state=has_target,
            middleware=make_middleware(
                primary_time_source=primary_time_source,
                stamp=mw_stamp,
            ),
        )
        frames.append(frame)

    return frames


__all__ = [
    "make_uav_state",
    "make_target_state",
    "make_image_meta",
    "make_camera_info",
    "make_middleware",
    "make_raw_frame",
    "make_sequence",
]
