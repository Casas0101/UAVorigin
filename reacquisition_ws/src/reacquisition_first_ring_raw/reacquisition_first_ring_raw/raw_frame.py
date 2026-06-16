"""raw_frame.py

使用 Python dataclass 定义原始数据结构, 提供 JSON 序列化/反序列化接口.

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md

不允许实现的内容:
- 检测 / 反投影 / 滤波 / 预测 / 控制等算法;
- 坐标转换或时间插值;
- 任何 ROS 2 / PX4 / Gazebo / OpenCV / numpy 依赖.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, List, Optional

from . import schema


# --- 基础原子类型 --------------------------------------------------------


@dataclass(frozen=True)
class Stamp:
    """ROS 风格时间戳: sec + nanosec."""

    sec: int
    nanosec: int


@dataclass(frozen=True)
class Header:
    """ROS 风格消息头."""

    stamp: Stamp
    frame_id: str


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


# --- UAV 自身状态原始数据 -------------------------------------------------


@dataclass(frozen=True)
class UavStateRaw:
    """UAV 自身状态原始字段, 不做任何坐标变换或插值."""

    stamp: Stamp
    frame_id: str
    raw_frame_convention: str
    position: Vector3
    orientation: Quaternion
    linear_velocity: Vector3
    angular_velocity: Vector3
    source: str


# --- 目标原始状态 ---------------------------------------------------------


@dataclass(frozen=True)
class TargetStateRaw:
    """目标状态原始字段.

    has_target_state=False 时, 不应伪造目标位置/速度, 仅保留 has_target_state /
    target_id / source 等元数据.
    """

    has_target_state: bool
    target_id: str
    stamp: Optional[Stamp]
    frame_id: Optional[str]
    position: Optional[Vector3]
    orientation: Optional[Quaternion]
    linear_velocity: Optional[Vector3]
    angular_velocity: Optional[Vector3]
    source: str


# --- 图像元数据 (不携带像素数据) -----------------------------------------


@dataclass(frozen=True)
class ImageMetaRaw:
    """图像消息元数据. 不携带像素二进制, 只携带追踪和回放所需字段."""

    header: Header
    image_topic: str
    image_sequence: int
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data_transport: str
    data_base64: Optional[str] = None  # 仅 debug 模式使用


# --- CameraInfo 元数据 ---------------------------------------------------


@dataclass(frozen=True)
class CameraInfoRaw:
    """相机内参/畸变参数原始字段, 仅原样转发."""

    header: Header
    camera_info_topic: str
    height: int
    width: int
    distortion_model: str
    d: List[float]
    k: List[float]
    r: List[float]
    p: List[float]


# --- 中间件自身元数据 -----------------------------------------------------


@dataclass(frozen=True)
class MiddlewareMeta:
    """中间件层元数据, 标识生产者/环境/主时间源等."""

    primary_time_source: str
    producer: str
    environment: str
    stamp: Optional[Stamp] = None  # 仅在 primary_time_source=middleware.stamp 时存在


# --- 第一环最简原始数据帧 -----------------------------------------------


@dataclass(frozen=True)
class FirstRingRawFrame:
    """第一环最简输入帧.

    顶层字段严格遵循理论文档 § 5.2:
        header / source_sequence / config_revision /
        uav_state / target_state / image / camera_info / middleware
    """

    schema_version: str
    header: Header
    source_sequence: int
    config_revision: int
    uav_state: UavStateRaw
    target_state: TargetStateRaw
    image: ImageMetaRaw
    camera_info: CameraInfoRaw
    middleware: MiddlewareMeta


# --- 序列化 / 反序列化 ---------------------------------------------------

# dataclass -> dict 时, 嵌套 dataclass 需要递归展开. 这里用 asdict 已经能
# 递归展开 dataclass, 但需要把 Optional[Stamp]=None 这类显式 None 保留.


def _drop_none(d: Dict[str, Any]) -> Dict[str, Any]:
    """递归删除值为 None 的键, 用于反序列化时把 None 还原为字段默认."""

    out: Dict[str, Any] = {}
    for k, v in d.items():
        if v is None:
            continue
        if isinstance(v, dict):
            out[k] = _drop_none(v)
        else:
            out[k] = v
    return out


def _coerce_optional(cls, value: Any):
    """Optional[T] 反序列化辅助: value 为 None 时直接返回 None."""

    if value is None:
        return None
    return cls(**value)


def to_dict(frame: FirstRingRawFrame) -> Dict[str, Any]:
    """将 FirstRingRawFrame 转成纯 dict (可被 json.dumps 序列化)."""

    return _drop_none(asdict(frame))


def to_json(frame: FirstRingRawFrame, *, indent: Optional[int] = None) -> str:
    """生成标准 JSON 字符串."""

    return json.dumps(to_dict(frame), ensure_ascii=False, indent=indent)


def from_dict(payload: Dict[str, Any]) -> FirstRingRawFrame:
    """从 dict 还原 FirstRingRawFrame.

    不做算法层校验, 仅做字段构造. 字段是否合法由 validators 负责.
    """

    if not isinstance(payload, dict):
        raise TypeError(f"payload must be dict, got {type(payload).__name__}")

    header = Header(
        stamp=Stamp(**payload["header"]["stamp"]),
        frame_id=payload["header"]["frame_id"],
    )

    uav_raw = payload["uav_state"]
    uav_state = UavStateRaw(
        stamp=Stamp(**uav_raw["stamp"]),
        frame_id=uav_raw["frame_id"],
        raw_frame_convention=uav_raw["raw_frame_convention"],
        position=Vector3(**uav_raw["position"]),
        orientation=Quaternion(**uav_raw["orientation"]),
        linear_velocity=Vector3(**uav_raw["linear_velocity"]),
        angular_velocity=Vector3(**uav_raw["angular_velocity"]),
        source=uav_raw["source"],
    )

    tgt_raw = payload["target_state"]
    target_state = TargetStateRaw(
        has_target_state=bool(tgt_raw["has_target_state"]),
        target_id=str(tgt_raw.get("target_id", "")),
        stamp=_coerce_optional(Stamp, tgt_raw.get("stamp")),
        frame_id=tgt_raw.get("frame_id"),
        position=_coerce_optional(Vector3, tgt_raw.get("position")),
        orientation=_coerce_optional(Quaternion, tgt_raw.get("orientation")),
        linear_velocity=_coerce_optional(Vector3, tgt_raw.get("linear_velocity")),
        angular_velocity=_coerce_optional(Vector3, tgt_raw.get("angular_velocity")),
        source=str(tgt_raw.get("source", schema.SRC_UNAVAILABLE)),
    )

    img_raw = payload["image"]
    image = ImageMetaRaw(
        header=Header(
            stamp=Stamp(**img_raw["header"]["stamp"]),
            frame_id=img_raw["header"]["frame_id"],
        ),
        image_topic=str(img_raw["image_topic"]),
        image_sequence=int(img_raw["image_sequence"]),
        height=int(img_raw["height"]),
        width=int(img_raw["width"]),
        encoding=str(img_raw["encoding"]),
        is_bigendian=int(img_raw["is_bigendian"]),
        step=int(img_raw["step"]),
        data_transport=str(img_raw["data_transport"]),
        data_base64=img_raw.get("data_base64"),
    )

    cam_raw = payload["camera_info"]
    camera_info = CameraInfoRaw(
        header=Header(
            stamp=Stamp(**cam_raw["header"]["stamp"]),
            frame_id=cam_raw["header"]["frame_id"],
        ),
        camera_info_topic=str(cam_raw["camera_info_topic"]),
        height=int(cam_raw["height"]),
        width=int(cam_raw["width"]),
        distortion_model=str(cam_raw["distortion_model"]),
        d=list(cam_raw.get("d", [])),
        k=list(cam_raw["k"]),
        r=list(cam_raw["r"]),
        p=list(cam_raw["p"]),
    )

    mw_raw = payload["middleware"]
    middleware = MiddlewareMeta(
        primary_time_source=str(mw_raw["primary_time_source"]),
        producer=str(mw_raw["producer"]),
        environment=str(mw_raw["environment"]),
        stamp=_coerce_optional(Stamp, mw_raw.get("stamp")),
    )

    return FirstRingRawFrame(
        schema_version=str(payload["schema_version"]),
        header=header,
        source_sequence=int(payload["source_sequence"]),
        config_revision=int(payload["config_revision"]),
        uav_state=uav_state,
        target_state=target_state,
        image=image,
        camera_info=camera_info,
        middleware=middleware,
    )


def from_json(text: str) -> FirstRingRawFrame:
    """从 JSON 字符串还原 FirstRingRawFrame."""

    payload = json.loads(text)
    return from_dict(payload)


# --- 构造辅助 ------------------------------------------------------------


def stamp_from_ns(total_ns: int) -> Stamp:
    """把一个纳秒整数拆成 sec + nanosec, 供 mock 使用."""

    if total_ns < 0:
        raise ValueError("total_ns must be non-negative")
    return Stamp(sec=total_ns // 1_000_000_000, nanosec=total_ns % 1_000_000_000)


__all__ = [
    "Stamp",
    "Header",
    "Vector3",
    "Quaternion",
    "UavStateRaw",
    "TargetStateRaw",
    "ImageMetaRaw",
    "CameraInfoRaw",
    "MiddlewareMeta",
    "FirstRingRawFrame",
    "to_dict",
    "to_json",
    "from_dict",
    "from_json",
    "stamp_from_ns",
]


# 仅用于测试时方便地拿 dataclass 字段列表, 不导出业务逻辑.
def _field_names(cls) -> List[str]:
    return [f.name for f in fields(cls)]
