"""reacquisition_first_ring_raw

第一环最简原始数据中间件 (Windows 11, 无 ROS 2 / 无 Gazebo / 无 PX4 / 无 C++).

仅提供:
- dataclass 原始数据结构;
- JSON 序列化/反序列化;
- mock 原始数据生成;
- JSONL 写入;
- 格式级 validator;
- pytest 测试入口.

禁止 (按生成指南 § 0):
- ROS 2 / PX4 / Gazebo / OpenCV / numpy / C++ 相关依赖;
- 检测 / 反投影 / 滤波 / 预测 / 控制 等任何算法;
- 坐标转换或时间插值.
"""

from . import schema
from .raw_frame import (
    CameraInfoRaw,
    FirstRingRawFrame,
    Header,
    ImageMetaRaw,
    MiddlewareMeta,
    Quaternion,
    Stamp,
    TargetStateRaw,
    UavStateRaw,
    Vector3,
    from_dict,
    from_json,
    stamp_from_ns,
    to_dict,
    to_json,
)
from .mock_sources import (
    make_camera_info,
    make_image_meta,
    make_middleware,
    make_raw_frame,
    make_sequence,
    make_target_state,
    make_uav_state,
)
from .jsonl_writer import append_jsonl, read_jsonl, write_jsonl
from .validators import (
    ValidationResult,
    validate_frame,
    validate_frame_dict,
    validate_sequence,
)

__version__ = "0.1.0"

__all__ = [
    # schema
    "schema",
    # raw_frame
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
    # mock_sources
    "make_uav_state",
    "make_target_state",
    "make_image_meta",
    "make_camera_info",
    "make_middleware",
    "make_raw_frame",
    "make_sequence",
    # jsonl_writer
    "write_jsonl",
    "append_jsonl",
    "read_jsonl",
    # validators
    "ValidationResult",
    "validate_frame",
    "validate_frame_dict",
    "validate_sequence",
    # meta
    "__version__",
]
