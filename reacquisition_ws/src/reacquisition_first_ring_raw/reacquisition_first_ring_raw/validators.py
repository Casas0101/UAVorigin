"""validators.py

只做格式级校验, 不判断算法意义上的有效性.

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md

允许检查:
- 顶层字段是否存在;
- stamp.sec / stamp.nanosec 是否为整数;
- frame_id 是否为空;
- source_sequence 是否递增;
- has_target_state=false 时是否没有伪造目标位置;
- JSON 是否为标准 JSON.

禁止检查:
- 目标是否在图像里;
- UAV 与目标坐标是否物理一致;
- 相机内参是否真实;
- 图像是否能检测目标;
- 任何几何或运动学结论.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from . import schema


# --- 校验结果结构 --------------------------------------------------------


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def __bool__(self) -> bool:  # 让 if result: 自然工作
        return self.ok

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


def _err(errors: List[str], msg: str) -> None:
    errors.append(msg)


def _warn(warnings: List[str], msg: str) -> None:
    warnings.append(msg)


# --- 单帧校验 ------------------------------------------------------------


def validate_frame_dict(payload: Dict[str, Any]) -> ValidationResult:
    """对单个 raw_frame dict 做格式校验."""

    errors: List[str] = []
    warnings: List[str] = []

    if not isinstance(payload, dict):
        _err(errors, f"payload must be dict, got {type(payload).__name__}")
        return ValidationResult(False, errors, warnings)

    # 1. 顶层必需字段
    for f in schema.REQUIRED_TOP_LEVEL_FIELDS:
        if f not in payload:
            _err(errors, f"missing top-level field: {f}")

    # 2. 禁止输出字段 (Windows 最简版不得伪造), 包括任意嵌套位置.
    _validate_no_forbidden_fields(payload, "", errors)

    # 3. schema_version
    if payload.get("schema_version") != schema.RAW_FRAME_SCHEMA_VERSION:
        _err(
            errors,
            f"schema_version must be {schema.RAW_FRAME_SCHEMA_VERSION}, "
            f"got {payload.get('schema_version')!r}",
        )

    # 4. header
    _validate_header(payload.get("header"), "header", errors)

    # 5. source_sequence
    seq = payload.get("source_sequence")
    if not isinstance(seq, int) or isinstance(seq, bool):
        _err(errors, f"source_sequence must be int, got {type(seq).__name__}")

    # 6. config_revision
    rev = payload.get("config_revision")
    if not isinstance(rev, int) or isinstance(rev, bool):
        _err(errors, f"config_revision must be int, got {type(rev).__name__}")

    # 7. uav_state
    _validate_uav_state(payload.get("uav_state"), errors)

    # 8. target_state
    _validate_target_state(payload.get("target_state"), errors, warnings)

    # 9. image
    _validate_image(payload.get("image"), errors)

    # 10. camera_info
    _validate_camera_info(payload.get("camera_info"), errors)

    # 11. middleware
    _validate_middleware(
        payload.get("middleware"),
        errors,
        payload.get("image"),
        payload.get("header"),
    )

    return ValidationResult(ok=not errors, errors=errors, warnings=warnings)


def validate_frame(frame) -> ValidationResult:
    """接受 FirstRingRawFrame / dict / JSON 字符串."""

    if isinstance(frame, str):
        try:
            payload = json.loads(frame)
        except json.JSONDecodeError as exc:
            return ValidationResult(False, [f"invalid JSON: {exc}"])
    elif hasattr(frame, "__dataclass_fields__"):
        from . import raw_frame as _rf
        if isinstance(frame, _rf.FirstRingRawFrame):
            payload = _rf.to_dict(frame)
        else:
            return ValidationResult(False, ["unsupported frame type"])
    elif isinstance(frame, dict):
        payload = frame
    else:
        return ValidationResult(False, [f"unsupported frame type: {type(frame).__name__}"])

    return validate_frame_dict(payload)


# --- 子结构校验 ----------------------------------------------------------


def _validate_header(header: Any, where: str, errors: List[str]) -> None:
    if not isinstance(header, dict):
        _err(errors, f"{where} must be dict")
        return

    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        _err(errors, f"{where}.stamp must be dict")
    else:
        for k in ("sec", "nanosec"):
            v = stamp.get(k)
            if not isinstance(v, int) or isinstance(v, bool):
                _err(errors, f"{where}.stamp.{k} must be int, got {type(v).__name__}")

    frame_id = header.get("frame_id")
    if not isinstance(frame_id, str):
        _err(errors, f"{where}.frame_id must be str")
    elif frame_id == "":
        _err(errors, f"{where}.frame_id must not be empty")


def _validate_stamp(stamp: Any, where: str, errors: List[str]) -> None:
    """校验裸 Stamp (sec + nanosec, 无 frame_id)."""

    if not isinstance(stamp, dict):
        _err(errors, f"{where} must be dict")
        return
    for k in ("sec", "nanosec"):
        v = stamp.get(k)
        if not isinstance(v, int) or isinstance(v, bool):
            _err(errors, f"{where}.{k} must be int, got {type(v).__name__}")


def _stamp_tuple(stamp: Any) -> Optional[tuple[int, int]]:
    if not isinstance(stamp, dict):
        return None
    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec")
    if (
        isinstance(sec, int)
        and not isinstance(sec, bool)
        and isinstance(nanosec, int)
        and not isinstance(nanosec, bool)
    ):
        return sec, nanosec
    return None


def _header_stamp_tuple(header: Any) -> Optional[tuple[int, int]]:
    if not isinstance(header, dict):
        return None
    return _stamp_tuple(header.get("stamp"))


def _validate_no_forbidden_fields(value: Any, where: str, errors: List[str]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{where}.{key}" if where else str(key)
            if key in schema.FORBIDDEN_OUTPUT_FIELDS:
                _err(errors, f"forbidden field present: {child_path}")
            _validate_no_forbidden_fields(child, child_path, errors)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            child_path = f"{where}[{index}]" if where else f"[{index}]"
            _validate_no_forbidden_fields(child, child_path, errors)


def _validate_uav_state(uav: Any, errors: List[str]) -> None:
    if not isinstance(uav, dict):
        _err(errors, "uav_state must be dict")
        return
    _validate_stamp(uav.get("stamp"), "uav_state.stamp", errors)
    if not isinstance(uav.get("frame_id"), str) or uav.get("frame_id") == "":
        _err(errors, "uav_state.frame_id must be non-empty str")

    conv = uav.get("raw_frame_convention")
    if conv not in schema.ALLOWED_COORD_CONVENTIONS:
        _err(
            errors,
            f"uav_state.raw_frame_convention must be one of {schema.ALLOWED_COORD_CONVENTIONS}, "
            f"got {conv!r}",
        )

    _validate_vector3(uav.get("position"), "uav_state.position", errors)
    _validate_quaternion(uav.get("orientation"), "uav_state.orientation", errors)
    _validate_vector3(uav.get("linear_velocity"), "uav_state.linear_velocity", errors)
    _validate_vector3(uav.get("angular_velocity"), "uav_state.angular_velocity", errors)

    src = uav.get("source")
    if src not in schema.ALLOWED_UAV_SOURCES:
        _err(
            errors,
            f"uav_state.source must be one of {schema.ALLOWED_UAV_SOURCES}, got {src!r}",
        )


def _validate_target_state(
    tgt: Any, errors: List[str], warnings: List[str]
) -> None:
    if not isinstance(tgt, dict):
        _err(errors, "target_state must be dict")
        return

    has = tgt.get("has_target_state")
    if not isinstance(has, bool):
        _err(errors, "target_state.has_target_state must be bool")
        return

    src = tgt.get("source")
    if src not in schema.ALLOWED_TARGET_SOURCES:
        _err(
            errors,
            f"target_state.source must be one of {schema.ALLOWED_TARGET_SOURCES}, "
            f"got {src!r}",
        )

    if not has:
        # 无目标状态场景: 不得伪造位置/速度
        for f in ("position", "orientation", "linear_velocity", "angular_velocity"):
            if f in tgt and tgt[f] is not None:
                _err(
                    errors,
                    f"target_state.{f} must be null when has_target_state=false, "
                    f"got {tgt[f]!r}",
                )
        if "stamp" in tgt and tgt["stamp"] is not None:
            _err(errors, "target_state.stamp must be null when has_target_state=false")
        if "frame_id" in tgt and tgt["frame_id"] is not None:
            _err(errors, "target_state.frame_id must be null when has_target_state=false")
        return

    # has_target_state=True: 必须有合法时间戳/frame_id/位置
    _validate_stamp(tgt.get("stamp"), "target_state.stamp", errors)
    if not isinstance(tgt.get("frame_id"), str) or tgt.get("frame_id") == "":
        _err(errors, "target_state.frame_id must be non-empty str")
    if not isinstance(tgt.get("target_id"), str) or tgt.get("target_id") == "":
        _err(errors, "target_state.target_id must be non-empty str when has_target_state=true")
    _validate_vector3(tgt.get("position"), "target_state.position", errors)
    _validate_quaternion(tgt.get("orientation"), "target_state.orientation", errors, allow_zero_w=True)
    _validate_vector3(tgt.get("linear_velocity"), "target_state.linear_velocity", errors)
    _validate_vector3(tgt.get("angular_velocity"), "target_state.angular_velocity", errors)


def _validate_image(img: Any, errors: List[str]) -> None:
    if not isinstance(img, dict):
        _err(errors, "image must be dict")
        return

    _validate_header(img.get("header"), "image.header", errors)
    if not isinstance(img.get("image_topic"), str) or img.get("image_topic") == "":
        _err(errors, "image.image_topic must be non-empty str")

    seq = img.get("image_sequence")
    if not isinstance(seq, int) or isinstance(seq, bool):
        _err(errors, "image.image_sequence must be int")

    h = img.get("height")
    w = img.get("width")
    if not isinstance(h, int) or h <= 0:
        _err(errors, f"image.height must be positive int, got {h!r}")
    if not isinstance(w, int) or w <= 0:
        _err(errors, f"image.width must be positive int, got {w!r}")

    enc = img.get("encoding")
    if enc not in schema.ALLOWED_IMAGE_ENCODINGS:
        _err(
            errors,
            f"image.encoding must be one of {schema.ALLOWED_IMAGE_ENCODINGS}, got {enc!r}",
        )

    be = img.get("is_bigendian")
    if not isinstance(be, int) or be not in (0, 1):
        _err(errors, "image.is_bigendian must be 0 or 1")

    step = img.get("step")
    if not isinstance(step, int) or step <= 0:
        _err(errors, "image.step must be positive int")

    transport = img.get("data_transport")
    if transport not in schema.ALLOWED_DATA_TRANSPORTS:
        _err(
            errors,
            f"image.data_transport must be one of {schema.ALLOWED_DATA_TRANSPORTS}, "
            f"got {transport!r}",
        )

    if transport == schema.DATA_TRANSPORT_BASE64_INLINE_DEBUG:
        b64 = img.get("data_base64")
        if not isinstance(b64, str) or not b64:
            _err(errors, "image.data_base64 must be non-empty str when transport=base64_inline_debug")


def _validate_camera_info(cam: Any, errors: List[str]) -> None:
    if not isinstance(cam, dict):
        _err(errors, "camera_info must be dict")
        return

    _validate_header(cam.get("header"), "camera_info.header", errors)

    h = cam.get("height")
    w = cam.get("width")
    if not isinstance(h, int) or h <= 0:
        _err(errors, f"camera_info.height must be positive int, got {h!r}")
    if not isinstance(w, int) or w <= 0:
        _err(errors, f"camera_info.width must be positive int, got {w!r}")

    if not isinstance(cam.get("camera_info_topic"), str) or cam.get("camera_info_topic") == "":
        _err(errors, "camera_info.camera_info_topic must be non-empty str")

    dm = cam.get("distortion_model")
    if dm not in schema.ALLOWED_DISTORTION_MODELS:
        _err(
            errors,
            f"camera_info.distortion_model must be one of {schema.ALLOWED_DISTORTION_MODELS}, "
            f"got {dm!r}",
        )

    _validate_9_list(cam.get("k"), "camera_info.k", errors)
    _validate_9_list(cam.get("r"), "camera_info.r", errors)
    _validate_12_list(cam.get("p"), "camera_info.p", errors)

    d = cam.get("d", [])
    if not isinstance(d, list) or not all(isinstance(x, (int, float)) for x in d):
        _err(errors, "camera_info.d must be list of numbers")


def _validate_middleware(
    mw: Any, errors: List[str], image: Any, header: Any
) -> None:
    if not isinstance(mw, dict):
        _err(errors, "middleware must be dict")
        return

    pts = mw.get("primary_time_source")
    if pts not in schema.ALLOWED_PRIMARY_TIME_SOURCES:
        _err(
            errors,
            f"middleware.primary_time_source must be one of "
            f"{schema.ALLOWED_PRIMARY_TIME_SOURCES}, got {pts!r}",
        )

    top_stamp = _header_stamp_tuple(header)
    image_stamp = _header_stamp_tuple(image.get("header")) if isinstance(image, dict) else None
    middleware_stamp = _stamp_tuple(mw.get("stamp"))

    if pts == schema.PRIMARY_TIME_SOURCE_MIDDLEWARE:
        _validate_stamp(mw.get("stamp"), "middleware.stamp", errors)
        if top_stamp is not None and middleware_stamp is not None and top_stamp != middleware_stamp:
            _err(errors, "header.stamp must equal middleware.stamp when primary_time_source=middleware.stamp")
    elif pts == schema.PRIMARY_TIME_SOURCE_IMAGE:
        if top_stamp is not None and image_stamp is not None and top_stamp != image_stamp:
            _err(errors, "header.stamp must equal image.header.stamp when primary_time_source=image.header.stamp")

    if not isinstance(mw.get("producer"), str) or mw.get("producer") == "":
        _err(errors, "middleware.producer must be non-empty str")
    if not isinstance(mw.get("environment"), str) or mw.get("environment") == "":
        _err(errors, "middleware.environment must be non-empty str")


# --- 基础原子校验 --------------------------------------------------------


def _validate_vector3(v: Any, where: str, errors: List[str]) -> None:
    if not isinstance(v, dict):
        _err(errors, f"{where} must be dict")
        return
    for k in ("x", "y", "z"):
        val = v.get(k)
        if not isinstance(val, (int, float)) or isinstance(val, bool):
            _err(errors, f"{where}.{k} must be number, got {type(val).__name__}")


def _validate_quaternion(
    q: Any, where: str, errors: List[str], *, allow_zero_w: bool = False
) -> None:
    if not isinstance(q, dict):
        _err(errors, f"{where} must be dict")
        return
    for k in ("x", "y", "z", "w"):
        val = q.get(k)
        if not isinstance(val, (int, float)) or isinstance(val, bool):
            _err(errors, f"{where}.{k} must be number, got {type(val).__name__}")
    if not allow_zero_w:
        # 默认为非退化四元数 (w != 0), 可选地用 allow_zero_w 放宽
        w = q.get("w")
        if w == 0:
            _err(errors, f"{where}.w must be non-zero")


def _validate_9_list(v: Any, where: str, errors: List[str]) -> None:
    if not isinstance(v, list) or len(v) != 9:
        _err(errors, f"{where} must be list of 9 numbers")
        return
    for i, x in enumerate(v):
        if not isinstance(x, (int, float)) or isinstance(x, bool):
            _err(errors, f"{where}[{i}] must be number")
            return


def _validate_12_list(v: Any, where: str, errors: List[str]) -> None:
    if not isinstance(v, list) or len(v) != 12:
        _err(errors, f"{where} must be list of 12 numbers")
        return
    for i, x in enumerate(v):
        if not isinstance(x, (int, float)) or isinstance(x, bool):
            _err(errors, f"{where}[{i}] must be number")
            return


# --- 序列级校验 ----------------------------------------------------------


def validate_sequence(frames: Sequence[Any]) -> ValidationResult:
    """校验一串帧的格式, 并检查 source_sequence 单调递增."""

    errors: List[str] = []
    warnings: List[str] = []

    last_seq: Optional[int] = None
    for i, frame in enumerate(frames):
        sub = validate_frame(frame)
        for e in sub.errors:
            errors.append(f"frame[{i}]: {e}")
        for w in sub.warnings:
            warnings.append(f"frame[{i}]: {w}")

        # source_sequence 单调 -- 与单帧校验独立, 即便单帧有其它问题也应报告递增问题
        if isinstance(frame, str):
            import json as _json
            try:
                payload = _json.loads(frame)
            except _json.JSONDecodeError:
                continue
        elif hasattr(frame, "__dataclass_fields__"):
            from . import raw_frame as _rf
            payload = _rf.to_dict(frame) if isinstance(frame, _rf.FirstRingRawFrame) else None
            if payload is None:
                continue
        else:
            payload = frame if isinstance(frame, dict) else None

        if isinstance(payload, dict):
            seq = payload.get("source_sequence")
            if isinstance(seq, int) and not isinstance(seq, bool):
                if last_seq is not None and seq <= last_seq:
                    errors.append(
                        f"frame[{i}].source_sequence={seq} is not strictly greater "
                        f"than previous {last_seq}"
                    )
                last_seq = seq

    return ValidationResult(ok=not errors, errors=errors, warnings=warnings)


__all__ = [
    "ValidationResult",
    "validate_frame",
    "validate_frame_dict",
    "validate_sequence",
]
