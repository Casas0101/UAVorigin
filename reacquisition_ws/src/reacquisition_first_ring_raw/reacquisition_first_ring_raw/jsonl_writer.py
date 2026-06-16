"""jsonl_writer.py

只负责把 FirstRingRawFrame 写成 JSONL.

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List, Union

from . import raw_frame


FrameLike = Union[raw_frame.FirstRingRawFrame, str, dict]


def _frame_to_json_text(frame: FrameLike, *, ensure_ascii: bool = False) -> str:
    """把帧统一转成 JSON 文本行."""

    if isinstance(frame, raw_frame.FirstRingRawFrame):
        return json.dumps(raw_frame.to_dict(frame), ensure_ascii=ensure_ascii)
    if isinstance(frame, str):
        # 假定调用者已经提供了合法 JSON, 不做二次序列化避免 key 顺序变化.
        return frame
    if isinstance(frame, dict):
        return json.dumps(frame, ensure_ascii=ensure_ascii)
    raise TypeError(f"unsupported frame type: {type(frame).__name__}")


def write_jsonl(
    frames: Iterable[FrameLike],
    output_path: Union[str, Path],
    *,
    overwrite: bool = False,
    ensure_ascii: bool = False,
) -> int:
    """把 frames 序列化为 JSONL, 写入指定路径.

    参数:
        frames: 可迭代的 FirstRingRawFrame / dict / JSON 字符串.
        output_path: 输出文件路径.
        overwrite: 是否允许覆盖已有文件, 默认 False (冲突时抛 FileExistsError).
        ensure_ascii: 是否把非 ASCII 转义, 默认 False, 与理论文档一致.

    返回:
        实际写入的帧数.
    """

    path = Path(output_path)
    if path.exists() and not overwrite:
        raise FileExistsError(
            f"output file already exists: {path} (pass overwrite=True to replace)"
        )

    parent = path.parent
    if parent and not parent.exists():
        parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with path.open("w", encoding="utf-8", newline="\n") as fp:
        for f in frames:
            text = _frame_to_json_text(f, ensure_ascii=ensure_ascii)
            fp.write(text)
            fp.write("\n")
            count += 1

    return count


def append_jsonl(
    frames: Iterable[FrameLike],
    output_path: Union[str, Path],
    *,
    ensure_ascii: bool = False,
) -> int:
    """追加写入 JSONL, 不做覆盖检查."""

    path = Path(output_path)
    parent = path.parent
    if parent and not parent.exists():
        parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with path.open("a", encoding="utf-8", newline="\n") as fp:
        for f in frames:
            text = _frame_to_json_text(f, ensure_ascii=ensure_ascii)
            fp.write(text)
            fp.write("\n")
            count += 1

    return count


def read_jsonl(output_path: Union[str, Path]) -> List[dict]:
    """把 JSONL 文件读回为 dict 列表, 仅用于校验和回放辅助."""

    path = Path(output_path)
    if not path.exists():
        raise FileNotFoundError(path)

    out: List[dict] = []
    with path.open("r", encoding="utf-8") as fp:
        for line_no, raw_line in enumerate(fp, start=1):
            line = raw_line.strip()
            if not line:
                continue
            out.append(json.loads(line))

    return out


__all__ = [
    "write_jsonl",
    "append_jsonl",
    "read_jsonl",
    "FrameLike",
]
