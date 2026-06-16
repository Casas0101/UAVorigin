"""generate_mock_raw_frames.py

Windows 11 mock 原始数据生成脚本.

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md

用法:
    python tools/generate_mock_raw_frames.py --output-dir outputs/raw_frames --count 10

特性:
- 默认不覆盖已有文件 (overwrite=False);
- 支持 --no-target 强制全部为 has_target_state=false;
- 支持 --primary-time-source image|middleware 切换主时间戳来源;
- 写入完成后, 立即对生成的 JSONL 做格式级 validate, 失败时以非零退出.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 把 src/reacquisition_first_ring_raw 加进 sys.path, 让脚本不依赖 pip install.
_HERE = Path(__file__).resolve().parent
_PKG_PARENT = _HERE.parent / "src" / "reacquisition_first_ring_raw"
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

from reacquisition_first_ring_raw import (  # noqa: E402
    jsonl_writer,
    mock_sources,
    schema,
    validators,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成 Windows 11 mock 原始数据帧, 写入 JSONL.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="输出目录, 默认文件名为 raw_frames.jsonl",
    )
    parser.add_argument(
        "--output-name",
        default="raw_frames.jsonl",
        help="输出文件名, 默认 raw_frames.jsonl",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="生成帧数, 默认 10",
    )
    parser.add_argument(
        "--config-revision",
        type=int,
        default=0,
        help="config_revision, 默认 0",
    )
    parser.add_argument(
        "--start-sequence",
        type=int,
        default=0,
        help="source_sequence 起始值, 默认 0",
    )
    parser.add_argument(
        "--no-target",
        action="store_true",
        help="所有帧的 target_state.has_target_state=false",
    )
    parser.add_argument(
        "--primary-time-source",
        choices=("image", "middleware"),
        default="image",
        help="主时间戳来源: image.header.stamp 或 middleware.stamp, 默认 image",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="允许覆盖已有文件, 默认 False",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.count < 0:
        print(f"[error] --count must be non-negative, got {args.count}", file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / args.output_name

    primary_time_source = (
        schema.PRIMARY_TIME_SOURCE_IMAGE
        if args.primary_time_source == "image"
        else schema.PRIMARY_TIME_SOURCE_MIDDLEWARE
    )

    frames = mock_sources.make_sequence(
        count=args.count,
        start_sequence=args.start_sequence,
        config_revision=args.config_revision,
        has_target=not args.no_target,
        primary_time_source=primary_time_source,
    )

    try:
        written = jsonl_writer.write_jsonl(frames, out_path, overwrite=args.overwrite)
    except FileExistsError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        print("[hint] 传入 --overwrite 以覆盖", file=sys.stderr)
        return 2

    print(f"[ok] wrote {written} frames to {out_path}")

    # 立刻回读并做格式级 validate, 失败时非零退出.
    payloads = jsonl_writer.read_jsonl(out_path)
    result = validators.validate_sequence(payloads)
    if not result.ok:
        print(f"[error] validation failed for {out_path}:", file=sys.stderr)
        for e in result.errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print(f"[ok] validated {len(payloads)} frames against schema {schema.RAW_FRAME_SCHEMA_VERSION}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
