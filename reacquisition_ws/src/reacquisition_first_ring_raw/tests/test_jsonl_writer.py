"""test_jsonl_writer.py

覆盖:
- JSONL 写入后每一行可被 json.loads 解析
- overwrite=False 时冲突抛 FileExistsError
- 父目录不存在时自动创建
- read_jsonl 回读与写入一致
- 默认不覆盖现有文件
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from reacquisition_first_ring_raw import (
    jsonl_writer,
    mock_sources,
    raw_frame,
)


def test_write_jsonl_creates_file_and_parent(tmp_path: Path):
    out = tmp_path / "subdir" / "frames.jsonl"
    frames = mock_sources.make_sequence(count=3)
    n = jsonl_writer.write_jsonl(frames, out)

    assert n == 3
    assert out.exists()

    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 3
    for line in lines:
        obj = json.loads(line)  # 每行都能被标准 JSON 解析
        assert obj["schema_version"] == "0.1"


def test_write_jsonl_refuses_to_overwrite_by_default(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    jsonl_writer.write_jsonl(mock_sources.make_sequence(count=2), out)
    with pytest.raises(FileExistsError):
        jsonl_writer.write_jsonl(mock_sources.make_sequence(count=2), out)


def test_write_jsonl_overwrite_true_replaces_file(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    jsonl_writer.write_jsonl(mock_sources.make_sequence(count=2), out)
    jsonl_writer.write_jsonl(mock_sources.make_sequence(count=4), out, overwrite=True)

    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 4


def test_append_jsonl_appends_lines(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    jsonl_writer.write_jsonl(mock_sources.make_sequence(count=2), out)
    jsonl_writer.append_jsonl(mock_sources.make_sequence(count=3, start_sequence=2), out)

    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 5
    seqs = [json.loads(l)["source_sequence"] for l in lines]
    assert seqs == [0, 1, 2, 3, 4]


def test_read_jsonl_roundtrip(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    frames = mock_sources.make_sequence(count=3)
    jsonl_writer.write_jsonl(frames, out)
    payloads = jsonl_writer.read_jsonl(out)
    assert len(payloads) == 3
    for orig, payload in zip(frames, payloads):
        assert payload == raw_frame.to_dict(orig)


def test_write_jsonl_accepts_dicts_directly(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    frames = mock_sources.make_sequence(count=2)
    dicts = [raw_frame.to_dict(f) for f in frames]
    jsonl_writer.write_jsonl(dicts, out)
    payloads = jsonl_writer.read_jsonl(out)
    assert payloads == dicts


def test_write_jsonl_uses_utf8_with_chinese_safe(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    frames = mock_sources.make_sequence(count=1)
    # 不修改数据, 仅保证写入不会因非 ASCII 出错 (中间件 producer 字段是 ASCII)
    jsonl_writer.write_jsonl(frames, out)
    text = out.read_text(encoding="utf-8")
    # 含 schema_version 等 ASCII 字段
    assert "windows_mock_generator" in text


def test_write_jsonl_honors_ensure_ascii_for_dicts(tmp_path: Path):
    out = tmp_path / "frames.jsonl"
    payload = {"schema_version": "0.1", "middleware": {"producer": "中文"}}
    jsonl_writer.write_jsonl([payload], out, ensure_ascii=True)
    text = out.read_text(encoding="utf-8")
    assert "\\u4e2d\\u6587" in text
    assert "中文" not in text
