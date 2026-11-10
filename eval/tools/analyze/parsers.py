#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评测数据解析：[EVAL] 埋点行 / recorder CSV / 时钟旁车文件。

只认统一契约格式（见 docs/评测架构定稿.md 与 hobot_mipi_eval_log.hpp）：
  [EVAL] ts_mono=<ns> event=<name> <key=value ...>
  topic,seq,stamp_ns,recv_mono_ns,width,height,encoding
  realtime_ns,mono_ns,offset_ns
解析鲁棒：坏行跳过并计数，不抛异常。
"""
import csv
import io
from typing import Dict, List, Optional


def _to_int(v: str) -> Optional[int]:
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def parse_eval_log(text: str) -> List[Dict]:
    """解析 [EVAL] 行 -> [{'ts_mono':int,'event':str,'kv':{str:int|str}}, ...]"""
    events = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("[EVAL] "):
            continue
        rest = line[len("[EVAL] "):]
        kv = {}
        parts = rest.split()
        ts_mono = None
        event = None
        for tok in parts:
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            if k == "ts_mono":
                ts_mono = _to_int(v)
            elif k == "event":
                event = v
            else:
                iv = _to_int(v)
                kv[k] = iv if iv is not None else v
        if ts_mono is None or event is None:
            continue
        events.append({"ts_mono": ts_mono, "event": event, "kv": kv})
    return events


class RecRow:
    __slots__ = ("topic", "seq", "stamp_ns", "recv_mono_ns", "width", "height", "encoding")

    def __init__(self, topic, seq, stamp_ns, recv_mono_ns, width, height, encoding):
        self.topic = topic
        self.seq = seq
        self.stamp_ns = stamp_ns
        self.recv_mono_ns = recv_mono_ns
        self.width = width
        self.height = height
        self.encoding = encoding

    def __repr__(self):  # pragma: no cover
        return (f"RecRow({self.topic},seq={self.seq},stamp={self.stamp_ns},"
                f"recv={self.recv_mono_ns},{self.width}x{self.height},{self.encoding})")


def parse_recorder_csv(text: str):
    """解析 recorder CSV -> (rows, skipped_count)"""
    rows = []
    skipped = 0
    reader = csv.reader(io.StringIO(text))
    header_seen = False
    for fields in reader:
        if not fields or not fields[0]:
            continue
        if not header_seen:
            header_seen = True  # topic,seq,stamp_ns,...
            continue
        if len(fields) != 7:
            skipped += 1
            continue
        try:
            stamp = int(fields[2])
            recv = int(fields[3])
            w = int(fields[4])
            h = int(fields[5])
        except ValueError:
            skipped += 1
            continue
        rows.append(RecRow(fields[0], fields[1], stamp, recv, w, h, fields[6]))
    return rows, skipped


def parse_clock_sidecar(text: str) -> Optional[int]:
    """解析时钟旁车文件 -> offset_ns（= realtime − monotonic）；失败返回 None"""
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("realtime_ns"):
            continue
        parts = line.split(",")
        if len(parts) != 3:
            continue
        off = _to_int(parts[2])
        if off is not None:
            return off
    return None
