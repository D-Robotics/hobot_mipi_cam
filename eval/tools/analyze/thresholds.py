#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评测阈值：默认值 + 用例 YAML 覆盖（配置化，评测程序可移植的基础）。

来源优先级：
1. 用例 YAML `thresholds:`（经 run_case.sh 写入 meta.txt 的 thresholds= 行）
2. 本文件 DEFAULT_THRESHOLDS

单位约定：*_ms 用毫秒、*_s 用秒、比率为小数；ns 只在 metrics 内部出现。
"""

DEFAULT_THRESHOLDS = {
    # A-01 实际帧率 ≥ 标称×fps_ratio_min
    "fps_ratio_min": 0.99,
    # A-06 丢帧率上限（0.5%）
    "drop_rate_max": 0.005,
    # A-07 分辨率一致率下限（100%）
    "resolution_match_min": 1.0,
    # A-02/稳定性：最大间隔 > 目标周期×该比率 → 告警
    "stability_interval_ratio": 3.0,
    # B-03 配对率下限（99.9%）
    "pair_rate_min": 0.999,
    # B-02 配对差 p99 上限（ms；口径=配对点 stamp 差）
    "pair_diff_p99_ms": 1.0,
    # 曝光↔stamp 最近邻匹配容差（ms）
    "exp_match_tol_ms": 50.0,
    # 曝光闸门硬上限（s）：超过视为跨域/陈旧 trig
    "exp_us_cap_s": 5.0,
    # 单侧缺帧持续多久才算 sync_pair 失败（驱动侧固定 100ms，仅分析侧对齐用）
    "sync_missing_sustain_ms": 100.0,
    # C-01 IMU 标称采样率（icm42688 odr 默认 200Hz）
    "imu_nominal_hz": 200.0,
    # C-01 实测频率 ≥ 标称×该比率
    "imu_rate_ratio_min": 0.99,
    # C-02 间隔 p99 ≤ 周期×该比率
    "imu_interval_p99_ratio": 0.5,
    # C-03 时间戳倒退容忍（0 = 不允许）
    "imu_backward_max": 0,
    # C-05 静止 gyro 任一轴零偏上限（rad/s）
    "imu_gyro_bias_max": 0.03,
    # C-05 accel 模长合理区间（m/s²，±5% 重力）
    "imu_accel_norm_min": 9.3,
    "imu_accel_norm_max": 10.3,
}

# 允许出现在 YAML 里的键（未知键告警并忽略，防拼写错误静默失效）
KNOWN_KEYS = set(DEFAULT_THRESHOLDS)


def merge_thresholds(override: dict = None) -> dict:
    """默认值 + 覆盖；未知键忽略（评测链对未知配置保持宽容）。"""
    out = dict(DEFAULT_THRESHOLDS)
    if override:
        for k, v in override.items():
            if k in KNOWN_KEYS and v is not None:
                out[k] = v
    return out
