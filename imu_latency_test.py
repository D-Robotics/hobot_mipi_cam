#!/usr/bin/env python3
"""
消息时间戳延迟受CPU负载影响测试脚本（含自动绘图）—— IMU 与 Image 分开统计/绘图

三阶段流程：预热 → 空载采集 → CPU压力 → 恢复采集；IMU 与 image 两类各自独立产出：
  - 各自的汇总表（**按空载/压力/恢复分段** + 全程对照：帧数/fps/p50/p95/p99/max）
  - 各自的 CSV（topic,stamp_ns,recv_mono_ns,recv_rt_ns,latency_ms）
  - 各自的延迟曲线图（红色区间标注 CPU 压力时段）
  - `<output stem>.phases.json`：阶段边界与域信息（供离线分析复算，避免按参数推算）

隔离设计（避免互相污染）：
  - IMU 与 image 各用**独立 Node + 独立 executor + 独立线程**，图像回调的
    反序列化不再阻塞 IMU 回调；
  - IMU 订阅用 **best_effort + 小 depth**（不随订阅端变慢而积压；实测 reliable
    depth=1000 会让 IMU 延迟被队列深度主导，出现秒级假延迟）。

时间戳域处理（mipi_cam 语义）：
  - IMU stamp 为 CLOCK_REALTIME 域（evdev/IIO 时间戳）→ 延迟 = recv_rt − stamp
  - image stamp 默认 CLOCK_MONOTONIC 域（frame_ts_type=sensor 内核帧时间戳）
    → 延迟 = recv_mono − stamp；--image_domain realtime 可切
  - 域配错时延迟出现 ~1e9 秒量级常数，一眼可辨
  - 启动时打印每类各 topic 的 `recv−stamp` 最小值（最接近纯偏移，排队最少），
    便于判断 image 的 boot 相关偏移

预热：建立订阅后先丢弃 --warmup 秒（默认 5s）的样本再开始计时——IIO 缓冲积压
  （实测前 5s 可积压到秒级）与订阅建立属启动瞬态。

注意：rclpy 反序列化整份图像消息，满配高帧率场景请改用 C++ recorder
（mipi_cam_eval/src/recorder_node.cpp）；本脚本面向快速诊断。

用法示例（板上）：
  python3 imu_latency_test.py \
      --imu_topic /imu_data \
      --image_topics /image_left_raw,/image_right_raw \
      --warmup 5 --pre_stress 60 --stress_duration 60 --post_stress 60 \
      --stress_cores 3 --output latency.csv --plot latency_plot.png
  # 产物：latency_imu.csv / latency_image.csv / latency_plot_imu.png /
  #       latency_plot_image.png / latency.phases.json
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import time
import multiprocessing
import csv
import json
import os
import argparse
import threading

try:
    import matplotlib
    matplotlib.use('Agg')  # 无GUI时保存图片用Agg后端，不会弹出窗口
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


# ============================== 基础设施 ==============================

def mono_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def rt_ns():
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


def cpu_stress_worker(stop_event):
    """单核空循环，直到stop_event被设置"""
    while not stop_event.is_set():
        _ = 1 + 1


def run_cpu_stress(duration, num_cores):
    """启动 num_cores 个进程死循环 duration 秒"""
    print(f"[Stress] 开始施加CPU压力：{num_cores} 核，持续 {duration} 秒")
    stop_event = multiprocessing.Event()
    processes = []
    for _ in range(num_cores):
        p = multiprocessing.Process(target=cpu_stress_worker, args=(stop_event,))
        p.daemon = True
        p.start()
        processes.append(p)
    time.sleep(duration)
    stop_event.set()
    for p in processes:
        p.join()
    print("[Stress] CPU压力结束")


def imu_worker(topic, out_csv, qos_depth, stop_evt):
    """独立进程订阅 IMU 并直接写 CSV。

    为什么必须用进程而不是线程：图像回调要反序列化 2MB 消息，持 GIL 期间
    IMU 回调无法进入；线程隔离下实测 IMU 延迟退化为队列深度/频率
    （depth=50 @200Hz → 固定 250ms 双峰）。进程隔离彻底消除 GIL 争抢。
    """
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import (QoSProfile as _QoS, ReliabilityPolicy as _Rel,
                           HistoryPolicy as _Hist)
    from sensor_msgs.msg import Imu as _Imu
    import time as _time
    import csv as _csv

    _rclpy.init()
    node = _Node('imu_latency_worker')
    qos = _QoS(history=_Hist.KEEP_LAST, depth=qos_depth,
               reliability=_Rel.BEST_EFFORT)
    f = open(out_csv, 'w', newline='', buffering=1)   # 行缓冲，进程被杀也不丢太多
    w = _csv.writer(f)
    w.writerow(['topic', 'stamp_ns', 'recv_mono_ns', 'recv_rt_ns', 'latency_ms'])

    def cb(msg):
        rm = _time.clock_gettime_ns(_time.CLOCK_MONOTONIC)
        rr = _time.clock_gettime_ns(_time.CLOCK_REALTIME)
        st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        w.writerow([topic, st, rm, rr, (rr - st) / 1e6])

    node.create_subscription(_Imu, topic, cb, qos)
    while not stop_evt.is_set():
        _rclpy.spin_once(node, timeout_sec=0.05)
    f.flush()
    f.close()
    node.destroy_node()
    _rclpy.shutdown()


def read_csv_rows(path):
    """读回子进程写的 CSV -> [(topic, stamp_ns, recv_mono_ns, recv_rt_ns, latency_ms)]"""
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for d in r:
            try:
                rows.append((d['topic'], int(d['stamp_ns']), int(d['recv_mono_ns']),
                             int(d['recv_rt_ns']), float(d['latency_ms'])))
            except (KeyError, ValueError):
                continue
    return rows


# ============================== 采集 ==============================

class TopicRecorder:
    """单 topic 延迟采集：记录 (stamp_ns, recv_mono_ns, recv_rt_ns)。

    domain='realtime' → 延迟 = recv_rt − stamp（IMU 语义）
    domain='mono'     → 延迟 = recv_mono − stamp（mipi_cam sensor 模式图像语义）
    """

    def __init__(self, node: Node, topic: str, msg_type, domain: str, qos):
        self.topic = topic
        self.domain = domain
        self.lock = threading.Lock()
        self.data = []
        node.create_subscription(msg_type, topic, self._cb, qos)
        node.get_logger().info(f"已订阅 {topic} (stamp域={domain})")

    def _cb(self, msg):
        # 先在回调入口取接收时刻，再做任何消息访问
        rm, rr = mono_ns(), rt_ns()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.lock:
            self.data.append((stamp, rm, rr))

    def clear(self):
        """丢弃已收样本（预热结束、正式计时前调用）"""
        with self.lock:
            self.data.clear()

    def rows(self):
        """-> [(topic, stamp_ns, recv_mono_ns, recv_rt_ns, latency_ms)]"""
        with self.lock:
            snap = list(self.data)
        out = []
        for stamp, recv_mono, recv_rt in snap:
            recv = recv_rt if self.domain == 'realtime' else recv_mono
            out.append((self.topic, stamp, recv_mono, recv_rt, (recv - stamp) / 1e6))
        return out

    def offset_hint_ms(self):
        """recv−stamp 的最小值（ms）：排队最少，最接近纯时钟域偏移"""
        with self.lock:
            snap = list(self.data)
        vals = [((rr if self.domain == 'realtime' else rm) - st) / 1e6
                for st, rm, rr in snap]
        return min(vals) if vals else None


class LatencyTester:
    """图像订阅（主进程）；IMU 由独立子进程负责（见 imu_worker）。"""

    def __init__(self, image_topics, image_domain: str = 'mono'):
        self.image_recorders = []
        self.image_node = None
        self.image_exec = None

        if image_topics:
            self.image_node = Node('image_latency_recorder')
            # sensor_qos：best_effort（与 mipi_cam 图像发布端一致）
            img_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            for t in image_topics:
                self.image_recorders.append(
                    TopicRecorder(self.image_node, t, Image, image_domain, img_qos))

    def start(self):
        """executor 起独立线程"""
        self._threads = []
        if self.image_node is not None:
            self.image_exec = SingleThreadedExecutor()
            self.image_exec.add_node(self.image_node)
            t = threading.Thread(target=self._spin, args=(self.image_exec,), daemon=True)
            t.start()
            self._threads.append(t)

    @staticmethod
    def _spin(executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass

    def stop(self):
        if self.image_exec is not None:
            self.image_exec.shutdown()
        for t in self._threads:
            t.join(timeout=2.0)

    def clear(self):
        for rec in self.image_recorders:
            rec.clear()

    def image_rows(self):
        return [r for rec in self.image_recorders for r in rec.rows()]

    def offset_hints(self):
        return {rec.topic: rec.offset_hint_ms() for rec in self.image_recorders}


# ============================== 统计/落盘/绘图 ==============================

STAGE_NAMES = (('all', '全程'), ('pre', '空载'),
               ('stress', '压力'), ('post', '恢复'))


def _pct(sorted_vals, q):
    n = len(sorted_vals)
    return sorted_vals[min(int(q * (n - 1)), n - 1)]


def _stats(vals):
    if not vals:
        return None
    s = sorted(vals)
    return {'n': len(s), 'p50': _pct(s, 0.50), 'p95': _pct(s, 0.95),
            'p99': _pct(s, 0.99), 'max': s[-1]}


def _stage_of(recv_mono, stress_start_ns, stress_end_ns):
    if stress_start_ns is None or stress_end_ns is None:
        return 'all'
    if recv_mono < stress_start_ns:
        return 'pre'
    if recv_mono < stress_end_ns:
        return 'stress'
    return 'post'


def print_summary(title, rows, stress_start_ns=None, stress_end_ns=None):
    """rows: [(topic, stamp, recv_mono, recv_rt, latency_ms)]；按阶段分段输出"""
    print(f"\n===== {title} =====")
    print(f"{'topic':<26}{'阶段':<6}{'帧数':>8}{'fps':>8}"
          f"{'p50(ms)':>10}{'p95(ms)':>10}{'p99(ms)':>10}{'max(ms)':>10}")
    for t in sorted({r[0] for r in rows}):
        trs = sorted((r for r in rows if r[0] == t), key=lambda r: r[2])
        for stage, label in STAGE_NAMES:
            rs = [r for r in trs
                  if stage == 'all' or _stage_of(r[2], stress_start_ns,
                                                 stress_end_ns) == stage]
            if not rs:
                continue
            lat = [r[4] for r in rs]
            dur = (rs[-1][2] - rs[0][2]) / 1e9 if len(rs) > 1 else 0.0
            fps = (len(rs) - 1) / dur if dur > 0 else 0.0
            st = _stats(lat)
            print(f"{t:<26}{label:<6}{st['n']:>8}{fps:>8.1f}"
                  f"{st['p50']:>10.1f}{st['p95']:>10.1f}"
                  f"{st['p99']:>10.1f}{st['max']:>10.1f}")
    print()


def save_csv(rows, filename):
    if not rows:
        print(f"[CSV] {filename}: 无数据，跳过")
        return
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['topic', 'stamp_ns', 'recv_mono_ns', 'recv_rt_ns', 'latency_ms'])
        writer.writerows(rows)
    print(f"[CSV] {filename}（{len(rows)} 行）")


def save_phases(filename, t0_mono_ns, stress_start_ns, stress_end_ns,
                warmup_s, image_domain, offset_hints):
    """阶段边界 sidecar：离线分析直接采用，不再按参数推算"""
    data = {
        't0_mono_ns': int(t0_mono_ns),
        'stress_start_mono_ns': int(stress_start_ns) if stress_start_ns else None,
        'stress_end_mono_ns': int(stress_end_ns) if stress_end_ns else None,
        'warmup_s': warmup_s,
        'image_domain': image_domain,
        'offset_hint_ms': {k: (round(v, 3) if v is not None else None)
                           for k, v in offset_hints.items()},
    }
    with open(filename, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"[Phases] {filename}（阶段边界/域信息，供离线分析）")


def plot_latency(rows, output_plot, stress_start_mono, stress_end_mono, title):
    """单类延迟图：x=相对首样本时间(s, mono域)，y=延迟(ms)；红色区间=压力时段"""
    if plt is None:
        print("[Plot] 缺少 matplotlib，跳过绘图")
        return
    if not rows:
        print(f"[Plot] {output_plot}: 无数据，跳过")
        return
    rs = sorted(rows, key=lambda r: r[2])
    t0 = rs[0][2]
    fig, ax = plt.subplots(figsize=(12, 5))
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
    for i, t in enumerate(sorted({r[0] for r in rows})):
        xs = [(r[2] - t0) / 1e9 for r in rs if r[0] == t]
        ys = [r[4] for r in rs if r[0] == t]
        ax.plot(xs, ys, linewidth=0.5, alpha=0.7,
                color=colors[i % len(colors)], label=t)
        avg = sum(ys) / len(ys)
        ax.axhline(y=avg, color=colors[i % len(colors)], linestyle='--',
                   linewidth=1, label=f'{t} mean={avg:.1f}ms')

    if stress_start_mono and stress_start_mono > 0 and stress_end_mono > stress_start_mono:
        ax.axvspan((stress_start_mono * 1e9 - t0) / 1e9,
                   (stress_end_mono * 1e9 - t0) / 1e9,
                   alpha=0.15, color='red', label='CPU stress')

    ax.set_xlabel('Time (s) since first message')
    ax.set_ylabel('Latency (ms)')
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_plot, dpi=150)
    plt.close(fig)
    print(f"[Plot] {output_plot}")


def derived_path(base, suffix, ext):
    """latency.csv + suffix '_imu' → latency_imu.csv；显式路径缺后缀扩展名时补 ext"""
    stem, _ = os.path.splitext(base)
    return f"{stem}{suffix}{ext}"


# ============================== 主流程 ==============================

def main():
    parser = argparse.ArgumentParser(description='IMU/Image延迟与CPU负载测试（分各自统计与绘图）')
    parser.add_argument('--imu_topic', type=str, default='/imu_data',
                        help='IMU话题名（留空则不测IMU）')
    parser.add_argument('--image_topics', type=str,
                        default='/image_left_raw,/image_right_raw',
                        help='image话题名，逗号分隔（留空则不测image）')
    parser.add_argument('--image_domain', type=str, default='mono',
                        choices=['mono', 'realtime'],
                        help='image stamp时钟域（mipi_cam frame_ts_type=sensor→mono）')
    parser.add_argument('--warmup', type=float, default=5.0,
                        help='预热秒数：先丢弃该窗口样本再开始计时（IIO 缓冲/订阅建立瞬态）')
    parser.add_argument('--output', type=str, default='latency.csv',
                        help='CSV输出基名（自动派生 <stem>_imu.csv / <stem>_image.csv）')
    parser.add_argument('--imu_output', type=str, default=None, help='IMU CSV路径（覆盖派生名）')
    parser.add_argument('--image_output', type=str, default=None, help='Image CSV路径（覆盖派生名）')
    parser.add_argument('--plot', type=str, default='./latency_plot.png',
                        help='图输出基名（自动派生 <stem>_imu.png / <stem>_image.png）')
    parser.add_argument('--imu_plot', type=str, default=None, help='IMU图路径（覆盖派生名）')
    parser.add_argument('--image_plot', type=str, default=None, help='Image图路径（覆盖派生名）')
    parser.add_argument('--phases_output', type=str, default=None,
                        help='阶段边界 sidecar 路径（默认 <output stem>.phases.json）')
    parser.add_argument('--pre_stress', type=float, default=30.0, help='压力前采集时长（秒）')
    parser.add_argument('--stress_duration', type=float, default=30.0, help='CPU压力时长（秒）')
    parser.add_argument('--post_stress', type=float, default=30.0, help='压力后采集时长（秒）')
    parser.add_argument('--stress_cores', type=int, default=max((os.cpu_count() or 2) - 1, 1),
                        help='压力核心数（默认留1核给系统与采集进程）')
    args = parser.parse_args()

    image_topics = [t.strip() for t in args.image_topics.split(',') if t.strip()]
    stress_start_time = 0.0
    stress_end_time = 0.0
    t0_mono_ns = 0

    rclpy.init()
    node = LatencyTester(image_topics, args.image_domain)
    node.start()

    # IMU 独立进程（spawn）：与图像的 Python 反序列化彻底隔离
    imu_ctx = imu_stop = imu_proc = None
    imu_raw_csv = None
    if args.imu_topic:
        imu_ctx = multiprocessing.get_context('spawn')
        imu_stop = imu_ctx.Event()
        imu_raw_csv = os.path.splitext(args.output)[0] + '_imu_raw.csv'
        imu_proc = imu_ctx.Process(
            target=imu_worker,
            args=(args.imu_topic, imu_raw_csv, 50, imu_stop),
            daemon=True)
        imu_proc.start()
        print(f"[IMU] 独立订阅进程已启动（pid={imu_proc.pid}，raw={imu_raw_csv}）")

    try:
        if args.warmup > 0:
            print(f"[Phase 0] 预热 {args.warmup} 秒（丢弃启动瞬态）...")
            time.sleep(args.warmup)
        node.clear()

        t0_mono_ns = mono_ns()
        print(f"[Phase 1] 无压力采集 {args.pre_stress} 秒...")
        time.sleep(args.pre_stress)

        # 压力区间用 CLOCK_MONOTONIC 记（与 recv_mono_ns 同域，绘图直接对齐）
        stress_start_time = mono_ns() / 1e9

        print(f"[Phase 2] CPU压力 {args.stress_duration} 秒（{args.stress_cores} 核）...")
        if args.stress_duration > 0:
            stress_proc = multiprocessing.Process(
                target=run_cpu_stress,
                args=(args.stress_duration, args.stress_cores))
            stress_proc.start()
            stress_proc.join()

        stress_end_time = mono_ns() / 1e9

        print(f"[Phase 3] 压力后采集 {args.post_stress} 秒...")
        time.sleep(args.post_stress)
    except KeyboardInterrupt:
        print("测试被用户中断")
    finally:
        # 收 IMU 子进程并读回样本；按 t0（预热结束时刻）过滤预热段
        imu_all = []
        if imu_proc is not None:
            imu_stop.set()
            imu_proc.join(timeout=15)
            if imu_proc.is_alive():
                imu_proc.terminate()
                imu_proc.join(timeout=5)
            imu_all = read_csv_rows(imu_raw_csv)
            try:
                os.unlink(imu_raw_csv)   # 原始文件含预热段，正式 CSV 已过滤
            except OSError:
                pass
        imu_rows = [r for r in imu_all if r[2] >= t0_mono_ns] if t0_mono_ns else imu_all

        image_rows = node.image_rows()
        hints = node.offset_hints()
        if imu_all:
            hints[args.imu_topic] = min(r[4] for r in imu_all)
        stress_start_ns = int(stress_start_time * 1e9) if stress_start_time else None
        stress_end_ns = int(stress_end_time * 1e9) if stress_end_time else None

        # 域/偏移提示：最小值最接近纯时钟域偏移（排队最少）
        print("\n[域检查] recv−stamp 最小值（ms，接近纯偏移）：")
        for t, v in hints.items():
            print(f"  {t:<28}{'-' if v is None else f'{v:.3f}'}")

        # ---- IMU：独立统计/CSV/图 ----
        if imu_rows:
            print_summary('IMU 统计', imu_rows, stress_start_ns, stress_end_ns)
            save_csv(imu_rows, args.imu_output or derived_path(args.output, '_imu', '.csv'))
            plot_latency(imu_rows,
                         args.imu_plot or derived_path(args.plot, '_imu', '.png'),
                         stress_start_time, stress_end_time,
                         'IMU Latency During CPU Stress Test')

        # ---- Image：独立统计/CSV/图 ----
        if image_rows:
            print_summary('Image 统计', image_rows, stress_start_ns, stress_end_ns)
            save_csv(image_rows, args.image_output or derived_path(args.output, '_image', '.csv'))
            plot_latency(image_rows,
                         args.image_plot or derived_path(args.plot, '_image', '.png'),
                         stress_start_time, stress_end_time,
                         'Image Latency During CPU Stress Test')

        phases_path = args.phases_output or (
            os.path.splitext(args.output)[0] + '.phases.json')
        save_phases(phases_path, t0_mono_ns, stress_start_ns, stress_end_ns,
                    args.warmup, args.image_domain, hints)

        node.stop()
        rclpy.shutdown()
        print("测试结束。")


if __name__ == '__main__':
    main()
