#!/usr/bin/env python3
"""
消息时间戳延迟受CPU负载影响测试脚本（含自动绘图）—— IMU 与 Image 分开统计/绘图

三阶段流程：空载采集 → CPU压力 → 恢复采集；IMU 与 image 两类各自独立产出：
  - 各自的汇总表（帧数/fps/p50/p95/p99/max）
  - 各自的 CSV（topic,stamp_ns,recv_mono_ns,recv_rt_ns,latency_ms）
  - 各自的延迟曲线图（红色区间标注 CPU 压力时段）

时间戳域处理（mipi_cam 语义）：
  - IMU stamp 为 CLOCK_REALTIME 域（evdev/IIO 时间戳）→ 延迟 = recv_rt − stamp
  - image stamp 默认 CLOCK_MONOTONIC 域（frame_ts_type=sensor 内核帧时间戳）
    → 延迟 = recv_mono − stamp；--image_domain realtime 可切
  - 域配错时延迟出现 ~1e9 秒量级常数，一眼可辨

注意：rclpy 反序列化整份图像消息，满配高帧率场景请改用 C++ recorder
（mipi_cam_eval/src/recorder_node.cpp）；本脚本面向快速诊断。

用法示例（板上）：
  python3 imu_latency_test.py \
      --imu_topic /imu_data \
      --image_topics /image_left_raw,/image_right_raw \
      --pre_stress 30 --stress_duration 30 --post_stress 30 \
      --stress_cores 4 --output latency.csv --plot latency_plot.png
  # 产物：latency_imu.csv / latency_image.csv / latency_plot_imu.png / latency_plot_image.png
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import time
import multiprocessing
import csv
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


# ============================== 采集 ==============================

class TopicRecorder:
    """单 topic 延迟采集：记录 (stamp_ns, recv_mono_ns, recv_rt_ns)。

    domain='realtime' → 延迟 = recv_rt − stamp（IMU 语义）
    domain='mono'     → 延迟 = recv_mono − stamp（mipi_cam sensor 模式图像语义）
    """

    def __init__(self, node: Node, topic: str, msg_type, domain: str,
                 depth: int = 1000, sensor_qos: bool = False):
        self.topic = topic
        self.domain = domain
        self.lock = threading.Lock()
        self.data = []
        qos = qos_profile_sensor_data if sensor_qos else depth
        node.create_subscription(msg_type, topic, self._cb, qos)
        node.get_logger().info(f"已订阅 {topic} (stamp域={domain})")

    def _cb(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        rm, rr = mono_ns(), rt_ns()
        with self.lock:
            self.data.append((stamp, rm, rr))

    def rows(self):
        """-> [(topic, stamp_ns, recv_mono_ns, recv_rt_ns, latency_ms)]"""
        with self.lock:
            snap = list(self.data)
        out = []
        for stamp, recv_mono, recv_rt in snap:
            recv = recv_rt if self.domain == 'realtime' else recv_mono
            out.append((self.topic, stamp, recv_mono, recv_rt, (recv - stamp) / 1e6))
        return out


class LatencyTester(Node):
    """IMU + image 多 topic 延迟采集（两类 recorder 分开持有）"""

    def __init__(self, imu_topic: str, image_topics, image_domain: str = 'mono'):
        super().__init__('latency_tester')
        self.imu_recorders = []
        self.image_recorders = []
        if imu_topic:
            self.imu_recorders.append(
                TopicRecorder(self, imu_topic, Imu, 'realtime'))
        for t in image_topics:
            self.image_recorders.append(
                TopicRecorder(self, t, Image, image_domain, sensor_qos=True))

    def imu_rows(self):
        return [r for rec in self.imu_recorders for r in rec.rows()]

    def image_rows(self):
        return [r for rec in self.image_recorders for r in rec.rows()]


# ============================== 统计/落盘/绘图 ==============================

def print_summary(title, rows):
    """rows: [(topic, stamp, recv_mono, recv_rt, latency_ms)]"""
    print(f"\n===== {title} =====")
    print(f"{'topic':<28}{'帧数':>7}{'fps':>8}"
          f"{'p50(ms)':>10}{'p95(ms)':>10}{'p99(ms)':>10}{'max(ms)':>10}")
    for t in sorted({r[0] for r in rows}):
        rs = sorted((r for r in rows if r[0] == t), key=lambda r: r[2])
        lat = sorted(r[4] for r in rs)
        n = len(lat)
        dur = (rs[-1][2] - rs[0][2]) / 1e9 if n > 1 else 0.0
        fps = (n - 1) / dur if dur > 0 else 0.0

        def pct(q):
            return lat[min(int(q * (n - 1)), n - 1)]

        print(f"{t:<28}{n:>7}{fps:>8.1f}"
              f"{pct(0.50):>10.1f}{pct(0.95):>10.1f}{pct(0.99):>10.1f}{lat[-1]:>10.1f}")
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
    t_rel = [(r[2] - t0) / 1e9 for r in rs]
    lat = [r[4] for r in rs]

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

    if stress_start_mono > 0 and stress_end_mono > stress_start_mono:
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

def spin_node(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


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
    parser.add_argument('--output', type=str, default='latency.csv',
                        help='CSV输出基名（自动派生 <stem>_imu.csv / <stem>_image.csv）')
    parser.add_argument('--imu_output', type=str, default=None, help='IMU CSV路径（覆盖派生名）')
    parser.add_argument('--image_output', type=str, default=None, help='Image CSV路径（覆盖派生名）')
    parser.add_argument('--plot', type=str, default='./latency_plot.png',
                        help='图输出基名（自动派生 <stem>_imu.png / <stem>_image.png）')
    parser.add_argument('--imu_plot', type=str, default=None, help='IMU图路径（覆盖派生名）')
    parser.add_argument('--image_plot', type=str, default=None, help='Image图路径（覆盖派生名）')
    parser.add_argument('--pre_stress', type=float, default=30.0, help='压力前采集时长（秒）')
    parser.add_argument('--stress_duration', type=float, default=30.0, help='CPU压力时长（秒）')
    parser.add_argument('--post_stress', type=float, default=30.0, help='压力后采集时长（秒）')
    parser.add_argument('--stress_cores', type=int, default=max((os.cpu_count() or 2) - 1, 1),
                        help='压力核心数（默认留1核给系统与采集进程）')
    args = parser.parse_args()

    image_topics = [t.strip() for t in args.image_topics.split(',') if t.strip()]
    stress_start_time = 0.0
    stress_end_time = 0.0

    rclpy.init()
    node = LatencyTester(args.imu_topic, image_topics, args.image_domain)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=spin_node, args=(executor,), daemon=True)
    spin_thread.start()

    try:
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
        imu_rows = node.imu_rows()
        image_rows = node.image_rows()

        # ---- IMU：独立统计/CSV/图 ----
        if imu_rows:
            print_summary('IMU 统计', imu_rows)
            save_csv(imu_rows, args.imu_output or derived_path(args.output, '_imu', '.csv'))
            plot_latency(imu_rows,
                         args.imu_plot or derived_path(args.plot, '_imu', '.png'),
                         stress_start_time, stress_end_time,
                         'IMU Latency During CPU Stress Test')

        # ---- Image：独立统计/CSV/图 ----
        if image_rows:
            print_summary('Image 统计', image_rows)
            save_csv(image_rows, args.image_output or derived_path(args.output, '_image', '.csv'))
            plot_latency(image_rows,
                         args.image_plot or derived_path(args.plot, '_image', '.png'),
                         stress_start_time, stress_end_time,
                         'Image Latency During CPU Stress Test')

        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("测试结束。")


if __name__ == '__main__':
    main()
