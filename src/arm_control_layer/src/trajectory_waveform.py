#!/usr/bin/env python3
"""Record JTC state without commanding motion; export CSV and static waveforms."""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import time


FIELDS = (
    'time_s', 'stamp_ns', 'joint', 'reference_position_rad',
    'feedback_position_rad', 'error_position_rad', 'reference_velocity_rad_s',
    'feedback_velocity_rad_s', 'output_velocity_rad_s',
)


def value(values, index):
    return values[index] if index < len(values) else math.nan


def plot_csv(csv_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    joints = {}
    with csv_path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            joints.setdefault(row['joint'], []).append(row)
    if not joints:
        print(f'没有数据，未生成图片：{csv_path}')
        return

    fig, axes = plt.subplots(len(joints), 3, figsize=(18, 3 * len(joints)), squeeze=False)
    degrees = 180.0 / math.pi
    print('每个关节的位置误差（整个记录区间）：')
    for index, (joint, rows) in enumerate(joints.items()):
        times = [float(row['time_s']) for row in rows]

        def series(key):
            return [float(row[key]) * degrees for row in rows]

        axes[index, 0].plot(times, series('reference_position_rad'), label='Reference')
        axes[index, 0].plot(times, series('feedback_position_rad'), label='Feedback')
        axes[index, 0].set_ylabel(f'{joint}\nPosition (deg)')
        errors = series('error_position_rad')
        axes[index, 1].plot(times, errors, label='Reference - feedback')
        axes[index, 1].axhline(0, color='gray', linewidth=0.5)
        axes[index, 1].set_ylabel('Position error (deg)')
        for key, label in (
            ('reference_velocity_rad_s', 'Reference'),
            ('feedback_velocity_rad_s', 'Feedback (filtered)'),
            ('output_velocity_rad_s', 'JTC output'),
        ):
            axes[index, 2].plot(times, series(key), label=label)
        axes[index, 2].set_ylabel('Velocity (deg/s)')
        finite_errors = [error for error in errors if math.isfinite(error)]
        if finite_errors:
            rms = math.sqrt(sum(error * error for error in finite_errors) / len(finite_errors))
            print(f'  {joint}: max_abs={max(map(abs, finite_errors)):.4f} deg, '
                  f'RMS={rms:.4f} deg, last={finite_errors[-1]:.4f} deg')
        for axis in axes[index]:
            axis.set_xlabel('Time from first state (s)')
            axis.grid(True, alpha=0.3)
            axis.legend(loc='best', fontsize=8)
    fig.suptitle('JTC tracking: output is not measured motor velocity')
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    image_path = csv_path.with_suffix('.png')
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    print(f'波形：{image_path}')


def record(args):
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / f'tracking_{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
    rclpy.init()
    node = Node('trajectory_waveform_recorder')
    first_stamp = None
    last_stamp = None
    count = 0
    max_gap = 0.0
    started = time.monotonic()
    try:
        with csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()

            def callback(msg):
                nonlocal first_stamp, last_stamp, count, max_gap
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if first_stamp is None:
                    first_stamp = stamp
                    print('收到状态，开始记录。请执行要对比的轨迹。', flush=True)
                if last_stamp is not None:
                    max_gap = max(max_gap, (stamp - last_stamp) / 1e9)
                last_stamp = stamp
                count += 1
                for index, joint in enumerate(msg.joint_names):
                    writer.writerow(dict(zip(FIELDS, (
                        (stamp - first_stamp) / 1e9, stamp, joint,
                        value(msg.reference.positions, index),
                        value(msg.feedback.positions, index),
                        value(msg.error.positions, index),
                        value(msg.reference.velocities, index),
                        value(msg.feedback.velocities, index),
                        value(msg.output.velocities, index),
                    ))))
                if count % 100 == 0:
                    stream.flush()

            subscription = node.create_subscription(
                JointTrajectoryControllerState, args.topic, callback, qos_profile_sensor_data)
            print(f'订阅：{args.topic}\nCSV：{csv_path}\nCtrl+C 停止并生成图片。', flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                elapsed = time.monotonic() - started
                if count == 0 and elapsed >= 10:
                    print('10 秒未收到状态，请检查话题名称、ROS_DOMAIN_ID 和控制器状态。')
                    break
                if args.duration > 0 and elapsed >= args.duration:
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'记录状态消息 {count} 条；最大消息时间间隔 {max_gap * 1000:.3f} ms。')
    if count > 1 and last_stamp > first_stamp:
        print(f'平均采样频率 {(count - 1) * 1e9 / (last_stamp - first_stamp):.2f} Hz。')
    print('采样间隔包含状态发布与采集因素，不代表 CAN 反馈间隔。')
    plot_csv(csv_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/manipulator_controller/controller_state')
    parser.add_argument('--duration', type=float, default=0,
                        help='记录秒数（从启动脚本计时）；0 表示直到 Ctrl+C')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[3] / 'tracking_logs')
    parser.add_argument('--plot', type=Path, help='仅从已有 CSV 生成图片，不连接 ROS')
    args = parser.parse_args()
    if args.duration < 0:
        parser.error('--duration 必须大于或等于 0')
    if args.plot:
        plot_csv(args.plot)
    else:
        record(args)


if __name__ == '__main__':
    main()