#!/usr/bin/env python3
"""Capture the next navigation run without publishing any ROS messages."""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


BASE = Path('/home/orangepi/ugv/odinauto')
CAPTURE_ROOT = BASE / 'captures'
TOPICS = [
    '/scan',
    '/odin1/cloud_raw',
    '/odin1/odometry',
    '/goal_pose',
    '/sru/preferred_route',
    '/sru/local_plan',
    '/sru/local_waypoint',
    '/sru/planner_active',
    '/sru/navigation_active',
    '/sru/obstacle_ahead',
    '/sru/obstacle_points',
    '/sru/odin_obstacle_points',
    '/sru/odin_obstacles_valid',
    '/sru/cmd_vel',
    '/sru/safe_cmd_vel',
    '/ctrl_cmd',
    '/chassis_info_fb',
]


class NextNavigationCapture(Node):
    def __init__(self):
        super().__init__('next_navigation_capture')
        self.capture_dir = None
        self.recorder = None
        self.started_at = None
        self.finished = False
        self.create_subscription(
            Bool, '/sru/navigation_active', self._active_callback, 10)
        self.create_timer(1.0, self._check_timeout)
        self.get_logger().info('Armed: waiting for the next navigation run.')

    def _active_callback(self, message):
        if message.data and self.recorder is None and not self.finished:
            self._start_capture()
        elif not message.data and self.recorder is not None:
            self._finish_capture('navigation_finished')

    def _start_capture(self):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.capture_dir = CAPTURE_ROOT / f'navigation_{stamp}'
        self.capture_dir.mkdir(parents=True, exist_ok=False)
        self.started_at = time.monotonic()
        self._write_status_snapshot('status_at_start.json')
        command = [
            'ros2', 'bag', 'record',
            '--output', str(self.capture_dir / 'bag'),
            '--max-bag-duration', '60',
            '--max-cache-size', '104857600',
            *TOPICS,
        ]
        self.recorder = subprocess.Popen(
            command,
            stdout=(self.capture_dir / 'rosbag.log').open('w'),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.get_logger().info(f'Capturing to {self.capture_dir}')

    def _check_timeout(self):
        if self.recorder is not None and time.monotonic() - self.started_at >= 180.0:
            self._finish_capture('maximum_duration_reached')

    def _write_status_snapshot(self, filename):
        try:
            with urllib.request.urlopen('http://127.0.0.1:8090/api/status', timeout=2) as response:
                status = json.load(response)
            (self.capture_dir / filename).write_text(
                json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')
        except Exception as error:
            (self.capture_dir / f'{filename}.error').write_text(
                str(error), encoding='utf-8')

    def _finish_capture(self, reason):
        if self.finished:
            return
        self.finished = True
        time.sleep(1.0)
        if self.recorder.poll() is None:
            os.killpg(self.recorder.pid, signal.SIGINT)
            try:
                self.recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.recorder.pid, signal.SIGTERM)
                self.recorder.wait(timeout=5)
        self._write_status_snapshot('status_at_end.json')
        for name in ('navigation.log', 'goal_ui.log', 'lidar.log'):
            source = BASE / 'run' / name
            if source.exists():
                shutil.copy2(source, self.capture_dir / name)
        metadata = {
            'reason': reason,
            'duration_seconds': round(time.monotonic() - self.started_at, 3),
            'topics': TOPICS,
            'rosbag_return_code': self.recorder.returncode,
        }
        (self.capture_dir / 'capture.json').write_text(
            json.dumps(metadata, indent=2), encoding='utf-8')
        (CAPTURE_ROOT / 'latest.txt').write_text(
            str(self.capture_dir) + '\n', encoding='utf-8')
        self.get_logger().info(f'Capture complete: {self.capture_dir}')


def main():
    CAPTURE_ROOT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = NextNavigationCapture()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        if node.recorder is not None and not node.finished:
            node._finish_capture('capture_process_stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
