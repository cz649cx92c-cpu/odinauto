#!/usr/bin/env python3
"""Small ROS 2-backed HTTP server for sending Odin odom goals."""

import json
import math
import mimetypes
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from lidar_preview import preview_points, retained_preview_points
from status_state import derive_navigation_state


STATIC_DIR = Path(__file__).resolve().parent / 'static'
LIDAR_CALIBRATION_PATH = Path(os.environ.get(
    'LIDAR_CALIBRATION_PATH',
    '/home/orangepi/ugv/autorunlida/config/lidar_calibration.json'))


def load_lidar_calibration():
    calibration = {
        'lidar_yaw_correction_deg': -3.0,
        'lidar_x_offset_m': 0.0,
        'lidar_y_offset_m': 0.035,
    }
    try:
        loaded = json.loads(LIDAR_CALIBRATION_PATH.read_text(encoding='utf-8'))
        for key in calibration:
            calibration[key] = float(loaded[key])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f'Warning: using fallback lidar calibration: {error}', flush=True)
    return calibration


class GoalBridge(Node):
    def __init__(self):
        super().__init__('odin_goal_ui')
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.odom = None
        self.odom_received = None
        self.obstacle = None
        self.safe_velocity = (0.0, 0.0, 0.0)
        self.velocity_received = None
        self.target = None
        self.target_active = False
        self.navigation_outcome = 'idle'
        self.route = []
        self.route_index = 0
        self.final_arrival_distance = 0.08
        self.local_plan = []
        self.lidar_points = []
        self.lidar_received = None
        self.lidar_points_received = None
        self.lidar_points_current = False
        self.lidar_raw_count = 0
        self.lidar_valid_count = 0
        calibration = load_lidar_calibration()
        self.lidar_yaw_deg = 180.0 + calibration['lidar_yaw_correction_deg']
        self.lidar_x = calibration['lidar_x_offset_m']
        self.lidar_y = calibration['lidar_y_offset_m']
        self.planner_active = False
        self.planner_received = None
        self.last_event = '等待 Odin 里程计'

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.preferred_route_pub = self.create_publisher(
            RosPath, '/sru/preferred_route', 10)
        self.cancel_pub = self.create_publisher(Bool, '/sru/cancel_navigation', 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odin1/odometry', self._odom_callback, sensor_qos)
        self.create_subscription(Bool, '/sru/obstacle_ahead', self._obstacle_callback, 10)
        self.create_subscription(Twist, '/sru/safe_cmd_vel', self._velocity_callback, 10)
        self.create_subscription(RosPath, '/sru/local_plan', self._plan_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, sensor_qos)
        self.create_subscription(Bool, '/sru/planner_active',
                                 self._planner_callback, 10)

    def _odom_callback(self, message):
        with self.lock:
            first = self.odom is None
            self.odom = message
            self.odom_received = time.monotonic()
            if first:
                self.last_event = 'Odin 里程计已连接'
        with self.lock:
            if not self.target_active or not self.target:
                return
            position = message.pose.pose.position
            distance = math.hypot(self.target['x'] - float(position.x),
                                  self.target['y'] - float(position.y))
            if distance <= self.final_arrival_distance:
                self.target_active = False
                self.planner_active = False
                self.navigation_outcome = 'completed'
                self.last_event = '路线已完成' if self.route else '已进入目标范围'

    def _obstacle_callback(self, message):
        with self.lock:
            self.obstacle = bool(message.data)

    def _velocity_callback(self, message):
        with self.lock:
            self.safe_velocity = (
                float(message.linear.x), float(message.linear.y), float(message.angular.z))
            self.velocity_received = time.monotonic()

    def _plan_callback(self, message):
        with self.lock:
            self.local_plan = [
                {'x': round(float(pose.pose.position.x), 3),
                 'y': round(float(pose.pose.position.y), 3)}
                for pose in message.poses]

    def _scan_callback(self, message):
        valid_count = sum(
            1 for value in message.ranges
            if math.isfinite(float(value)) and
            max(float(message.range_min), 0.05) <= float(value) <=
            min(float(message.range_max), 6.0))
        points = preview_points(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max,
            yaw_deg=self.lidar_yaw_deg,
            offset_x=self.lidar_x,
            offset_y=self.lidar_y)
        with self.lock:
            self.lidar_received = time.monotonic()
            self.lidar_points_current = bool(points)
            if points:
                self.lidar_points = [{'x': x, 'y': y} for x, y in points]
                self.lidar_points_received = self.lidar_received
            self.lidar_raw_count = len(message.ranges)
            self.lidar_valid_count = valid_count

    def _planner_callback(self, message):
        with self.lock:
            self.planner_active = bool(message.data)
            self.planner_received = time.monotonic()
            if self.target_active:
                self.last_event = ('实时路径已更新' if self.planner_active
                                   else '正在等待可行路径')

    @staticmethod
    def _yaw_from_quaternion(orientation):
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny, cosy)

    def status(self):
        now = time.monotonic()
        with self.lock:
            odom_age = None if self.odom_received is None else now - self.odom_received
            connected = odom_age is not None and odom_age < 1.5
            position = None
            frame = 'odom'
            if self.odom is not None:
                pose = self.odom.pose.pose
                position = {
                    'x': round(float(pose.position.x), 3),
                    'y': round(float(pose.position.y), 3),
                    'yaw': round(math.degrees(self._yaw_from_quaternion(pose.orientation)), 1),
                }
                frame = self.odom.header.frame_id or 'odom'
            target = self.target.copy() if self.target else None
            route = [point.copy() for point in self.route]
            active = self.target_active
            if active and position and target:
                distance = math.hypot(target['x'] - position['x'], target['y'] - position['y'])
            else:
                distance = None
            velocity_age = None if self.velocity_received is None else now - self.velocity_received
            planner_age = (None if self.planner_received is None else
                           now - self.planner_received)
            planner_active = (self.planner_active and planner_age is not None and
                              planner_age < 1.5)
            lidar_age = (None if self.lidar_received is None else
                         now - self.lidar_received)
            lidar_points_age = (None if self.lidar_points_received is None else
                                now - self.lidar_points_received)
            # Retain a valid visualization frame briefly across zero-return
            # scans, but never present old points as live sensor data.
            displayed_lidar_points = retained_preview_points(
                self.lidar_points, lidar_points_age)
            navigation_state = derive_navigation_state(
                target_active=active,
                planner_active=planner_active,
                obstacle=self.obstacle,
                outcome=self.navigation_outcome,
                has_route=bool(route))
            return {
                'connected': connected,
                'frame': frame,
                'position': position,
                'target': target,
                'target_active': active,
                'route': route,
                'route_index': self.route_index,
                'local_plan': [point.copy() for point in self.local_plan],
                'lidar_points': [point.copy() for point in displayed_lidar_points],
                'lidar_fresh': lidar_age is not None and lidar_age < 1.0,
                'lidar_points_current': self.lidar_points_current,
                'lidar_points_age': (None if lidar_points_age is None else
                                     round(lidar_points_age, 3)),
                'lidar_counts': {
                    'raw': self.lidar_raw_count,
                    'valid': self.lidar_valid_count,
                    'visible': len(self.lidar_points),
                },
                'lidar_calibration': {
                    'yaw_deg': self.lidar_yaw_deg,
                    'x': self.lidar_x,
                    'y': self.lidar_y,
                },
                'planner_active': planner_active,
                'navigation_state': navigation_state,
                'distance': None if distance is None else round(distance, 3),
                'obstacle': self.obstacle,
                'velocity': {
                    'x': round(self.safe_velocity[0], 3),
                    'y': round(self.safe_velocity[1], 3),
                    'yaw': round(self.safe_velocity[2], 3),
                    'fresh': velocity_age is not None and velocity_age < 1.0,
                },
                'event': self.last_event,
            }

    def _build_goal(self, x, y):
        with self.lock:
            if self.odom is None or self.odom_received is None or \
                    time.monotonic() - self.odom_received >= 1.5:
                raise RuntimeError('Odin 里程计未连接，不能发送目标')
            frame = self.odom.header.frame_id or 'odom'
            z = float(self.odom.pose.pose.position.z)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        return message

    def send_goal(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y) or abs(x) > 100.0 or abs(y) > 100.0:
            raise ValueError('坐标必须是 -100 到 100 之间的有效数字')
        with self.command_lock:
            message = self._build_goal(x, y)
            with self.lock:
                self.route = []
                self.route_index = 0
                self.target = {'x': round(x, 3), 'y': round(y, 3)}
                self.target_active = True
                self.navigation_outcome = 'active'
                self.last_event = f'目标已发送：X {x:.2f}，Y {y:.2f}'
            self.goal_pub.publish(message)
            self._publish_preferred_route([])

    def _publish_preferred_route(self, points):
        message = RosPath()
        message.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            message.header.frame_id = (
                self.odom.header.frame_id if self.odom is not None else 'odom') or 'odom'
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.preferred_route_pub.publish(message)

    def send_route(self, points):
        if not isinstance(points, list) or not 1 <= len(points) <= 160:
            raise ValueError('路线必须包含 1 到 160 个路点')
        parsed = []
        for point in points:
            try:
                x, y = float(point['x']), float(point['y'])
            except (KeyError, TypeError, ValueError):
                raise ValueError('路线中存在无效坐标')
            if not math.isfinite(x) or not math.isfinite(y) or abs(x) > 100.0 or abs(y) > 100.0:
                raise ValueError('路线坐标必须在 -100 到 100 之间')
            if not parsed or math.hypot(x - parsed[-1]['x'], y - parsed[-1]['y']) >= 0.10:
                parsed.append({'x': round(x, 3), 'y': round(y, 3)})
        with self.command_lock:
            with self.lock:
                if self.odom is None or self.odom_received is None or \
                        time.monotonic() - self.odom_received >= 1.5:
                    raise RuntimeError('Odin 里程计未连接，不能发送路线')
                position = self.odom.pose.pose.position
                while len(parsed) > 1 and math.hypot(
                        parsed[0]['x'] - float(position.x),
                        parsed[0]['y'] - float(position.y)) <= self.final_arrival_distance:
                    parsed.pop(0)
            if not parsed:
                raise ValueError('路线终点距离当前位置太近')
            message = self._build_goal(parsed[-1]['x'], parsed[-1]['y'])
            with self.lock:
                self.route = parsed
                self.route_index = 0
                self.target = parsed[-1].copy()
                self.target_active = True
                self.navigation_outcome = 'active'
                self.last_event = '实时规划已开始'
            self.goal_pub.publish(message)
            self._publish_preferred_route(parsed)

    def cancel(self):
        with self.command_lock:
            with self.lock:
                self.target_active = False
                self.navigation_outcome = 'stopped'
                self.route = []
                self.route_index = 0
                self.last_event = '导航已停止'
            self.cancel_pub.publish(Bool(data=True))
            self._publish_preferred_route([])


class GoalRequestHandler(BaseHTTPRequestHandler):
    bridge = None

    def log_message(self, fmt, *args):
        return

    def _json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get('Content-Length', '0'))
        if length <= 0 or length > 32768:
            raise ValueError('请求内容无效')
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/api/status':
            self._json(200, self.bridge.status())
            return
        asset = 'index.html' if path == '/' else path.lstrip('/')
        candidate = (STATIC_DIR / asset).resolve()
        if STATIC_DIR not in candidate.parents and candidate != STATIC_DIR:
            self.send_error(404)
            return
        if not candidate.is_file():
            self.send_error(404)
            return
        body = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or 'application/octet-stream'
        self.send_response(200)
        self.send_header('Content-Type', content_type + ('; charset=utf-8' if content_type.startswith('text/') else ''))
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            if path == '/api/goal':
                payload = self._read_json()
                self.bridge.send_goal(float(payload['x']), float(payload['y']))
                self._json(200, {'ok': True, 'message': '目标已发送'})
            elif path == '/api/route':
                payload = self._read_json()
                self.bridge.send_route(payload['points'])
                self._json(200, {'ok': True, 'message': '路线已开始'})
            elif path == '/api/stop':
                self.bridge.cancel()
                self._json(200, {'ok': True, 'message': '导航已停止'})
            else:
                self._json(404, {'ok': False, 'message': '接口不存在'})
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._json(400, {'ok': False, 'message': str(error)})
        except RuntimeError as error:
            self._json(409, {'ok': False, 'message': str(error)})


def main():
    rclpy.init()
    bridge = GoalBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    GoalRequestHandler.bridge = bridge
    port = int(os.environ.get('GOAL_UI_PORT', '8090'))
    server = ThreadingHTTPServer(('0.0.0.0', port), GoalRequestHandler)
    print(f'Goal UI listening on 0.0.0.0:{port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
