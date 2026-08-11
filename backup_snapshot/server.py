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
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


STATIC_DIR = Path(__file__).resolve().parent / 'static'


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
        self.route = []
        self.route_index = 0
        self.arrival_distance = 0.50
        self.last_event = '等待 Odin 里程计'

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Bool, '/sru/cancel_navigation', 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odin1/odometry', self._odom_callback, sensor_qos)
        self.create_subscription(Bool, '/sru/obstacle_ahead', self._obstacle_callback, 10)
        self.create_subscription(Twist, '/sru/safe_cmd_vel', self._velocity_callback, 10)

    def _odom_callback(self, message):
        with self.lock:
            first = self.odom is None
            self.odom = message
            self.odom_received = time.monotonic()
            if first:
                self.last_event = 'Odin 里程计已连接'
        next_waypoint = None
        with self.command_lock:
            with self.lock:
                if not self.target_active or not self.target:
                    return
                position = message.pose.pose.position
                distance = math.hypot(
                    self.target['x'] - float(position.x),
                    self.target['y'] - float(position.y))
                if distance <= self.arrival_distance:
                    if self.route and self.route_index + 1 < len(self.route):
                        self.route_index += 1
                        self.target = self.route[self.route_index].copy()
                        next_waypoint = (self.target['x'], self.target['y'])
                        self.last_event = '进入路点 %d / %d' % (
                            self.route_index + 1, len(self.route))
                    else:
                        self.target_active = False
                        self.last_event = '路线已完成' if self.route else '已进入目标范围'
            if next_waypoint is not None:
                self.goal_pub.publish(self._build_goal(*next_waypoint))

    def _obstacle_callback(self, message):
        with self.lock:
            self.obstacle = bool(message.data)

    def _velocity_callback(self, message):
        with self.lock:
            self.safe_velocity = (
                float(message.linear.x), float(message.linear.y), float(message.angular.z))
            self.velocity_received = time.monotonic()

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
            return {
                'connected': connected,
                'frame': frame,
                'position': position,
                'target': target,
                'target_active': active,
                'route': route,
                'route_index': self.route_index,
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
                self.last_event = f'目标已发送：X {x:.2f}，Y {y:.2f}'
            self.goal_pub.publish(message)

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
                while parsed and math.hypot(
                        parsed[0]['x'] - float(position.x),
                        parsed[0]['y'] - float(position.y)) <= self.arrival_distance:
                    parsed.pop(0)
            if not parsed:
                raise ValueError('路线终点距离当前位置太近')
            message = self._build_goal(parsed[0]['x'], parsed[0]['y'])
            with self.lock:
                self.route = parsed
                self.route_index = 0
                self.target = parsed[0].copy()
                self.target_active = True
                self.last_event = '路线已开始：1 / %d' % len(parsed)
            self.goal_pub.publish(message)

    def cancel(self):
        with self.command_lock:
            with self.lock:
                self.target_active = False
                self.route = []
                self.route_index = 0
                self.last_event = '导航已停止'
            self.cancel_pub.publish(Bool(data=True))


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
