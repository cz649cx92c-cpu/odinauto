"""ROS 2 SRU policy node consuming Odin depth and odometry."""

import math
from pathlib import Path

import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

from .model import LearningModel


class NavigationNode(Node):
    def __init__(self):
        super().__init__('sru_nav_node')
        self._declare_parameters()
        p = lambda name: self.get_parameter(name).value
        self.min_depth = float(p('min_depth'))
        self.max_depth = float(p('max_depth'))
        self.period = 1.0 / float(p('control_frequency'))
        self.arrival_distance = float(p('arrival_distance'))
        self.steering_drive_mode = bool(p('steering_drive_mode'))
        self.lateral_to_steering_gain = max(0.0, float(p('lateral_to_steering_gain')))
        self.steering_max_angular = max(0.01, float(p('steering_max_angular')))
        self.steering_min_speed_factor = min(
            1.0, max(0.0, float(p('steering_min_speed_factor'))))
        self.steering_model_yaw_weight = min(
            1.0, max(0.0, float(p('steering_model_yaw_weight'))))
        self.last_action = np.zeros(3, dtype=np.float32)
        self.previous_cmd = np.zeros(3, dtype=np.float32)
        self.odom = None
        self.goal = None
        self.latest_depth = None
        self.latest_depth_time = None
        self.bridge = CvBridge()

        vae_path = self._model_path('vae_model_path', 'vae_encoder.onnx')
        policy_path = self._model_path('policy_model_path', 'nav_policy.onnx')
        self.model = LearningModel(
            vae_path, policy_path, p('policy_scale'), p('onnx_intra_op_threads'))
        self.get_logger().info('Loaded VAE: %s; policy: %s' % (vae_path, policy_path))

        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.navigation_active_pub = self.create_publisher(
            Bool, p('navigation_active_topic'), 10)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, p('odom_topic'), self._odom_callback, odom_qos)
        self.create_subscription(Image, p('depth_topic'), self._depth_callback, depth_qos)
        self.create_subscription(PoseStamped, p('goal_topic'), self._goal_callback, 10)
        self.create_subscription(Bool, p('cancel_topic'), self._cancel_callback, 10)
        self.create_timer(self.period, self._control_callback)

    def _declare_parameters(self):
        defaults = {
            'depth_topic': '/odin1/depth_img_competetion',
            'odom_topic': '/odin1/odometry_highfreq',
            'goal_topic': '/goal_pose',
            'cancel_topic': '/sru/cancel_navigation',
            'cmd_vel_topic': '/sru/cmd_vel',
            'navigation_active_topic': '/sru/navigation_active',
            'vae_model_path': '', 'policy_model_path': '',
            'min_depth': 0.25, 'max_depth': 10.0, 'control_frequency': 5.0,
            'onnx_intra_op_threads': 4,
            'policy_scale': [0.20, 0.12, 0.35],
            'arrival_distance': 0.50,
            # Convert the holonomic policy's normal lateral request into a
            # heading change. The safety layer can still request gear-8
            # lateral motion after this node when an emergency sidestep is needed.
            'steering_drive_mode': True,
            'lateral_to_steering_gain': 0.80,
            'steering_max_angular': 0.35,
            'steering_min_speed_factor': 0.25,
            'steering_model_yaw_weight': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _model_path(self, parameter, filename):
        configured = self.get_parameter(parameter).value
        if configured:
            path = Path(configured)
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('sru_nav_odin_ros2')) / 'models' / filename
        if not path.is_file():
            raise FileNotFoundError('Model does not exist: %s' % path)
        return str(path)

    def _odom_callback(self, message):
        self.odom = message

    def _goal_callback(self, message):
        if self.odom and message.header.frame_id and message.header.frame_id != self.odom.header.frame_id:
            self.get_logger().warning('Goal frame %s differs from odom frame %s; refusing goal.' %
                                      (message.header.frame_id, self.odom.header.frame_id))
            return
        goal_z = message.pose.position.z
        if abs(goal_z) < 1e-3 and self.odom is not None:
            goal_z = self.odom.pose.pose.position.z
        self.goal = np.array([message.pose.position.x, message.pose.position.y, goal_z],
                             dtype=np.float32)
        self.navigation_active_pub.publish(Bool(data=True))
        self.get_logger().info('Accepted navigation goal [%.2f, %.2f, %.2f].' % tuple(self.goal))

    def _cancel_callback(self, message):
        if not message.data:
            return
        self.goal = None
        self.navigation_active_pub.publish(Bool(data=False))
        self.model.reset()
        self.last_action.fill(0.0)
        self._publish_stop()
        self.get_logger().info('Navigation goal cancelled; command stopped.')

    def _depth_callback(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        except Exception as error:
            self.get_logger().error('Depth conversion failed: %s' % error)
            return
        depth = np.nan_to_num(np.asarray(depth, dtype=np.float32), nan=0.0,
                              posinf=self.max_depth * 2.0, neginf=0.0)
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0.0
        self.latest_depth = depth
        self.latest_depth_time = self.get_clock().now()

    def _control_callback(self):
        if self.odom is None or self.latest_depth is None:
            self._publish_stop()
            return

        depth_age = (self.get_clock().now() - self.latest_depth_time).nanoseconds * 1e-9
        if depth_age > 0.5:
            self.get_logger().warning(
                'Depth input is stale (%.2fs); stopping.' % depth_age,
                throttle_duration_sec=2.0)
            self._publish_stop()
            return

        if self.goal is None:
            self._publish_stop()
            return
        self._run_policy(self.latest_depth)

    @staticmethod
    def _convert_to_steering(command, gain, max_angular, min_speed_factor,
                             model_yaw_weight):
        converted = np.asarray(command, dtype=np.float32).copy()
        if converted.shape != (3,) or not np.isfinite(converted).all():
            return np.zeros(3, dtype=np.float32)
        forward, lateral = float(converted[0]), float(converted[1])
        planar_speed = math.hypot(forward, lateral)
        direction = -1.0 if forward < -1e-4 else 1.0
        desired_heading = math.atan2(lateral, max(abs(forward), 1e-4))
        speed_factor = max(min_speed_factor, max(0.0, math.cos(desired_heading)))
        converted[0] = (direction * planar_speed * speed_factor
                        if abs(forward) > 1e-4 else 0.0)
        converted[1] = 0.0
        converted[2] = np.clip(
            model_yaw_weight * converted[2] + direction * gain * desired_heading,
            -max_angular, max_angular)
        return converted

    def _run_policy(self, depth):
        pose = self.odom.pose.pose
        robot_position = [pose.position.x, pose.position.y, pose.position.z]
        distance = np.linalg.norm(
            self.goal[:2] - np.asarray(robot_position[:2], dtype=np.float32))
        if distance <= self.arrival_distance:
            self.get_logger().info('Goal reached.')
            self.goal = None
            self.navigation_active_pub.publish(Bool(data=False))
            self.model.reset()
            self.last_action.fill(0.0)
            self._publish_stop()
            return

        orientation = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        rotation = Rotation.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
        twist_world = self.odom.twist.twist
        linear = rotation.inv().apply([twist_world.linear.x, twist_world.linear.y, twist_world.linear.z])
        angular = rotation.inv().apply([twist_world.angular.x, twist_world.angular.y, twist_world.angular.z])
        gravity = rotation.inv().apply([0.0, 0.0, -1.0])
        command, self.last_action = self.model.predict(
            linear, angular, gravity, self.last_action, self.goal, robot_position, orientation, depth)
        if not np.isfinite(command).all():
            self.get_logger().error(
                'Policy produced a non-finite command; stopping.',
                throttle_duration_sec=2.0)
            self._publish_stop()
            return
        filtered = np.array([0.9, 0.5, 0.5]) * command + np.array([0.1, 0.5, 0.5]) * self.previous_cmd
        if self.steering_drive_mode:
            filtered = self._convert_to_steering(
                filtered, self.lateral_to_steering_gain,
                self.steering_max_angular, self.steering_min_speed_factor,
                self.steering_model_yaw_weight)
        self.previous_cmd = filtered
        output = Twist()
        output.linear.x, output.linear.y, output.angular.z = map(float, filtered)
        self.cmd_pub.publish(output)

    def _publish_stop(self):
        self.previous_cmd.fill(0.0)
        self.cmd_pub.publish(Twist())


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.navigation_active_pub.publish(Bool(data=False))
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
