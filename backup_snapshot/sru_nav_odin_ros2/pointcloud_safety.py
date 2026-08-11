"""Point-cloud safety gate using the measured Odin mounting transform."""

import math
import time

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class PointCloudSafety(Node):
    def __init__(self):
        super().__init__('sru_pointcloud_safety')
        self._declare_parameters()
        p = lambda name: self.get_parameter(name).value
        self.cloud_timeout = float(p('cloud_timeout'))
        self.cloud_processing_period = 1.0 / float(p('cloud_processing_frequency'))
        self.fail_safe = bool(p('fail_safe_on_cloud_timeout'))
        self.last_cloud_time = None
        self.latest_cloud = None
        self.latest_cloud_receive_time = None
        self.cached_ground_plane = None
        self.last_ground_fit_time = None
        self.last_diagnostics_time = 0.0
        self.rng = np.random.default_rng(0)
        self.block_forward = True
        self.block_left = True
        self.block_right = True
        self.safety_valid = False
        self.left_obstacle_count = 0
        self.right_obstacle_count = 0
        self.avoidance_direction = 0
        self.avoidance_direction_since = None
        self.navigation_active = False
        self.latest_requested_command = Twist()
        self.latest_command_time = None
        self.latest_odom = None
        self.latest_odom_time = None
        self.avoidance_state = 'IDLE'
        self.state_enter_time = time.monotonic()
        self.episode_start_time = None
        self.sidestep_start_pose = None
        self.sidestep_lateral_axis = None
        self.front_clear_frames = 0
        self.left_assert_frames = 0
        self.left_clear_frames = 0
        self.right_assert_frames = 0
        self.right_clear_frames = 0
        self.rotation, self.translation = self._mount_transform()

        self.output_pub = self.create_publisher(Twist, p('safe_cmd_vel_topic'), 10)
        self.obstacle_pub = self.create_publisher(Bool, p('obstacle_ahead_topic'), 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, p('cloud_topic'), self._cloud_callback, sensor_qos)
        self.create_subscription(Twist, p('cmd_vel_topic'), self._command_callback, 10)
        self.create_subscription(Bool, p('navigation_active_topic'),
                                 self._navigation_active_callback, 10)
        self.create_subscription(Odometry, p('odom_topic'), self._odom_callback, sensor_qos)
        self.create_timer(self.cloud_processing_period, self._process_latest_cloud)
        self.create_timer(1.0 / float(p('safety_control_frequency')), self._control_tick)

    def _declare_parameters(self):
        defaults = {
            'cloud_topic': '/odin1/cloud_raw',
            'cmd_vel_topic': '/sru/cmd_vel',
            'navigation_active_topic': '/sru/navigation_active',
            'odom_topic': '/odin1/odometry',
            'safe_cmd_vel_topic': '/sru/safe_cmd_vel',
            'obstacle_ahead_topic': '/sru/obstacle_ahead',
            # Transform from odin1_base_link coordinates into base_link coordinates.
            # Odin center is 5 cm ahead of the vehicle center (base_link).
            'odin_x': 0.05, 'odin_y': 0.0, 'odin_z': 1.0,
            'odin_roll_deg': -1.20,
            # Positive here means that the Odin forward axis is tilted downward.
            'odin_pitch_down_deg': 7.72,
            'odin_yaw_deg': 0.0,
            # A point must be this far above the fitted ground plane to block motion.
            'ground_clearance': 0.10,
            'obstacle_max_height': 1.50,
            'ground_fit_min_distance': 0.40,
            'ground_fit_max_distance': 4.00,
            'ground_fit_half_width': 1.50,
            'ground_fit_max_tilt_deg': 15.0,
            'ground_fit_inlier_distance': 0.04,
            'ground_fit_iterations': 20,
            'ground_fit_min_inliers': 300,
            # Collision checks stay at 5 Hz; the more expensive ground fit is
            # refreshed independently because the mounting transform is fixed.
            'ground_fit_frequency': 2.0,
            'ground_plane_max_age': 0.75,
            'front_min_distance': 0.30,
            # Keep this as an emergency stop zone. The SRU policy uses depth
            # to handle obstacles farther ahead without freezing in a room.
            'front_max_distance': 0.50,
            'front_half_width': 0.35,
            'side_min_distance': 0.20,
            'side_max_distance': 0.55,
            'side_half_length': 0.40,
            'minimum_obstacle_points': 20,
            'cloud_processing_frequency': 5.0,
            'max_cloud_points': 12000,
            'cloud_timeout': 0.75,
            'fail_safe_on_cloud_timeout': True,
            'diagnostics_log_frequency': 0.5,
            # When SRU requests forward motion into a blocked corridor, sidestep
            # toward the safer side until the forward corridor becomes clear.
            'enable_lateral_avoidance_assist': True,
            'lateral_avoidance_speed': 0.06,
            'forward_command_threshold': 0.01,
            'lateral_command_threshold': 0.02,
            'avoidance_direction_hold_time': 1.5,
            'avoidance_clear_margin_points': 5,
            'safety_control_frequency': 10.0,
            'raw_command_timeout': 0.60,
            'odom_timeout': 0.75,
            'shift_pause': 0.20,
            'min_lateral_distance': 0.20,
            'max_lateral_distance': 0.55,
            'front_clear_frames_required': 4,
            'side_clear_frames_required': 3,
            'obstacle_assert_frames': 2,
            'obstacle_clear_frames': 3,
            'recovery_ramp_time': 0.80,
            'sidestep_timeout': 8.0,
            'avoidance_episode_timeout': 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _mount_transform(self):
        p = lambda name: self.get_parameter(name).value
        roll = math.radians(float(p('odin_roll_deg')))
        pitch = math.radians(float(p('odin_pitch_down_deg')))
        yaw = math.radians(float(p('odin_yaw_deg')))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=np.float32)
        translation = np.array([p('odin_x'), p('odin_y'), p('odin_z')], dtype=np.float32)
        return rotation, translation

    @staticmethod
    def _xyz(message, max_points):
        fields = {field.name: field for field in message.fields}
        if not {'x', 'y', 'z'} <= fields.keys():
            return np.empty((0, 3), dtype=np.float32), 1
        byte_order = '>' if message.is_bigendian else '<'
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [byte_order + 'f4'] * 3,
            'offsets': [fields['x'].offset, fields['y'].offset, fields['z'].offset],
            'itemsize': message.point_step,
        })
        cloud = np.frombuffer(message.data, dtype=dtype)
        # Subsample the zero-copy structured view before making the contiguous
        # XYZ array. A raw Odin cloud can contain hundreds of thousands of
        # points, while the safety gate only needs a representative subset.
        sample_stride = max(1, math.ceil(len(cloud) / max_points))
        if sample_stride > 1:
            cloud = cloud[::sample_stride]
        return np.column_stack((cloud['x'], cloud['y'], cloud['z'])), sample_stride

    def _fit_ground_plane(self, points, sample_stride):
        """Fit z = ax + by + c with RANSAC in the forward ground region."""
        p = lambda name: self.get_parameter(name).value
        region = points[
            (points[:, 0] >= float(p('ground_fit_min_distance'))) &
            (points[:, 0] <= float(p('ground_fit_max_distance'))) &
            (np.abs(points[:, 1]) <= float(p('ground_fit_half_width')))
        ]
        if len(region) < 3:
            return None

        # Restrict sampling to lower points so vertical objects cannot become ground.
        lower = region[region[:, 2] <= np.percentile(region[:, 2], 45)]
        if len(lower) < 3:
            return None
        max_tilt_cos = math.cos(math.radians(float(p('ground_fit_max_tilt_deg'))))
        inlier_distance = float(p('ground_fit_inlier_distance'))
        best_mask = None
        best_count = 0
        for _ in range(int(p('ground_fit_iterations'))):
            sample = lower[self.rng.choice(len(lower), size=3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            if normal[2] < 0.0:
                normal = -normal
            if normal[2] < max_tilt_cos:
                continue
            offset = -float(np.dot(normal, sample[0]))
            distance = (region[:, 0] * normal[0] + region[:, 1] * normal[1] +
                        region[:, 2] * normal[2] + offset)
            mask = np.abs(distance) <= inlier_distance
            count = int(mask.sum())
            if count > best_count:
                best_mask, best_count = mask, count
        minimum_inliers = max(3, math.ceil(int(p('ground_fit_min_inliers')) / sample_stride))
        if best_mask is None or best_count < minimum_inliers:
            return None

        # Refine using all RANSAC inliers: the least-squares plane is less noisy.
        inliers = region[best_mask]
        matrix = np.column_stack((inliers[:, 0], inliers[:, 1], np.ones(len(inliers))))
        coefficients, _, _, _ = np.linalg.lstsq(matrix, inliers[:, 2], rcond=None)
        normal = np.array([-coefficients[0], -coefficients[1], 1.0], dtype=np.float32)
        normal /= np.linalg.norm(normal)
        if normal[2] < max_tilt_cos:
            return None
        return normal, -float(coefficients[2]) / float(np.linalg.norm(
            np.array([-coefficients[0], -coefficients[1], 1.0])))

    def _cloud_callback(self, message):
        # KEEP_LAST(1) plus this assignment avoids processing an old backlog.
        self.latest_cloud = message
        self.latest_cloud_receive_time = time.monotonic()

    def _publish_blocked(self, reason):
        self.safety_valid = False
        self.block_forward = True
        self.block_left = True
        self.block_right = True
        self.left_obstacle_count = 0
        self.right_obstacle_count = 0
        self.avoidance_direction = 0
        self.avoidance_direction_since = None
        self.obstacle_pub.publish(Bool(data=True))
        self.output_pub.publish(Twist())
        self.get_logger().warning(reason, throttle_duration_sec=2.0)

    def _log_diagnostics(self, now, front_count, nearest_distance, ground_valid):
        frequency = float(self.get_parameter('diagnostics_log_frequency').value)
        if frequency <= 0.0 or now - self.last_diagnostics_time < 1.0 / frequency:
            return
        self.last_diagnostics_time = now
        nearest = f'{nearest_distance:.2f}m' if math.isfinite(nearest_distance) else 'none'
        self.get_logger().info(
            f'safety: ground_valid={ground_valid} front_points={front_count} '
            f'nearest_front={nearest} left_points={self.left_obstacle_count} '
            f'right_points={self.right_obstacle_count} blocked={self.block_forward}')

    def _process_latest_cloud(self):
        message = self.latest_cloud
        receive_time = self.latest_cloud_receive_time
        now = time.monotonic()
        if message is None:
            cloud_stale = receive_time is None or now - receive_time > self.cloud_timeout
            if cloud_stale and self.fail_safe:
                self._publish_blocked('Point cloud is stale; blocking motion for safety.')
            else:
                self.obstacle_pub.publish(Bool(data=self.block_forward))
            return
        # Callbacks run on one executor thread, so consuming the newest sample
        # requires no lock and a newer cloud cannot be overwritten here.
        self.latest_cloud = None
        max_points = max(1, int(self.get_parameter('max_cloud_points').value))
        points, sample_stride = self._xyz(message, max_points)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.einsum('ij,ij->i', points, points) > 0.0025]
        if points.size:
            points = points @ self.rotation.T + self.translation

        fit_frequency = float(self.get_parameter('ground_fit_frequency').value)
        fit_period = 1.0 / max(fit_frequency, 1e-3)
        max_plane_age = float(self.get_parameter('ground_plane_max_age').value)
        fit_due = (self.cached_ground_plane is None or self.last_ground_fit_time is None or
                   now - self.last_ground_fit_time >= fit_period or
                   now - self.last_ground_fit_time > max_plane_age)
        if fit_due:
            plane = self._fit_ground_plane(points, sample_stride)
            if plane is not None:
                self.cached_ground_plane = plane
                self.last_ground_fit_time = now
            elif (self.last_ground_fit_time is None or
                  now - self.last_ground_fit_time > max_plane_age):
                # A single failed refresh must not stop the vehicle, but an old
                # or never-valid plane still fails closed.
                self.cached_ground_plane = None
        plane = self.cached_ground_plane
        if plane is None:
            self.last_cloud_time = receive_time
            self._publish_blocked('No reliable ground plane; blocking motion for safety.')
            self._log_diagnostics(now, 0, math.inf, False)
            return
        normal, offset = plane
        p = lambda name: self.get_parameter(name).value
        height_above_ground = points @ normal + offset
        height = ((height_above_ground >= float(p('ground_clearance'))) &
                  (height_above_ground <= float(p('obstacle_max_height'))))
        candidates = points[height]
        minimum = max(3, math.ceil(int(p('minimum_obstacle_points')) / sample_stride))
        front = ((candidates[:, 0] >= float(p('front_min_distance'))) &
                 (candidates[:, 0] <= float(p('front_max_distance'))) &
                 (np.abs(candidates[:, 1]) <= float(p('front_half_width'))))
        left = ((candidates[:, 1] >= float(p('side_min_distance'))) &
                (candidates[:, 1] <= float(p('side_max_distance'))) &
                (np.abs(candidates[:, 0]) <= float(p('side_half_length'))))
        right = ((candidates[:, 1] <= -float(p('side_min_distance'))) &
                 (candidates[:, 1] >= -float(p('side_max_distance'))) &
                 (np.abs(candidates[:, 0]) <= float(p('side_half_length'))))
        front_count = int(front.sum())
        self.left_obstacle_count = int(left.sum())
        self.right_obstacle_count = int(right.sum())
        self.block_forward = front_count >= minimum
        if self.block_forward:
            self.front_clear_frames = 0
        else:
            self.front_clear_frames += 1
        self.block_left, self.left_assert_frames, self.left_clear_frames = \
            self._update_hysteresis(
                self.block_left, self.left_obstacle_count >= minimum,
                self.left_assert_frames, self.left_clear_frames)
        self.block_right, self.right_assert_frames, self.right_clear_frames = \
            self._update_hysteresis(
                self.block_right, self.right_obstacle_count >= minimum,
                self.right_assert_frames, self.right_clear_frames)
        self.safety_valid = True
        front_points = candidates[front]
        nearest_distance = (float(front_points[:, 0].min())
                            if len(front_points) else math.inf)
        self.last_cloud_time = receive_time
        self.obstacle_pub.publish(Bool(data=self.block_forward))
        self._log_diagnostics(now, front_count, nearest_distance, True)

    def _update_hysteresis(self, blocked, raw_blocked, assert_frames, clear_frames):
        p = lambda name: self.get_parameter(name).value
        if raw_blocked:
            assert_frames += 1
            clear_frames = 0
            if assert_frames >= int(p('obstacle_assert_frames')):
                blocked = True
        else:
            clear_frames += 1
            assert_frames = 0
            if clear_frames >= int(p('obstacle_clear_frames')):
                blocked = False
        return blocked, assert_frames, clear_frames

    @staticmethod
    def _select_lateral_direction(block_left, block_right, left_count, right_count,
                                  previous_direction, hold_previous, clear_margin):
        """Return +1 for left, -1 for right, or 0 when neither side is safe."""
        if block_left and block_right:
            return 0
        if block_left:
            return -1
        if block_right:
            return 1
        if previous_direction in (-1, 1):
            if hold_previous:
                return previous_direction
            if previous_direction > 0:
                return -1 if right_count + clear_margin < left_count else 1
            return 1 if left_count + clear_margin < right_count else -1
        return 1 if left_count <= right_count else -1

    def _command_callback(self, message):
        self.latest_requested_command = message
        self.latest_command_time = time.monotonic()

    def _navigation_active_callback(self, message):
        self.navigation_active = bool(message.data)
        if not self.navigation_active:
            self._reset_avoidance('IDLE')
            self.output_pub.publish(Twist())

    def _odom_callback(self, message):
        self.latest_odom = message
        self.latest_odom_time = time.monotonic()

    def _enter_state(self, state, now):
        if state != self.avoidance_state:
            self.get_logger().info(
                f'avoidance state: {self.avoidance_state} -> {state}')
            self.avoidance_state = state
            self.state_enter_time = now

    def _reset_avoidance(self, state='CRUISE'):
        self.avoidance_state = state
        self.state_enter_time = time.monotonic()
        self.episode_start_time = None
        self.sidestep_start_pose = None
        self.sidestep_lateral_axis = None
        self.avoidance_direction = 0
        self.avoidance_direction_since = None

    def _side_blocked(self, direction):
        return self.block_left if direction > 0 else self.block_right

    def _choose_avoidance_direction(self):
        return self._select_lateral_direction(
            self.block_left, self.block_right,
            self.left_obstacle_count, self.right_obstacle_count,
            self.avoidance_direction, self.avoidance_direction != 0,
            int(self.get_parameter('avoidance_clear_margin_points').value))

    def _current_pose_2d(self):
        pose = self.latest_odom.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        return np.array([pose.position.x, pose.position.y]), yaw

    def _begin_sidestep(self, now):
        position, yaw = self._current_pose_2d()
        self.sidestep_start_pose = position
        self.sidestep_lateral_axis = np.array([-math.sin(yaw), math.cos(yaw)])
        self._enter_state('SIDESTEP', now)

    def _lateral_distance(self):
        if self.sidestep_start_pose is None or self.latest_odom is None:
            return 0.0
        position, _ = self._current_pose_2d()
        displacement = float(np.dot(
            position - self.sidestep_start_pose, self.sidestep_lateral_axis))
        return self.avoidance_direction * displacement

    @staticmethod
    def _copy_command(source):
        output = Twist()
        output.linear.x = source.linear.x
        output.linear.y = source.linear.y
        output.angular.z = source.angular.z
        return output

    def _inputs_healthy(self, now):
        p = lambda name: self.get_parameter(name).value
        cloud_ok = (self.last_cloud_time is not None and
                    now - self.last_cloud_time <= self.cloud_timeout and
                    self.safety_valid)
        command_ok = (self.latest_command_time is not None and
                      now - self.latest_command_time <= float(p('raw_command_timeout')))
        odom_ok = (self.latest_odom_time is not None and
                   now - self.latest_odom_time <= float(p('odom_timeout')))
        return cloud_ok and command_ok and odom_ok

    def _control_tick(self):
        now = time.monotonic()
        output = Twist()
        p = lambda name: self.get_parameter(name).value

        if not self.navigation_active:
            if self.avoidance_state != 'IDLE':
                self._reset_avoidance('IDLE')
            self.output_pub.publish(output)
            return
        if not self._inputs_healthy(now):
            self._enter_state('SENSOR_FAULT', now)
            self.output_pub.publish(output)
            return
        if self.avoidance_state == 'SENSOR_FAULT':
            self._enter_state('CRUISE', now)

        if (self.episode_start_time is not None and
                now - self.episode_start_time > float(p('avoidance_episode_timeout'))):
            self._enter_state('ABORT_STOP', now)

        state = self.avoidance_state
        requested = self.latest_requested_command
        if state in ('IDLE', 'CRUISE'):
            if self.block_forward and bool(p('enable_lateral_avoidance_assist')):
                direction = self._choose_avoidance_direction()
                if direction == 0:
                    self._enter_state('WAIT_BLOCKED', now)
                else:
                    self.avoidance_direction = direction
                    self.avoidance_direction_since = now
                    self.episode_start_time = now
                    self._enter_state('SHIFT_TO_LATERAL', now)
            else:
                self._enter_state('CRUISE', now)
                output = self._copy_command(requested)
                output.linear.y = 0.0

        elif state == 'SHIFT_TO_LATERAL':
            if self._side_blocked(self.avoidance_direction):
                self._enter_state('WAIT_BLOCKED', now)
            elif now - self.state_enter_time >= float(p('shift_pause')):
                self._begin_sidestep(now)

        elif state == 'SIDESTEP':
            distance = self._lateral_distance()
            if self._side_blocked(self.avoidance_direction):
                self._enter_state('WAIT_BLOCKED', now)
            elif (distance >= float(p('max_lateral_distance')) or
                  now - self.state_enter_time >= float(p('sidestep_timeout'))):
                self._enter_state('ABORT_STOP', now)
            elif (distance >= float(p('min_lateral_distance')) and
                  self.front_clear_frames >= int(p('front_clear_frames_required'))):
                self._enter_state('SHIFT_TO_FORWARD', now)
            else:
                output.linear.y = (self.avoidance_direction *
                                   float(p('lateral_avoidance_speed')))

        elif state == 'WAIT_BLOCKED':
            if (not self.block_forward and
                    self.front_clear_frames >= int(p('front_clear_frames_required'))):
                self._enter_state('SHIFT_TO_FORWARD', now)
            elif self.avoidance_direction == 0:
                direction = self._choose_avoidance_direction()
                if direction:
                    self.avoidance_direction = direction
                    self.episode_start_time = self.episode_start_time or now
                    self._enter_state('SHIFT_TO_LATERAL', now)
            elif not self._side_blocked(self.avoidance_direction):
                clear_frames = (self.left_clear_frames if self.avoidance_direction > 0
                                else self.right_clear_frames)
                if clear_frames >= int(p('side_clear_frames_required')):
                    self._enter_state('SHIFT_TO_LATERAL', now)

        elif state == 'SHIFT_TO_FORWARD':
            if self.block_forward:
                self._enter_state('SHIFT_TO_LATERAL', now)
            elif now - self.state_enter_time >= float(p('shift_pause')):
                self._enter_state('RECOVER_FORWARD', now)

        elif state == 'RECOVER_FORWARD':
            if self.block_forward:
                if self._side_blocked(self.avoidance_direction):
                    self._enter_state('WAIT_BLOCKED', now)
                else:
                    self._enter_state('SHIFT_TO_LATERAL', now)
            else:
                output = self._copy_command(requested)
                output.linear.y = 0.0
                ramp = min(1.0, (now - self.state_enter_time) /
                           max(float(p('recovery_ramp_time')), 1e-3))
                output.linear.x *= ramp
                if ramp >= 1.0:
                    self._reset_avoidance('CRUISE')

        elif state == 'ABORT_STOP':
            if self.front_clear_frames >= int(p('front_clear_frames_required')) + 1:
                self._reset_avoidance('CRUISE')

        self.output_pub.publish(output)


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = PointCloudSafety()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
