"""Safety adapter from geometry_msgs/Twist to the installed YHS CAN interface."""

import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from yhs_can_interfaces.msg import CtrlCmd


class YhsAdapter(Node):
    def __init__(self):
        super().__init__('sru_twist_to_yhs')
        self.declare_parameter('cmd_vel_topic', '/sru/safe_cmd_vel')
        self.declare_parameter('ctrl_cmd_topic', '/ctrl_cmd')
        self.declare_parameter('max_linear_x', 0.20)
        self.declare_parameter('max_linear_y', 0.12)
        self.declare_parameter('max_angular_z', 0.35)
        self.declare_parameter('command_timeout', 0.50)
        self.declare_parameter('drive_gear', 6)
        self.declare_parameter('lateral_gear', 8)
        self.declare_parameter('lateral_activation_threshold', 0.02)
        self.declare_parameter('gear_stop_cycles', 2)
        self.declare_parameter('gear_settle_cycles', 2)
        self.last_command = Twist()
        self.last_received = 0.0
        p = lambda name: self.get_parameter(name).value
        self.max_linear = float(p('max_linear_x'))
        self.max_lateral = float(p('max_linear_y'))
        self.max_angular = float(p('max_angular_z'))
        self.timeout = float(p('command_timeout'))
        self.drive_gear = int(p('drive_gear'))
        self.lateral_gear = int(p('lateral_gear'))
        self.lateral_threshold = float(p('lateral_activation_threshold'))
        self.gear_stop_cycles = max(1, int(p('gear_stop_cycles')))
        self.gear_settle_cycles = max(1, int(p('gear_settle_cycles')))
        self.active_gear = self.drive_gear
        self.transition_target = None
        self.transition_phase = 'RUN'
        self.transition_remaining = 0
        self.publisher = self.create_publisher(CtrlCmd, p('ctrl_cmd_topic'), 10)
        self.create_subscription(Twist, p('cmd_vel_topic'), self._command_callback, 10)
        self.create_timer(0.05, self._publish_command)

    def _command_callback(self, message):
        self.last_command = message
        self.last_received = time.monotonic()

    def _publish_command(self):
        message = CtrlCmd()
        command_valid = time.monotonic() - self.last_received <= self.timeout
        desired_gear = self.drive_gear
        lateral = 0.0
        if command_valid:
            lateral = max(-self.max_lateral, min(self.max_lateral, self.last_command.linear.y))
            if abs(lateral) >= self.lateral_threshold:
                desired_gear = self.lateral_gear

        if self.transition_phase == 'RUN' and desired_gear != self.active_gear:
            self.transition_target = desired_gear
            self.transition_phase = 'STOP_CURRENT'
            self.transition_remaining = self.gear_stop_cycles
        elif (self.transition_phase != 'RUN' and
              desired_gear != self.transition_target):
            # A changed request restarts the interlock from the gear currently
            # being emitted. No powered command is sent during a transition.
            self.transition_target = desired_gear
            self.transition_phase = 'STOP_CURRENT'
            self.transition_remaining = self.gear_stop_cycles

        if self.transition_phase == 'STOP_CURRENT':
            message.ctrl_cmd_gear = self.active_gear
            self.transition_remaining -= 1
            if self.transition_remaining <= 0:
                self.active_gear = self.transition_target
                self.transition_phase = 'SETTLE_TARGET'
                self.transition_remaining = self.gear_settle_cycles
        elif self.transition_phase == 'SETTLE_TARGET':
            message.ctrl_cmd_gear = self.active_gear
            self.transition_remaining -= 1
            if self.transition_remaining <= 0:
                self.transition_phase = 'RUN'
                self.transition_target = None
        else:
            message.ctrl_cmd_gear = self.active_gear
            if command_valid and self.active_gear == self.lateral_gear:
                # YHS mode 8 is lateral-only. Do not combine it with drive/yaw mode 6.
                message.ctrl_cmd_y_linear = lateral
            elif command_valid and self.active_gear == self.drive_gear:
                message.ctrl_cmd_x_linear = max(
                    -self.max_linear, min(self.max_linear, self.last_command.linear.x))
                message.ctrl_cmd_z_angular = max(
                    -self.max_angular, min(self.max_angular, self.last_command.angular.z))
        self.publisher.publish(message)


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = YhsAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.last_command = Twist()
            node.last_received = 0.0
            node._publish_command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
