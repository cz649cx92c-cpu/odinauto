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
        self.publisher = self.create_publisher(CtrlCmd, p('ctrl_cmd_topic'), 10)
        self.create_subscription(Twist, p('cmd_vel_topic'), self._command_callback, 10)
        self.create_timer(0.05, self._publish_command)

    def _command_callback(self, message):
        self.last_command = message
        self.last_received = time.monotonic()

    def _publish_command(self):
        message = CtrlCmd()
        if time.monotonic() - self.last_received <= self.timeout:
            lateral = max(-self.max_lateral, min(self.max_lateral, self.last_command.linear.y))
            if abs(lateral) >= self.lateral_threshold:
                # YHS mode 8 is lateral-only. Do not combine it with drive/yaw mode 6.
                message.ctrl_cmd_gear = self.lateral_gear
                message.ctrl_cmd_y_linear = lateral
            else:
                message.ctrl_cmd_gear = self.drive_gear
                message.ctrl_cmd_x_linear = max(-self.max_linear, min(self.max_linear, self.last_command.linear.x))
                message.ctrl_cmd_z_angular = max(-self.max_angular, min(self.max_angular, self.last_command.angular.z))
        else:
            message.ctrl_cmd_gear = self.drive_gear
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
            node._publish_command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
