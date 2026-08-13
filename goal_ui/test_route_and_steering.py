#!/usr/bin/env python3
import threading
import time

import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from server import GoalBridge
from sru_nav_odin_ros2.navigation_node import NavigationNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Stamp:
    @staticmethod
    def to_msg():
        return Time()


class Clock:
    @staticmethod
    def now():
        return Stamp()


def odom(x=0.0, y=0.0):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    return message


def bridge():
    item = GoalBridge.__new__(GoalBridge)
    item.lock = threading.Lock()
    item.command_lock = threading.Lock()
    item.odom = odom()
    item.odom_received = time.monotonic()
    item.obstacle = False
    item.safe_velocity = (0.0, 0.0, 0.0)
    item.velocity_received = None
    item.target = None
    item.target_active = False
    item.route = []
    item.route_index = 0
    item.route_waypoint_distance = 0.32
    item.final_arrival_distance = 0.08
    item.last_event = ''
    item.goal_pub = Publisher()
    item.cancel_pub = Publisher()
    item.get_clock = lambda: Clock()
    return item


item = bridge()
item.send_route([
    {'x': 0.0, 'y': 0.0},
    {'x': 1.0, 'y': 0.0},
    {'x': 1.0, 'y': 1.0},
])
assert item.target == {'x': 1.0, 'y': 0.0}
assert len(item.goal_pub.messages) == 1

item._odom_callback(odom(1.0, 0.0))
assert item.route_index == 1
assert item.target == {'x': 1.0, 'y': 1.0}
assert len(item.goal_pub.messages) == 2

item.cancel()
assert not item.target_active and item.route == []
item._odom_callback(odom(1.0, 1.0))
assert len(item.goal_pub.messages) == 2

item.send_goal(2.0, 0.0)
assert item.target_active and item.route == []
item._odom_callback(odom(1.919, 0.0))
assert item.target_active
item._odom_callback(odom(1.921, 0.0))
assert not item.target_active

convert = NavigationNode._convert_to_steering
straight = convert([0.1, 0.0, 0.1], 0.8, 0.35, 0.25, 0.35)
assert np.allclose(straight, [0.1, 0.0, 0.035], atol=1e-5)

left = convert([0.1, 0.1, 0.0], 0.8, 0.35, 0.25, 0.35)
assert left[0] > 0.0 and left[1] == 0.0 and left[2] > 0.0

right = convert([0.1, -0.1, 0.0], 0.8, 0.35, 0.25, 0.35)
assert right[0] > 0.0 and right[1] == 0.0 and right[2] < 0.0

reverse = convert([-0.1, 0.0, 0.1], 0.8, 0.35, 0.25, 0.35)
assert reverse[0] < 0.0 and reverse[1] == 0.0

lateral_only = convert([0.0, 0.1, 0.0], 0.8, 0.35, 0.25, 0.35)
assert lateral_only[0] == 0.0 and lateral_only[1] == 0.0 and lateral_only[2] > 0.0

invalid = convert([np.nan, 0.1, 0.0], 0.8, 0.35, 0.25, 0.35)
assert np.allclose(invalid, [0.0, 0.0, 0.0])

guide = NavigationNode._apply_goal_heading
diagonal_left = guide([0.1, 0.0, 0.0], np.pi / 4, 0.8, 0.7, 0.35, 0.25)
assert diagonal_left[0] > 0.0 and diagonal_left[2] > 0.0

diagonal_right = guide([0.1, 0.0, 0.0], -np.pi / 4, 0.8, 0.7, 0.35, 0.25)
assert diagonal_right[0] > 0.0 and diagonal_right[2] < 0.0

straight_goal = guide([0.1, 0.0, 0.05], 0.0, 0.8, 0.7, 0.35, 0.25)
assert np.isclose(straight_goal[0], 0.1) and straight_goal[2] > 0.0

print('route and steering tests: PASS')
