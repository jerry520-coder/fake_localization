#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import rclpy
import os
import math
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time

PATH_DATA = os.path.join(os.path.dirname(__file__), "path_points.txt")

"""
Basic navigation demo to follow a given path
"""


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


def main():
    rclpy.init()
    rclpy_node = rclpy.create_node("path_marker")
    path_marker_pub = rclpy_node.create_publisher(
        Marker,
        "/path_marker",
        QoSProfile(depth=1),
    )
    path = Path()
    path.header.frame_id = "map"
    path_maker = Marker()
    path_maker.header.frame_id = "map"
    path_maker.type = Marker.LINE_STRIP
    path_maker.color.r = 1.0
    path_maker.color.g = 0.0
    path_maker.color.b = 0.0
    path_maker.color.a = 1.0  # Don't forget to set the alpha!
    path_maker.scale.x = 0.1  # for example, 0.1m width lines
    with open(PATH_DATA, "r") as f:
        # read x, y , theta
        for line in f:
            x, y, theta = line.split()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rclpy_node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = float(theta)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            path_maker.points.append(pose.pose.position)
    while True:
        path_marker_pub.publish(path_maker)
        time.sleep(0.1)
    


if __name__ == "__main__":
    main()
