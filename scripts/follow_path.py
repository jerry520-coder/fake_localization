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

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import (
    FollowPath,
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
)
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import math
from nav_msgs.msg import Path


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


def feedbackCallback(feedback_msg):
    print("feedback_msg", feedback_msg)


def main():
    rclpy.init()
    navigator = rclpy.create_node("navigator")
    follow_path_client = ActionClient(navigator, FollowPath, "/follow_path")
    # wait for action server to start up
    while not follow_path_client.wait_for_server(timeout_sec=1.0):
        navigator.get_logger().info(
            "FollowPath action server not available, waiting..."
        )
    # Set our demo's initial pose
    path = Path()
    path.header.frame_id = "map"
    with open(PATH_DATA, "r") as f:
        # read x, y , theta
        for line in f:
            x, y, theta = line.split()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = float(theta)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
    #path.poses.reverse()
    controller_id = ""
    goal_checker_id = ""
    goal_msg = FollowPath.Goal()
    goal_msg.path = path
    goal_msg.controller_id = controller_id
    goal_msg.goal_checker_id = goal_checker_id
    navigator.get_logger().info("Executing path...")
    send_goal_future = follow_path_client.send_goal_async(
        goal_msg, feedbackCallback
    )
    rclpy.spin_until_future_complete(navigator, send_goal_future)
    follow_path_client.goal_handle = send_goal_future.result()

    if not follow_path_client.goal_handle.accepted:
        navigator.get_logger.error("Follow path was rejected!")
        return

    result_future = follow_path_client.goal_handle.get_result_async()
    print("result_future", result_future)
    exit(0)


if __name__ == "__main__":
    main()
