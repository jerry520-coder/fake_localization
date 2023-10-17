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
    navigator = BasicNavigator()
    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 7.80
    initial_pose.pose.position.y = -2.40
    yaw = 0.0
    quaternion = quaternion_from_euler(0.0, 0.0, math.radians(yaw))
    initial_pose.pose.orientation.x = quaternion[0]
    initial_pose.pose.orientation.y = quaternion[1]
    initial_pose.pose.orientation.z = quaternion[2]
    initial_pose.pose.orientation.w = quaternion[3]
    navigator.setInitialPose(initial_pose)
    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    path = Path()
    path.header.frame_id = "map"
    path_maker = Marker()
    path_maker.header.frame_id = "map"
    path_maker.type = Marker.LINE_STRIP
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
            path_maker.points.append(pose.pose.position)
    # reverse the path
    path.poses.reverse()
    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                "Estimated distance remaining to goal position: "
                + "{0:.3f}".format(feedback.distance_to_goal)
                + "\nCurrent speed of the robot: "
                + "{0:.3f}".format(feedback.speed)
            )

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == "__main__":
    main()
