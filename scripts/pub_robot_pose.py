#!/usr/bin/env python

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from visualization_msgs.msg import Marker
import math


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

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_pose_publisher")
        self.sub = self.create_subscription(
            ModelStates,
            "/model_states",
            self.callback,
            10,
        )
        self.model_state_marker_pub = self.create_publisher(
            Marker,
            "/model_state_marker",
            10,
        )    
    def callback(self, msg : ModelStates):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        name_list = msg.name
        pose_list = msg.pose
        print(name_list)
        for name in name_list:
            if name == 'waffle':
                pose = pose_list[name_list.index(name)]
                print(pose)
                marker.pose = pose
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.5
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                self.model_state_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()    