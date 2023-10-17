#!/usr/bin/env python

import rclpy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
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


class SetModelStateClient(Node):
    def __init__(self):
        super().__init__("set_model_state_client")
        self.cli = self.create_client(SetEntityState, "/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetEntityState.Request()

    def send_request(self):
        self.req.state.name = "waffle"
        self.req.state.pose.position.x = -8.20
        self.req.state.pose.position.y = -2.40
        self.req.state.pose.position.z = 0.0
        yaw = 90.0
        quaternion = quaternion_from_euler(0.0, 0.0, math.radians(yaw))
        self.req.state.pose.orientation.x = quaternion[0]
        self.req.state.pose.orientation.y = quaternion[1]
        self.req.state.pose.orientation.z = quaternion[2]
        self.req.state.pose.orientation.w = quaternion[3]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = SetModelStateClient()
    node.send_request()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
