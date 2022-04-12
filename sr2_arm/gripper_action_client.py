import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import argparse
from sr2_interfaces.action import Gripper


class GripperActionClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')

        self._action_client = ActionClient(self, Gripper, 'gripper')

    def send_goal(self, pos, vel=0.1, force=5.0):
        goal_msg = Gripper.Goal()
        goal_msg.pos = pos
        goal_msg.vel = vel
        goal_msg.force = force

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)



def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument('--pos', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init(args=None)

    node = GripperActionClient()

    future = node.send_goal(float(args.pos))

    rclpy.spin_until_future_complete(node, future)

if __name__ == "__main__":
    main()
