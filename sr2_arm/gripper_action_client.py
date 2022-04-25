# Copyright (c) 2022 Walking Machine
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

__author__ = "WalkingMachine"
__copyright__ = "Copyright (C) 2022 Walking Machine"
__license__ = "MIT"
__version__ = "1.0"

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import argparse
from sr2_interfaces.action import Gripper


class GripperActionClient(Node):
    """
    This node sends a request to the :class: `GripperActionServer`.

    The request sends a position for the grippers, this is open, closed, or
    somewhere in between.
    The return will be send when the goal is reached, or if there was an
    object detected.
    """

    def __init__(self):
        super().__init__('gripper_action_client')

        self._action_client = ActionClient(self, Gripper, 'gripper')

    def send_goal(self, pos, vel=0.1, force=5.0) -> None:
        """
        Send the position that we want the grippers to be at.

        The force is the max exertion before the gripper determins if there
        is an object or not.

        :param pos: The requested position of the gripper
        :param vel: The speed of the grippers movement, defaults to 0.1
        :param force: The force applied to the grippers to move them, defaults to 5.0
        """
        goal_msg = Gripper.Goal()
        goal_msg.pos = pos
        goal_msg.vel = vel
        goal_msg.force = force

        # Wait for the server to be active
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """
        Handle the callback with the status of the goal from the server.

        :param future: The future request from the server
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """
        Handle the result from the server when the goal is complete.

        :param future: The future of the task
        """
        result = future.result().result
        self.get_logger().info('Result: {0}, {1}'.format(
            result.final_pos, result.obj_detected))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg) -> None:
        """
        Handle the feedback from the server while it is processing the goal.

        :param feedback_msg: Message from the server
        """
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}, {1}'.format(
            feedback.cur_pos, feedback.obj_detected))


def main(args=None):
    """
    Start client using CLI.

    CLI - Parameters:
        :\ --pos <int>: The position of the gripper.

    :param args: rclpy arguments, defaults to None
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--pos', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init(args=None)

    node = GripperActionClient()

    node.send_goal(float(args.pos))

    rclpy.spin(node)


if __name__ == "__main__":
    main()
