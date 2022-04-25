"""This module contains the Action server for the Gripper."""

__author__ = "Walking Machine"
__copyright__ = "Copyright (C) 2022 Walking Machine"
__license__ = "MIT"
__version__ = "1.0"

import argparse
from typing import Union
from math import floor

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sr2_interfaces.action import Gripper
from sr2_interfaces.msg import Heartbeat

from sr2_arm.robotiq_85.robotiq_test_gripper import RobotiqTestGripper
from sr2_arm.robotiq_85.robotiq_85_gripper import Robotiq85Gripper


class GripperActionServer(Node):
    """
    This Node is the server control for the gripper.

    This node receives requests from the :class:`GripperActionClient`.

    :param test_mode: Determines if the server is run in test mode, defaults to False
    """

    POS_MIN = 0.0
    """The minimum position of the gripper."""

    POS_MAX = 0.085
    """The Maximum position of the gripper."""

    VEL_MIN = 0.013
    """The minimum velocity of the gripper."""

    VEL_MAX = 0.1
    """The Maximum velocity of the gripper."""

    FORCE_MIN = 5.0
    """The Minimum force that the grippers can apply."""
    
    FORCE_MAX = 220.0
    """The Maximum force that the grippers can apply."""

    def __init__(self, test_mode=False) -> None:
        super().__init__('gripper_action_server')
        if test_mode:
            self.gripper = RobotiqTestGripper()
        else:
            self.gripper = Robotiq85Gripper()
        if not self.gripper.init_success:
            self.get_logger().error("Gripper not found")
            return

        # initialization of gripper
        self.gripper.process_act_cmd(0)
        self.gripper.process_stat_cmd(0)

        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper',
            self.execute_callback
        )

        if not test_mode:
            # teleop heartbeat
            self.teleop_heartbeat_sub = self.create_subscription(
                Heartbeat, "sr2/teleop/heartbeat", self.teleop_heartbeat_callback, 10)
            self.teleop_heartbeat_timer = self.create_timer(
                1, self.teleop_heartbeat_timer_callback)

        # flow control
        self.open_ = True

    def execute_callback(self, goal_handle) -> None:
        """
        Execute this method when the server receives a request.

        :param goal_handle: This is the request sent to the server
        """
        self.get_logger().info("executing goal")
        pos = self._clip_values(goal_handle.request.pos,
                                self.POS_MIN, self.POS_MAX)
        vel = self._clip_values(goal_handle.request.vel,
                                self.VEL_MIN, self.VEL_MAX)
        force = self._clip_values(
            goal_handle.request.force, self.FORCE_MIN, self.FORCE_MAX)

        self.gripper.goto(pos=pos, vel=vel, force=force)
        self.gripper.process_act_cmd(0)
        self.gripper.process_stat_cmd(0)
        while round(floor(self.gripper.get_pos()*pow(10, 3)) / pow(10, 3), 3) \
                != round(floor(self.gripper.get_req_pos()*pow(10, 3)) / pow(10, 3), 3):
            if not self.open_:
                return Gripper.Result()
            self.gripper.process_act_cmd(0)
            self.gripper.process_stat_cmd(0)

            feedback_msg = Gripper.Feedback()
            feedback_msg.cur_pos = self.gripper.get_pos()
            feedback_msg.obj_detected = self.gripper.object_detected()
            goal_handle.publish_feedback(feedback_msg)
            if feedback_msg.obj_detected:
                break

        goal_handle.succeed()
        result = Gripper.Result()
        result.final_pos = self.gripper.get_pos()
        result.obj_detected = self.gripper.object_detected()
        return result

    def teleop_heartbeat_callback(self, msg) -> None:
        """
        Handle heartbeat callback.

        This method enables the flow of the gripper. If the flow stops,
        the gripper will stop what it is doing.
        """
        self.teleop_heartbeat_timer.reset()
        self.open_ = True

    def teleop_heartbeat_timer_callback(self) -> None:
        """Heartbeat timer closes dataflow if triggered."""
        self.open_ = False
        self.get_logger().error("No connection to Teleop...")

    @staticmethod
    def _clip_values(value, min_, max_) -> Union[int, float]:
        if min_ <= value <= max_:
            return value
        if value < min_:
            return min_
        return max_


def main(args=None):
    """
    Start Node using CLI.

    CLI - options:
        :\--test: This starts the system using mock hardware.

    :param args: rclpy arguments, defaults to None
    """
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--test", action="store_true")

    rclpy.init(args=args)
    # args = parser.parse_args()
    node = GripperActionServer()

    if node.gripper.init_success:
        rclpy.spin(node)

    node.destroy_node()

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
