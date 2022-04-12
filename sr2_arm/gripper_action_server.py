from math import floor
from time import sleep
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sr2_interfaces.action import Gripper
from sr2_interfaces.msg import Heartbeat
from .robotiq_85.robotiq_85_gripper import Robotiq85Gripper

from threading import Thread


class GripperActionServer(Node):

    _POS_MIN = 0.0
    _POS_MAX = 0.085
    _VEL_MIN = 0.013
    _VEL_MAX = 0.1
    _FORCE_MIN = 5.0
    _FORCE_MAX = 220.0

    def __init__(self):
        super().__init__('gripper_action_server')
        self.gripper = Robotiq85Gripper(comport="/dev/gripper")
        if not self.gripper.init_success:
            self.get_logger().error("Gripper not found")
            return

        print(self.gripper.is_ready())
        # initialization of gripper
        self.gripper.process_act_cmd(0)
        self.gripper.process_stat_cmd(0)
        
        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper',
            self.execute_callback
        )

        # teleop heartbeat
        self.teleop_heartbeat_sub = self.create_subscription(Heartbeat, "sara/teleop/heartbeat", self.teleop_heartbeat_callback, 10)
        self.teleop_heartbeat_timer = self.create_timer(1, self.teleop_heartbeat_timer_callback)

        # flow control
        self.open_ = True

    def _clip_values(self, value, min, max):
        if min <= value <= max:
            return value
        elif value < min:
            return min
        elif value > max:
            return max

    def execute_callback(self, goal_handle):
        self.get_logger().info("executing goal")
        pos = self._clip_values(goal_handle.request.pos, self._POS_MIN, self._POS_MAX)
        vel = self._clip_values(goal_handle.request.vel, self._VEL_MIN, self._VEL_MAX)
        force = self._clip_values(goal_handle.request.force, self._FORCE_MIN, self._FORCE_MAX)

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

    def teleop_heartbeat_callback(self, msg):
        self.teleop_heartbeat_timer.reset()
        self.open_ = True

    
    def teleop_heartbeat_timer_callback(self):
        self.open_ = False
        self.get_logger().error("No connection to Teleop...")

def main(args=None):
    rclpy.init(args=args)
    
    node = GripperActionServer()

    if node.gripper.init_success:
        rclpy.spin(node)

    node.destroy_node()

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    