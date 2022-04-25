import rclpy
from rclpy.node import Node
from sr2_interfaces.msg  import JointChange, MoveJoint, ArmControl
from sr2_arm.gripper_action_client import GripperActionClient

class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')

        # Subscriber to this Node
        self._sub = self.create_subscription(ArmControl, 'sr2/arm/control', self.control_callback, 10)

        # Publisher for kinova
        self._kinova_joint_pub = self.create_publisher(JointChange, 'sr2/arm/kinova/joint_change', 10)
        self._kinova_move_pub = self.create_publisher(MoveJoint, 'sr2/arm/kinova/joint_move', 10)

        # TODO: publishers for Dynamixal wrist

        # Client for Gripper
        self._gripper_client = GripperActionClient()


    def control_callback(self, msg: ArmControl):
        if msg.kinova_joint:
            joint_msg = JointChange()
            if msg.change == ArmControl.JOINT_UP:
                joint_msg.index_change = JointChange.UP
            elif msg.change == ArmControl.JOINT_DOWN:
                joint_msg.index_change = JointChange.DOWN
            self._kinova_joint_pub.publish(joint_msg)
        if msg.kinova_move:
            move_msg = MoveJoint()
            move_msg.speed = msg.direction * msg.speed
            self._kinova_move_pub(move_msg)
        if msg.gripper:
            self._gripper_client.send_goal(0.085 if msg.state else 0.0)
            rclpy.spin(self._gripper_client)


def main(args=None):
    rclpy.init(args=args)

    node = ArmController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
