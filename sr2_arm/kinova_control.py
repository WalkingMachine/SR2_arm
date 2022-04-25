import rclpy
from rclpy.node import Node
from .kinova.kinova_driver import KinovaDriver
from sr2_interfaces.msg import JointChange, Heartbeat

class Kinova_Control(Node):

    def __init__(self):
        super().__init__('kinova_control')

        # subscriber to change selected joint
        self.joint_sub = self.create_subscription(JointChange, "sara/arm/kinova/joint_change", self.joint_change_callback, 10)

        # subscriber to move a joint
        # self.move_sub = self.create_subscription(Twist, "sara/arm/kinova/joint-move", self.joint_move_callback, 10)

        self.driver = KinovaDriver()

        self.driver.initialize()

        # heartbeat check from teleop
        self.heartbeat_sub = self.create_subscription(Heartbeat, "sara/teleop/heartbeat", self.teleop_heartbeat_callback, 10)
        self.heartbeat_timer = self.create_timer(1, self.teleop_heartbeat_timer_callback)

        # Flow control
        self.open_ = True

    def teleop_heartbeat_callback(self, msg):
        self.heartbeat_timer.reset()
        self.open_ = True

    def teleop_heartbeat_timer_callback(self):
        self.open_ = False
        self.driver.send_estop()
        self.get_logger().error("No connection to Teleop...")

    def joint_change_callback(self, msg):
        if self.open_:
            if msg.SET:
                if 0 <= msg.set_index <= 4:
                    self.driver.set_joint(msg.set_index)
                else:
                    self.get_logger().warning("Joint index out of range, not setting")
            elif msg.index_change == JointChange.UP:
                self.driver.joint_up()
                self.get_logger().info(f"Joint changed (+), joint is now {self.driver.joint_index}")
            elif msg.index_change == JointChange.DOWN:
                self.driver.joint_down()
                self.get_logger().info(f"Joint changed (-), joint is now {self.driver.joint_index}")


    def joint_move_callback(self, msg):
        if self.open_:
            self.driver.send_cmd_val(msg.linear.x)
        else:
            self.driver.send_estop()


    def close(self):
        self.driver.shutdown()


def main(args=None):
    rclpy.init()

    node = Kinova_Control()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

