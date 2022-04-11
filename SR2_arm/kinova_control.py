import rclpy
from rclpy.node import Node
from kinova_driver import KinovaDriver
from std_msgs.msg import Bool


class Kinova_Control(Node):

    def __init__(self):
        super().__init__('kinova_control')

        self.joint_sub = self.create_subscription(Bool, "sara/arm/kinova/joint-change", self.joint_change_callback, 10)

        self.driver = KinovaDriver()

        self.driver.initialize()


    def joint_change_callback(self, msg):
        if msg.data:
            self.driver.joint_up()
            self.get_logger().info(f"Joint changed (+), joint is now {self.driver.joint_index}")
        else:
            self.driver.joint_down()
            self.get_logger().info(f"Joint changed (-), joint is now {self.driver.joint_index}")


    def close(self):
        self.driver.shutdown()