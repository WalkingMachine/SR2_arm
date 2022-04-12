import rclpy
from rclpy.node import Node
from kinova_driver import KinovaDriver
from std_msgs.msg import Bool, Twist
from threading import Lock

class Kinova_Control(Node):

    def __init__(self):
        super().__init__('kinova_control')

        # subscriber to change selected joint
        self.joint_sub = self.create_subscription(Bool, "sara/arm/kinova/joint-change", self.joint_change_callback, 10)

        # subscriber to move a joint
        self.move_sub = self.create_subscription(Twist, "sara/arm/kinova/joint-move", self.joint_move_callback, 10)

        self.driver = KinovaDriver()

        self.driver.initialize()

        self.mutex_lock = Lock()


    def joint_change_callback(self, msg):
        with self.mutex_lock:
            if msg.data:
                self.driver.joint_up()
                self.get_logger().info(f"Joint changed (+), joint is now {self.driver.joint_index}")
            else:
                self.driver.joint_down()
                self.get_logger().info(f"Joint changed (-), joint is now {self.driver.joint_index}")


    def joint_move_callback(self, msg):
        with self.mutex_lock:
            self.driver.send_cmd_val(msg.linear.x)


    def close(self):
        self.driver.shutdown()