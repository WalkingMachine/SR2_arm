import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

#change this for the real interface
from action_tutorials_interfaces.action import Fibonacci

"""
This node have the goal to provide gripper motion to sara 2
"""
class Gripper(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = Gripper()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()