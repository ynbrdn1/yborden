import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import PoseStamped


class SaveStates(Node):

    def __init__(self):
        super().__init__('save_states')
        self.subscription = self.create_subscription(
            PoseStamped,
            'cf_1/pose',
            self.save_state_callback,
            10)
        self.subscription  # prevent unused variable warning

    def save_state_callback(self, msg):
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        with open('cf_states.csv', 'a') as f:
            f.write(f"{time.time()}, {position[0]}, {position[1]}, {position[2]}\n")


def main(args=None):
    rclpy.init(args=args)

    node = SaveStates()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()