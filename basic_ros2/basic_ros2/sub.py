import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,  '/Bos_data_tv_hz_vey', self.chatter_callback, 10)

    def chatter_callback(self, topic):
        print(topic)
        print(topic.data)
        # self.get_logger().info(f"Received: ")

def main(args=None):
    rclpy.init(args=args)
    minimalSub = MinimalSubscriber()
    rclpy.spin(minimalSub)
    minimalSub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
