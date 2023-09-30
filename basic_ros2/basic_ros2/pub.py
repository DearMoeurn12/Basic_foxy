import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.bosData = self.create_publisher(String, '/Bos_data_tv_hz_vey', 10)
        timer_period = 0.5  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        
        self.sum()

        msg = String()
        msg.data = 'Hello ROS2: %d' % self.i
        self.bosData.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)

        # timer = self.i +  1
        # self.i = timer
        

    def sum(self):
        timer = self.i +  1
        self.i = timer
    


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()