import rclpy
from rclpy.node import Node
from bboxes_ex_msgs.msg import BoundingBox
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.bosData = self.create_publisher(BoundingBox, '/Bos_data_tv_hz_vey', 10)
        timer_period = 0.5  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 100

    def timer_callback(self):

        #***** Publish**********

        msg = BoundingBox()
        msg.probability = self.i*10/100
        msg.xmin = self.i - 1
        msg.ymin = self.i - 1
        msg.xmax = self.i + 1
        msg.ymax = self.i + 1

        self.bosData.publish(msg)

        #***** Publish**********

        self.get_logger().info('Publishing: "%s"' % msg)

        self.i += 100

    

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


