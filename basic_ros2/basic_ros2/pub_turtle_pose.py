import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.bosData = self.create_publisher(Pose, '/turtle1/pose', 10)
        timer_period = 0.5  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 100

    def timer_callback(self):
        #***** Publish**********
        msg = Pose()

        msg.position.x = 0.2
        msg.position.y = 0.8
        msg.position.z = 0.

        msg.orientation.x = 0.
        msg.orientation.y = 0.
        msg.orientation.z = 0.
        msg.orientation.w = 0.

        self.bosData.publish(msg)

        # #***** Publish**********

        # self.get_logger().info('Publishing:')


    

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


