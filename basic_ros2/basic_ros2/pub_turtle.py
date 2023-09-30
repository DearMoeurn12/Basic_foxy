import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.bosData = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 100

    def timer_callback(self):
        #***** Publish**********
        msg = Twist()
        # msg.linear.x = 0.2
        msg.linear.y = 0.0
        # msg.linear.z = 0.
        # msg.angular.x = 0.
        # msg.angular.y = 0.
        msg.angular.z = 0.9

        self.bosData.publish(msg)

        # #***** Publish**********

        # self.get_logger().info('Publishing:')

        # self.i += 100

    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


