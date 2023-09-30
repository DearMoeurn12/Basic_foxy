import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import math

def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class RobotNavigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')

        self.pub = self.create_publisher(Twist,'/turtle1/cmd_vel', 10)

        self.sub = self.create_subscription(Point, '/turtle1/pose' , self.callback_odom, 10)


        # Initial position 
        self.position_ = Point()
        self.yaw_ = 0
        # machine state
        self.state_ = 0
        
        # goal
        self.desired_position_ = Point()
        self.desired_position_.x = 8.0
        self.desired_position_.y = 5.0
        # parameters
        self.yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
        self.dist_precision_ = 0.01
        self.idex_path = 0

        # Path points
        self.start = (0.0,0.0)
        self.goal = (10.0, 10.0)



                            
    def fix_yaw(self):
        desired_yaw = math.atan2(self.desired_position_.y - self.position_.y, self.desired_position_.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_

        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision_:
            twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        
        self.pub.publish(twist_msg)
        
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision_:
            #print('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def go_straight_ahead(self):

        desired_yaw = math.atan2(self.desired_position_.y - self.position_.y, self.desired_position_.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_
        err_pos = math.sqrt(pow(self.desired_position_.y - self.position_.y, 2) + pow(self.desired_position_.x - self.position_.x, 2))
        
        if err_pos > self.dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            self.pub.publish(twist_msg)
        else:
            print('Position error: [%s]' % err_pos)
            self.change_state(2)
        
        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision_:
            print( 'Yaw error: [%s]' % err_yaw)
            self.change_state(0)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub.publish(twist_msg)

    def change_state(self, state):
        self.state_ = state
        print('State changed to [%s]' % self.state_)


    def callback_odom(self, msg):
        # position
        self.position_ = msg
        print(msg)
        # self.start = [self.position_.x, self.position_.y]
        # # yaw
        # x = msg.pose.pose.orientation.x
        # y = msg.pose.pose.orientation.y
        # z = msg.pose.pose.orientation.z
        # w = msg.pose.pose.orientation.w
        # roll_x, pitch_y, yaw_z = euler_from_quaternion(x,y,z,w)
        # self.yaw_ = yaw_z

        # #print(self.Path)

        # self.obstacle()

        # if self.state_ == 0:
        #     self.fix_yaw()
        #     print('Adjust Yaw')
        # elif self.state_ == 1:
        #     self.go_straight_ahead()
        #     print('Go Straight')
        # elif self.state_ == 2:
        #     print('Reach Goal')
        #     self.done()
        # else:
        #     rclpy.get_logger().error('Unknown state!')
        #     pass


def main(args=None):
    rclpy.init(args=args)
    robot_navigation = RobotNavigation()
    rclpy.spin(robot_navigation)
    robot_navigation.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()



