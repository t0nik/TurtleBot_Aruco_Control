import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32 # ArUco Zone for subscriber
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class ControlTurtleBot(Node):

    def __init__(self):
        super().__init__('control_turtlebot')
        # Publisher
        queue_size = 10
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', queue_size)
        timer_period = 0.5  # seconds, time of callbacks
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Subscriber
        self.subscription = self.create_subscription(Int32,'aruco_zone',self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.zone = 0.0

    def timer_callback(self):
        set_control = Twist()
        # Floats
        set_control.linear.x = self.zone
        set_control.linear.y = 0.0
        set_control.linear.z = 0.0
        set_control.angular.x = 0.0
        set_control.angular.y = 0.0
        set_control.angular.z = 0.0
        self.publisher_.publish(set_control)
        self.get_logger().info('control_turtlebot: publishing x-linear: "%d" to \cmd_vel' % set_control.linear.x)
        self.i += 1 # Whatever
    
    def listener_callback(self, recieved_zone):
        self.zone =  float(recieved_zone.data)


def main(args=None):
    rclpy.init(args=args)
    control_turtlebot = ControlTurtleBot()
    rclpy.spin(control_turtlebot)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_turtlebot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
