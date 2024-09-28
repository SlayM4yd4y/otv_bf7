import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')
        self.subscription = self.create_subscription(
            String,
            'key_press',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        twist = Twist()
        if msg.data == 'w':
            twist.linear.x = 2.0
        elif msg.data == 's':
            twist.linear.x = -2.0
        elif msg.data == 'a':
            twist.angular.z = 2.0
        elif msg.data == 'd':
            twist.angular.z = -2.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    draw_node = DrawNode()
    rclpy.spin(draw_node)
    draw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()