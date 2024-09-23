import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(String, 'key_press', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = None
        if keyboard.is_pressed('w'):
            key = 'w'
        elif keyboard.is_pressed('a'):
            key = 'a'
        elif keyboard.is_pressed('s'):
            key = 's'
        elif keyboard.is_pressed('d'):
            key = 'd'
        elif keyboard.is_pressed('q'):
            key = 'q'

        if key:
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
