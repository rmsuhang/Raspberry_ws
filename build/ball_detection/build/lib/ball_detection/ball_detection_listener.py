import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BallDetectionListener(Node):
    def __init__(self):
        super().__init__('ball_detection_listener')
        self.subscription = self.create_subscription(
            String,
            'ball_detection_result',
            self.result_callback,
            10
        )
        self.subscription  # 防止subscription被垃圾回收

    def result_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
