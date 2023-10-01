# 导入ROS 2的Python客户端库rclpy
import rclpy
# 导入Node（节点）类，它是创建ROS 2节点的基础
from rclpy.node import Node
# 从std_msgs包中导入String消息类型
from std_msgs.msg import String

# 定义一个名为BallDetectionListener的类，它继承自Node
class BallDetectionListener(Node):
    def __init__(self):
        # 使用名为'ball_detection_listener'的节点名称初始化Node
        super().__init__('ball_detection_listener')
        
        # 创建一个订阅者，用于订阅'ball_detection_result'主题
        # 该主题应该发布String消息类型
        # 当收到消息时，会调用self.result_callback函数
        # 10是队列大小，意味着该订阅者可以缓存10条消息
        self.subscription = self.create_subscription(
            String,
            'ball_detection_result',
            self.result_callback,
            10
        )
        # 保持对subscription的引用，防止它在其作用域外被Python的垃圾收集器回收
        self.subscription  

    # 当从'ball_detection_result'主题收到消息时调用此回调函数
    def result_callback(self, msg):
        # 使用节点的日志记录器打印收到的消息内容
        self.get_logger().info(f'Received: {msg.data}')

# 主函数，启动ROS 2节点
def main(args=None):
    # 初始化ROS 2的Python客户端库
    rclpy.init(args=args)
    # 创建BallDetectionListener的实例
    node = BallDetectionListener()
    # 使节点保持运行，直到它被显式关闭或接收到关闭信号
    rclpy.spin(node)
    # 关闭ROS 2的Python客户端库
    rclpy.shutdown()

# 如果这个文件作为主程序运行，调用上面的main函数
if __name__ == '__main__':
    main()