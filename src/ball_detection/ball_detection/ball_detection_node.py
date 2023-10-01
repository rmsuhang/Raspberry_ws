# 导入ROS 2的Python客户端库rclpy和Node类
import rclpy
from rclpy.node import Node
# 导入OpenCV库
import cv2
# 导入NumPy库，用于数组和数学操作
import numpy as np
# 从sensor_msgs包导入Image消息类型，用于图像传输
from sensor_msgs.msg import Image
# 从std_msgs包导入String消息类型
from std_msgs.msg import String  
# 导入cv_bridge库，用于在ROS和OpenCV之间转换图像
from cv_bridge import CvBridge

# 定义一个名为BallDetectionNode的类，继承自Node
class BallDetectionNode(Node):
    def __init__(self):
        # 使用名为'ball_detection_node'的节点名称初始化Node
        super().__init__('ball_detection_node')
        # 创建一个发布Image消息的发布者，用于发布检测后的图像
        self.image_publisher = self.create_publisher(Image, 'ball_detection_image', 10)     
        # 创建一个发布String消息的发布者，用于发布检测结果
        self.result_publisher = self.create_publisher(String, 'ball_detection_result', 10)  
        # 创建一个CvBridge对象，用于ROS和OpenCV图像之间的转换
        self.bridge = CvBridge()
        # 打开默认摄像头设备
        self.cap = cv2.VideoCapture(0)

    # 定义一个名为detect_ball的方法，用于从摄像头捕获图像并检测红色的球
    def detect_ball(self):
        # 只要ROS运行正常，就继续循环
        while rclpy.ok():
            # 从摄像头读取一帧图像
            ret, frame = self.cap.read()

            # 如果读取失败，打印错误消息并跳出循环
            if not ret:
                print("can not read image from camera correctly,please check your device")
                break

            # 将BGR图像转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 定义红色的HSV范围
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            # 创建一个掩码，其中红色区域的像素为1，其他像素为0
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # 对掩码进行腐蚀和膨胀操作，以减少噪声
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # 查找掩码中的轮廓
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 如果找到了轮廓（即检测到了红色的球）
            if len(contours) > 0:
                for contour in contours:
                    # 获取包围轮廓的最小圆
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    # 计算圆心和半径
                    center = (int(x), int(y))
                    radius = int(radius)
                    # 在原图像上画圆
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)

                # 将OpenCV图像转换为ROS的Image消息
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                # 发布ROS的Image消息
                self.image_publisher.publish(ros_image)
                # 创建并发布"yes"消息，表示检测到了球
                result_msg = String()
                result_msg.data = "yes"
                self.result_publisher.publish(result_msg)
            else:
                # 创建并发布"no"消息，表示没有检测到球
                result_msg = String()
                result_msg.data = "no"
                self.result_publisher.publish(result_msg)

# 主函数，启动ROS 2节点
def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionNode()
    node.detect_ball()
    rclpy.spin(node)
    rclpy.shutdown()

# 如果这个文件作为主程序运行，调用上面的main函数
if __name__ == '__main__':
    main()






