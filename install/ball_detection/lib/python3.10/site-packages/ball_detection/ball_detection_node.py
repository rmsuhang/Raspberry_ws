import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String  # 导入String消息类型
from cv_bridge import CvBridge

class BallDetectionNode(Node):
    def __init__(self):
        super().__init__('ball_detection_node')
        self.image_publisher = self.create_publisher(Image, 'ball_detection_image', 10)     # 创建图像发布者
        self.result_publisher = self.create_publisher(String, 'ball_detection_result', 10)  # 创建发布 "yes" 或 "no" 的发布者
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0)

    def detect_ball(self):

        while rclpy.ok():
            ret, frame = self.cap.read()
            

            if not ret:
                print("can not read image from camera correctly,please chaeck your device")
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                for contour in contours:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)

                # Convert the OpenCV image to a ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                # Publish the ROS Image message
                self.image_publisher.publish(ros_image)

                # Publish "yes" message
                result_msg = String()
                result_msg.data = "yes"
                self.result_publisher.publish(result_msg)
            else:
                # Publish "no" message
                result_msg = String()
                result_msg.data = "no"
                self.result_publisher.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionNode()
    node.detect_ball()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
