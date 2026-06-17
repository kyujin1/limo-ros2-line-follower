import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('object_detect')
        
        # 퍼블리셔 및 서브스크라이버
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.obstacle_detected = False
        self.cmd = Twist()

    def scan_callback(self, msg):
        # 전방 60도 범위(각도 기준) 내 장애물 확인 (임계값 0.5m)
        ranges = np.array(msg.ranges)
        # 라이다 데이터 인덱스 계산 (보통 정면이 인덱스 0 근처)
        front_ranges = np.concatenate((ranges[-30:], ranges[:30]))
        self.obstacle_detected = np.min(front_ranges) < 0.5

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 1. ROI 설정 및 이진화 (검정색 추출)
        h, w = cv_image.shape[:2]
        roi = cv_image[int(h*0.7):h, 0:w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # 2. 무게중심 계산
        M = cv2.moments(binary)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            error = (cx - w / 2) / (w / 2)
        else:
            error = 0.0
            
        # 3. 우선순위 제어 로직
        if self.obstacle_detected:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5 # 정지 후 회전
        else:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = -1.0 * error # 라인 추종
            
        self.cmd_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
