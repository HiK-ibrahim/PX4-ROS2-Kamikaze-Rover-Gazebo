import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # Hedef bilgisi için kullanacağız
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Kamera abonesi
        self.subscription = self.create_subscription(Image, '/front_camera/image', self.image_callback, 10)
        
        # Hedef bilgisi yayıncısı
        self.target_pub = self.create_publisher(Point, '/vision/target_info', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Vision Node baslatildi. Hedef araniyor...")

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Kırmızı Maskesi
            l1 = np.array([0, 150, 50]); u1 = np.array([10, 255, 255])
            l2 = np.array([170, 150, 50]); u2 = np.array([180, 255, 255])
            mask = cv2.bitwise_or(cv2.inRange(hsv, l1, u1), cv2.inRange(hsv, l2, u2))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            target_msg = Point()
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 400:
                    cx = int(cv2.moments(c)["m10"] / cv2.moments(c)["m00"])
                    width = frame.shape[1]
                    
                    # Normalize x konumu (-1...1 arası)
                    target_msg.x = float((cx - (width / 2)) / (width / 2))
                    target_msg.y = float(area)
                    target_msg.z = 1.0 # Görünürlük TRUE
                    
                    cv2.circle(frame, (cx, 200), 15, (0, 255, 0), -1)
                else:
                    target_msg.z = 0.0 # Çok küçük, yok say
            else:
                target_msg.z = 0.0 # Hiç kontur yok
            
            self.target_pub.publish(target_msg)
            cv2.imshow("Vision Processing", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

def main():
    rclpy.init()
    rclpy.spin(VisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
