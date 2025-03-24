import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/video', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)  # ~20 FPS
        self.cap = cv2.VideoCapture(3)  # change to your camera index if needed

        # Set resolution to 1280x720 (720p)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)     

        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
        else:
            self.get_logger().info('Video device opened successfully')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
            msg = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")  # Publish as mono8
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to read from camera.")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
