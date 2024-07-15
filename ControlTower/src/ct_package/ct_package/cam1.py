import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class Cam1Pulisher(Node):
    def __init__(self):
        super().__init__('cam1_publisher')
        self.publisher = self.create_publisher(Image, 'image_raw_cam1', 10) 
        time_period = 0.01
        self.timer = self.create_timer(time_period, self.time_callback) # 주기적으로 토픽을 발행하기 위함
        self.cap = cv2.VideoCapture(0)

    def time_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            fra = bridge.cv2_to_imgmsg(frame) #ros2 형식의 imgmsg로 형태변환 사용시에 반대로 변환해서 사용해야함
            self.publisher.publish(fra)
            # cv2.imshow('image_raw_cam1', frame)
            # cv2.waitKey(2)
        # self.get_logger().info('Publishing Cam1 Image')

def main(args = None):
    rclpy.init(args = args)
    node = Cam1Pulisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher Cam1 Stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == "__main__":
        main()
