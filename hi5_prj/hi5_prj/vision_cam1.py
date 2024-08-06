import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from hi5_message.srv import Status
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSPresetProfiles

class Cam1ImageProcessor(Node):
    def __init__(self):
        super().__init__('cam1_image_processor')
        self.init()

        qos_profile_sensor = QoSPresetProfiles.SENSOR_DATA.value
        qos_profile_system = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.img_subscriber = self.create_subscription(Image, "image_raw_cam1", self.change_image, qos_profile_sensor)
        #vision status 받을 서비스 서버
        self.vision_status_service = self.create_service(Status, "vision_status", self.vision_status_callback)

        #hand area 보내줄 클라이언트
        self.donduruma_publisher = self.create_publisher(Bool, "donduruma_pub", 10)
        time_period = 0.01
        self.timer = self.create_timer(time_period, self.time_callback)
        self.bridge = CvBridge()

    def init(self):
        self.detector = HandDetector(maxHands=5, detectionCon=0.8)  # 최대 5개의 손을 인식
        # 임계값 설정 (이 값을 조정하여 손이 너무 가까워졌을 때를 결정)
        self.AREA_THRESHOLD = 100000  # 예시 값, 실제 상황에 맞게 조정 필요
        # 손 ID 저장
        self.tracked_hand_bbox = None
        self.robot_status = None
        self.detected = False
        self.vision_status = None

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f"{service_name} is ready")

    def vision_status_callback(self, req, res):
        self.get_logger().info("vision status callback")
        if req.cmd:
            self.get_logger().info(f"cmd: {req.cmd}, data: {req.status}")
            self.vision_status = 4
            res.success = True
            res.message = "vision status callback"
        else:
            self.get_logger().info("no data in viison status callback")
            res.success = False
            res.message = "vision status callback fail"
        return res
    

    def change_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
            self.donduruma()
            cv2.imshow('Processed Image', self.frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"Conversion failed: {e}")

    def time_callback(self):
        if self.vision_status == 4: 
            
            status_request = Bool()
            if self.detected:
                status_request.data = self.detected
                self.donduruma_publisher.publish(status_request)
            else:
                pass
        else:
            pass

    def donduruma(self):
        try:
            hands, frame = self.detector.findHands(self.frame)
            self.detected = False
            if hands:
            # 인식된 손들을 면적 기준으로 정렬
                hands.sort(key=lambda x: x['bbox'][2] * x['bbox'][3], reverse=True)
                print('hand:', hands)

                # 손마다 바운딩 박스를 표시
                for hand in hands:
                    x, y, w, h = hand['bbox']
                    cvzone.putTextRect(frame, f'Area: {w * h}', (x, y - 10), scale=1, thickness=2, colorR=(255, 0, 0))
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                if self.tracked_hand_bbox is None:
                    # 면적이 가장 큰 손의 바운딩 박스를 저장
                    self.tracked_hand_bbox = hands[0]['bbox']
                
                # 추적 중인 손을 찾기
                hand_to_track = None
                for hand in hands:
                    if hand['bbox'] == self.tracked_hand_bbox:
                        hand_to_track = hand
                        break
                
                if hand_to_track:
                    # Get the bounding box of the tracked hand
                    x, y, w, h = hand_to_track['bbox']
                    # Calculate the area of the bounding box
                    area = w * h

                    # 면적이 임계값을 초과하면 경고 메시지 표시
                    if area > self.AREA_THRESHOLD:
                        cvzone.putTextRect(frame, "Oops! Too close, trying to steal the ice cream?", (50, 50), scale=1, thickness=2, colorR=(0, 0, 255))
                        self.detected = True
                    # Display the area on the image
                    cvzone.putTextRect(frame, f'Tracked Area: {int(area)}', (50, 100), scale=2, thickness=2, colorR=(255, 0, 0))
                    
                    # 추적 중인 손을 강조 표시
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 4)  # 빨간색 두꺼운 사각형
                    cvzone.putTextRect(frame, 'Tracking', (x, y - 50), scale=2, thickness=2, colorR=(0, 0, 255))
                else:
                    # 추적 중인 손을 찾을 수 없는 경우, 초기화
                    self.tracked_hand_bbox = None
                    self.detected = False
            else:
                # 손이 없으면 ID 초기화
                self.tracked_hand_bbox = None
                self.detected = False
        except Exception as e:
            self.get_logger().info(f"{e}")
            self.detected = False


def main(args=None):
    rclpy.init(args=args)
    image_processer = Cam1ImageProcessor()

    while rclpy.ok():
        rclpy.spin_once(image_processer)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    image_processer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
