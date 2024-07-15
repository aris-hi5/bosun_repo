import cv2
import cv2.aruco as aruco
import numpy as np
import mediapipe as mp
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# class Cam2Subscriber(Node):
#     def __init__(self):
#         super().__init__('cam2_subscriber')
#         self.img_subscriber = self.create_subscription(Image, "image_raw_cam2", self.change_image, 10)
#         self.bridge = CvBridge()
#         self.frame = None

#     def change_image(self, msg):
#         # self.get_logger().info(f"Incoming image encoding: {msg.encoding}")
#         try:
#             self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
#         except CvBridgeError as e:
#             self.get_logger().error(f"Conversion failed: {e}")

#     def get_frame(self):
#         return self.frame

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.img_subscriber = self.create_subscription(Image, "image_raw_cam2", self.change_image, 10)
        self.bridge = CvBridge()
        self.frame = None
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.last_seen = {}

    def change_image(self, msg):
        # self.get_logger().info(f"Incoming image encoding: {msg.encoding}")
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
        except CvBridgeError as e:
            self.get_logger().error(f"Conversion failed: {e}")
    def get_frame(self):
        return self.frame

    def detect(self, frame):
        roi_medium = frame[0:60, 270:540]
        corners, ids, _ = aruco.detectMarkers(roi_medium, self.aruco_dict, parameters=self.parameters)
        current_time = time.time()
        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(roi_medium, corners, ids)
            for id in ids.flatten():
                self.last_seen[id] = current_time
            for id, last_time in list(self.last_seen.items()):
                if current_time - last_time > 5:
                    print(f"Action executed for marker ID {id} (not moved for 5 seconds)")
                    del self.last_seen[id]
                    break
        return frame

class StarShapeDetector(Node):
    def __init__(self):
        super().__init__('star_shape_detector')
        self.img_subscriber = self.create_subscription(Image, "image_raw_cam2", self.change_image, 10)
        self.bridge = CvBridge()
        self.frame = None

    def change_image(self, msg):
        # self.get_logger().info(f"Incoming image encoding: {msg.encoding}")
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
        except CvBridgeError as e:
            self.get_logger().error(f"Conversion failed: {e}")

    def detect_star_shape(self, image, canny_thresh1=50, canny_thresh2=112):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, canny_thresh1, canny_thresh2)
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 1:
            contour = contours[0]
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) >= 10:
                return True, approx, edged, len(contours)
        return False, None, edged, len(contours)

    def detect(self, frame):
        small_roi = frame[118:153, 464:499]
        star_detected, star_contour, edged, contour_count = self.detect_star_shape(small_roi)
        if star_detected:
            star_contour += [464, 118]
            cv2.drawContours(frame, [star_contour], -1, (0, 255, 0), 3)
            cv2.putText(frame, "Star detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "Star not detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        frame[118:153, 464:499] = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
        return frame

class IntrusionDetector(Node):
    def __init__(self):
        super().__init__('intrusion_detector')
        self.img_subscriber = self.create_subscription(Image, "image_raw_cam2", self.change_image, 10)
        self.bridge = CvBridge()
        self.frame = None
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
        self.roi_x_large = 52
        self.roi_y_large = 0
        self.roi_width_large = 500
        self.roi_height_large = 310

    def change_image(self, msg):
        # self.get_logger().info(f"Incoming image encoding: {msg.encoding}")
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
        except CvBridgeError as e:
            self.get_logger().error(f"Conversion failed: {e}")

    def detect(self, frame):
        threshold = 20000
        fgmask = self.fgbg.apply(frame)
        roi_large = fgmask[self.roi_y_large:self.roi_y_large + self.roi_height_large, self.roi_x_large:self.roi_x_large + self.roi_width_large]
        intrusion_detected = np.sum(roi_large) > threshold
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                for landmark in hand_landmarks.landmark:
                    x = int(landmark.x * frame.shape[1])
                    y = int(landmark.y * frame.shape[0])
                    if self.roi_x_large < x < self.roi_x_large + self.roi_width_large and self.roi_y_large < y < self.roi_y_large + self.roi_height_large and intrusion_detected:
                        print("Intrusion detected!")
                        text = "Warning: Intrusion detected!"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 1
                        font_thickness = 2
                        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
                        text_width, text_height = text_size
                        text_x = (frame.shape[1] - text_width) // 2
                        text_y = (frame.shape[0] + text_height) // 2
                        cv2.putText(frame, text, (text_x, text_y), font, font_scale, (255, 255, 0), font_thickness)
                        break
                mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        cv2.rectangle(frame, (self.roi_x_large, self.roi_y_large), (self.roi_x_large + self.roi_width_large, self.roi_y_large + self.roi_height_large), (255, 0, 0), 2)
        return frame

def main(args=None):
    rclpy.init(args=args)
    
    # cam2_subscriber = Cam2Subscriber()
    aruco_detector = ArucoMarkerDetector()
    star_detector = StarShapeDetector()
    intrusion_detector = IntrusionDetector()

    nodes = [aruco_detector, star_detector, intrusion_detector]

    for node in nodes:
        rclpy.spin_once(node, timeout_sec=0.1)

    cv2.namedWindow('Frame')
    cv2.createTrackbar('Threshold', 'Frame', 0, 20000, lambda x: None)
    cv2.createTrackbar('Canny Thresh1', 'Frame', 50, 255, lambda x: None)
    cv2.createTrackbar('Canny Thresh2', 'Frame', 112, 255, lambda x: None)
    cv2.createTrackbar('Brightness', 'Frame', 50, 100, lambda x: None)

    while rclpy.ok():
        rclpy.spin_once(aruco_detector)
        frame = aruco_detector.get_frame()
        if frame is not None:
            frame = aruco_detector.detect(frame)
            frame = star_detector.detect(frame)
            frame = intrusion_detector.detect(frame)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
