import cv2
import cv2.aruco as aruco
import numpy as np
import mediapipe as mp
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from hi5_message.srv import SetInt16ById, Status
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSPresetProfiles


class Cam2ImageProcessor(Node):
    def __init__(self):
        super().__init__('cam2_image_processor')
        self.init()

        qos_profile_sensor = QoSPresetProfiles.SENSOR_DATA.value
        qos_profile_system = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.img_subscriber = self.create_subscription(Image, "image_raw_cam2", self.change_image, qos_profile_sensor)
        self.intruder_publisher = self.create_publisher(Bool, 'intruder_signal', qos_profile_system)

        self.aruco_status_service = self.create_client(SetInt16ById, "aruco_status")
        self.star_status_service = self.create_client(SetBool, "star_status")
        self.robot_status_yolo_service = self.create_client(SetBool, "robot_status_yolo")
        self.vision_status_service = self.create_client(Status, "vision_status")
        
        self.robot_status_service = self.create_service(Status, "robot_status_v2", self.status_callback)

        self.timer_callback = self.create_timer(0.5, self.intruder_callback)

        self.wait_for_service(self.aruco_status_service, 'aruco status service')
        self.wait_for_service(self.star_status_service, 'star status service')
        self.wait_for_service(self.robot_status_yolo_service, 'robot status yolo service')

    def init(self):
        self.bridge = CvBridge()
        self.frame = None
        self.intruder_detect = False
        self.last_seen = {}

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)

        self.roi_x_large = 52
        self.roi_y_large = 0
        self.roi_width_large = 500
        self.roi_height_large = 310
        self.font_scale = 1
        self.font_thickness = 2
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.robot_status = None
        self.vision_status = 0
        
    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f"{service_name} is ready")

    def async_service_call(self, client, request, service_name): # 비동기 콜
        future = client.call_async(request)
        future.add_done_callback(lambda future: self.handle_service_response(future, service_name))
        return future

    def handle_service_response(self, future, service_name):
        try:
            response = future.result()
            self.get_logger().info(f'{service_name} succeeded with message: {response.message}')
        except Exception as e:
            self.get_logger().warning(f'Failed to send {service_name} command: {e}')
    
    def status_callback(self, req, res):
        self.get_logger().info("robot status vision2 callback")
        if req.cmd == "RS":
            self.get_logger().info(f"cmd: {req.cmd}, data: {req.status}")
            self.robot_status = req.status
            request = SetBool.Request()
            if self.robot_status == 1:
                self.get_logger().info("로봇 제조중 로봇 제조중 로봇 제조중 로봇 제조중 로봇 제조중 로봇 제조중")
                request.data = True
                self.vision_status = 1
            else:
                self.get_logger().info("로봇대기중 로봇대기중 로봇대기중 로봇대기중 로봇대기중 로봇대기중 로봇대기중")
                request.data = False
                self.vision_status = 0
            self.robot_status_yolo_service.call_async(request)
            res.success = True
            res.message = "robot status2 callback"
        else:
            self.get_logger().info("no data in status2 callback")
            res.success = False
            res.message = "robot status2 callback fail"
        return res

    def intruder_callback(self):
        # This function should now be called at the specified interval
        # print("Intruder callback triggered")
        msg = Bool()
        msg.data = self.intruder_detect
        self.intruder_publisher.publish(msg)
        # self.get_logger().info(f'Intruder detection signal published: {self.intruder_detect}')

    def change_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
        except CvBridgeError as e:
            self.get_logger().error(f"Conversion failed: {e}")

    def vision_status_call(self):
        if self.vision_status == 3:
            request = Status.Request()
            request.cmd = "VS"
            request.status = 4
            return self.async_service_call(self.vision_status_service, request, "vision status to vision2")
            

    def process_frame(self, frame):
        frame = self.detect_aruco_markers(frame)
        frame = self.detect_star_shape(frame)
        frame = self.detect_intrusion(frame)
        self.vision_status_call()
        return self.frame
    
    def set_star_status(self):
        self.get_logger().info("set star status")
        service_name = "star_status"
        star_request = SetBool.Request()
        star_request.data = self.star_status
        return self.async_service_call(self.star_status_service, star_request, service_name)
    
    def set_aruco_status(self, id, data): #this is for gripper motion
        self.get_logger().info(f"1111111111111set arcuo status id: {id}, data: {data}")
        service_name = "aruco_status"
        aruco_request = SetInt16ById.Request()
        aruco_request.id = int(id)
        aruco_request.data = int(data)
        return self.async_service_call(self.aruco_status_service, aruco_request, service_name)

    def detect_aruco_markers(self, frame):
        try:
            # self.get_logger().info("skskskskskskskssk")
            if self.vision_status == 1:
                self.get_logger().info("detect_aruco_markers: Vision status is 1, detecting markers")
                roi_medium = frame[0:60, 270:540]
                corners, ids, _ = aruco.detectMarkers(roi_medium, self.aruco_dict, parameters=self.parameters)
                current_time = time.time()
                
                if ids is not None and len(ids) > 0:
                    aruco.drawDetectedMarkers(roi_medium, corners, ids)
                    
                    # Update the last seen time for detected markers
                    for id in ids.flatten():
                        self.last_seen[id] = current_time
                    
                    # Check if any markers have not been seen for more than 5 seconds
                    for id, last_time in self.last_seen.items():
                        # self.get_logger().info(f"id: {id}, current-time - last-time:{current_time-last_time}")
                        if current_time - last_time > 3:
                            self.get_logger().info(f"Action executed for marker ID {id} (not moved for 5 seconds)")
                            self.set_aruco_status(id, 1)
                            self.vision_status = 2
                            del self.last_seen[id]
                            break
                else:
                    # No markers detected, so clear the last seen dictionary
                    # self.get_logger().info("NO Markers detected")
                    pass
                
            return frame
        except Exception as e:
            self.get_logger().error(f"Error in detect_aruco_markers: {e}")
            return frame  # Return the frame even if there's an error

    def detect_star_shape(self, frame, canny_thresh1=50, canny_thresh2=112):
        if self.vision_status == 2:
            small_roi = frame[118:153, 464:499]
            gray = cv2.cvtColor(small_roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edged = cv2.Canny(blurred, canny_thresh1, canny_thresh2)
            contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == (1 or 2):
                contour = contours[0]
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) <= 10:
                    star_contour = approx + [464, 118]
                    cv2.drawContours(frame, [star_contour], -1, (0, 255, 0), 3)
                    cv2.putText(frame, "Star detected", (10, 30), self.font, 1, (0, 255, 255), 2)
                    self.star_status = True
                    self.set_star_status()
                    self.vision_status = 3
                else:
                    cv2.putText(frame, "Star not detected", (10, 30), self.font, 1, (0, 255, 255), 2)
                    self.star_status = False
            else:
                cv2.putText(frame, "Star not detected", (10, 30), self.font, 1, (0, 255, 255), 2)
            frame[118:153, 464:499] = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
        else:
            pass
        return frame
    
            
    def detect_intrusion(self, frame): # always
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
                        # print("Intrusion detected!")
                        self.intruder_detect = True
                        text = "Warning: Intrusion detected!"
                        text_size, _ = cv2.getTextSize(text, self.font, self.font_scale, self.font_thickness)
                        text_width, text_height = text_size
                        text_x = (frame.shape[1] - text_width) // 2
                        text_y = (frame.shape[0] + text_height) // 2
                        cv2.putText(frame, text, (text_x, text_y), self.font, self.font_scale, (255, 255, 0), self.font_thickness)
                        break
                    else:
                        self.intruder_detect = False
                mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        else:
            self.intruder_detect = False
        cv2.rectangle(frame, (self.roi_x_large, self.roi_y_large), (self.roi_x_large + self.roi_width_large, self.roi_y_large + self.roi_height_large), (255, 0, 0), 2)
        return frame


def main(args=None):
    rclpy.init(args=args)
    image_processor = Cam2ImageProcessor()
    cv2.namedWindow('Frame')
    cv2.createTrackbar('Threshold', 'Frame', 0, 20000, lambda x: None)
    cv2.createTrackbar('Canny Thresh1', 'Frame', 50, 255, lambda x: None)
    cv2.createTrackbar('Canny Thresh2', 'Frame', 112, 255, lambda x: None)
    cv2.createTrackbar('Brightness', 'Frame', 50, 100, lambda x: None)

    while rclpy.ok():
        rclpy.spin_once(image_processor)
        frame = image_processor.frame
        if frame is not None:
            frame = image_processor.process_frame(frame)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
