import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetDigitalIO, SetAnalogIO, Call, SetInt16ById
from std_msgs.msg import Bool
from rclpy.qos import QoSPresetProfiles
import logging
import queue

class XArmControlNode(Node):
    def __init__(self):
        super().__init__('xarm_test2')
        qos_profile_system = QoSPresetProfiles.SYSTEM_DEFAULT.value
        self.capsule_queue = queue.Queue()

        #topic
        self.intruder_subscriber = self.create_subscription(Bool, 'intruder_signal', self.intruder_callback, qos_profile_system) #접근하는 사람 감지

        # 서비스 클라이언트 초기화
        self.set_servo_angle_service = self.create_client(MoveJoint, '/ufactory/set_servo_angle')
        self.set_position_service = self.create_client(MoveCartesian, '/ufactory/set_position')
        self.set_tgpio_digital_service = self.create_client(SetDigitalIO, '/ufactory/set_tgpio_digital')
        self.set_cgpio_analog_service = self.create_client(SetAnalogIO, '/ufactory/set_cgpio_analog')
        self.set_cgpio_digital_service = self.create_client(SetDigitalIO, '/ufactory/set_cgpio_digital')
        self.open_lite6_gripper_service = self.create_client(Call, '/ufactory/open_lite6_gripper')
        self.close_lite6_gripper_service = self.create_client(Call, '/ufactory/close_lite6_gripper')
        self.stop_lite6_gripper_service = self.create_client(Call, '/ufactory/stop_lite6_gripper')
        
        #서비스 만들기
        self.aruco_status_service = self.create_service(SetInt16ById, "aruco_status", self.aruco_callback)

        # 서비스가 활성화될 때까지 대기
        self.wait_for_service(self.set_servo_angle_service, 'angle set service')
        self.wait_for_service(self.set_position_service, 'position set service')
        self.wait_for_service(self.set_tgpio_digital_service, 'tgpio digital set service')
        self.wait_for_service(self.set_cgpio_digital_service, 'cgpio digital set service')
        self.wait_for_service(self.set_cgpio_analog_service, 'cgpio analog set service')
        self.wait_for_service(self.open_lite6_gripper_service, 'open lite6 gripper service')
        self.wait_for_service(self.close_lite6_gripper_service, 'close lite6 gripper service')
        self.wait_for_service(self.stop_lite6_gripper_service, 'stop lite6 gripper service')

        self.call_services()
    
    def aruco_callback(self, req, res): # aruco 
        if req.id == 0 :
            self.capsule_position = 1
        elif req.id == 1:
            self.capsule_position = 2
        elif req.id == 2:
            self.capsule_position = 3
        else:
            self.get_logger().info("aruco id's error")
        self.capsule_queue.put(self.capsule_position) # queue 형식으로 들어온 capsule 위치 추가 -> 순차적 처리 위함, 처리됐을때 큐에서 제거하는거 추가필요
        self.get_logger().info(f"Queue : {self.capsule_queue}")
        self.get_logger().info(f"Aruco Callback Request id: {req.id}, data: {req.data}")
    
    def intruder_callback(self, signal):
        if signal.data:  # Bool 메시지의 data 필드를 확인해야 합니다.
            self.get_logger().info("Intruder Detected !")
        else:
            pass
            

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')

    def async_service_call(self, client, request, service_name): #비동기 콜 필요시 동기로 수정
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'{service_name} succeeded with message: {future.result().message}')
            return 0  # 성공하면 0 반환
        else:
            self.get_logger().warning(f'Failed to send {service_name} command')
            return -1  # 실패하면 -1 반환

    def set_tgpio_digital(self, ionum, value): #this is for gripper motion
        service_name = "set_tgpio_digital"
        tgpio_request = SetDigitalIO.Request()
        tgpio_request.ionum = ionum
        tgpio_request.value = value
        # tgpio_request.delay_sec = delay_sec
        return self.async_service_call(self.set_tgpio_digital_service, tgpio_request, service_name)
    
    def set_cgpio_analog(self, ionum, value):
        service_name = "set_cgpio_analog"
        analog_request = SetAnalogIO.Request()
        analog_request.ionum = ionum
        analog_request.value = value
        self.async_service_call(self.set_cgpio_analog_service, analog_request, service_name)

    def set_cgpio_digital(self, ionum, value):
        service_name = "set_cgpio_digital"
        analog_request = SetAnalogIO.Request()
        analog_request.ionum = ionum
        analog_request.value = value
        self.async_service_call(self.set_cgpio_digital_service, analog_request, service_name)

    def control_gripper(self, gripper_action): #action name
        if gripper_action == 'open':
            ret1 = self.set_tgpio_digital(0, 1)
            ret2 = self.set_tgpio_digital(1, 0)
        elif gripper_action == 'close':
            ret1 = self.set_tgpio_digital(0, 0)
            ret2 = self.set_tgpio_digital(1, 1)
        elif gripper_action == 'stop':
            ret1 = self.set_tgpio_digital(0, 0)
            ret2 = self.set_tgpio_digital(1, 0)
        else:
            self.get_logger().warning(f'Invalid gripper action: {gripper_action}')
            return

        self.get_logger().debug(f'Gripper action: {gripper_action}, ret1: {ret1}, ret2: {ret2}')

        if ret1 != 0 or ret2 != 0:
            self.get_logger().warning(f'Failed to perform gripper action: {gripper_action}')
        else:
            self.get_logger().info(f'Gripper action: {gripper_action} succeeded')
    
    def set_servo_angle(self, angles, speed, acc, mvtime, wait, radius):
        service_name = "set_servo_angle"
        angle_request = MoveJoint.Request()
        angle_request.angles = angles
        angle_request.speed = speed
        angle_request.acc = acc
        angle_request.mvtime = mvtime
        angle_request.wait = wait
        angle_request.radius = radius
        self.async_service_call(self.set_servo_angle_service, angle_request, service_name)

    def set_position(self, pose, speed, acc, mvtime, relative=False, wait=False, radius=-1,): 
        service_name = "set_position"
        position_request = MoveCartesian.Request()
        position_request.pose = pose
        position_request.speed = speed
        position_request.acc = acc
        position_request.mvtime = mvtime
        position_request.relative = relative
        position_request.wait = wait
        position_request.radius = radius
        self.async_service_call(self.set_position_service, position_request, service_name)
        
    def call_services(self):
        self.control_gripper('close')
        # self.control_gripper('stop', 10.0)
        # self.control_gripper('open')
        # self.control_gripper('stop')


def main(args=None):
    # 로깅 설정
    logging.basicConfig(level=logging.INFO, force=True)
    
    rclpy.init(args=args)
    node = XArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
