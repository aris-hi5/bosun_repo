# S/N : XYZARIS0V3P2402N01
# Robot IP : 192.168.1.185
# code_version : 3.1.5.2


#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
#
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from rclpy.qos import QoSPresetProfiles

from hi5_message.srv import OrderCall, Status, SetInt16ById

import sys
import os
import math
import time
import queue
import datetime
import random
import traceback
from threading import Thread, Event
import json
import numpy as np
import threading

from xarm import version
from xarm.wrapper import XArmAPI
from rclpy.executors import MultiThreadedExecutor

import pygame

class SubscriptionServer(Node):
    def __init__(self, server):
        super().__init__("sub_server")
        qos_profile_system = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.intruder_subscriber = self.create_subscription(Bool, 'intruder_signal', self.intruder_callback, qos_profile_system) #접근하는 사람 감지
        self.donduruma_subscriber = self.create_subscription(Bool, 'donduruma_pub', self.dondurma_callback, qos_profile_system)
        self.server = server
        self.temp_signal = False  # 초기화
        self.temp1_signal = False

    def intruder_callback(self, signal):
        # self.get_logger().info(f"intruder callback data:{signal.data}")

        if self.temp1_signal != signal.data:
            if signal.data == True:  # Bool 메시지의 data 필드를 확인해야 합니다.
                self.get_logger().info("intruder detected")
                self.server._arm.set_state(3)
                # self.set_state_service.call_async(state_request)
                # self.async_service_call(self.set_state_service, state_request, "set_state_service")
            else:
                self.get_logger().info("intruder 해제")
                self.server._arm.set_state(0)
                # self.async_service_call(self.set_state_service, state_request, "set_state_service")
            self.temp1_signal = signal.data
        else:
            pass

    def dondurma_callback(self, signal):
        if self.temp_signal != signal.data:
            self.get_logger().info(f"dondurma callback data:{signal.data}")
            if signal.data:
                self.get_logger().info("donduruma detect !!!")
                self.server.dondurma_detected = True

            else:
                self.server.dondurma_detected = False
            self.temp_signal = signal.data
            
        else:
            pass



class Hi5_Sound:
    def __init__(self):
        self.path = '/home/bo/hi5_prj/src/hi5_prj/sound/'
        self.bgm_volume = 1.0  # BGM의 기본 볼륨 설정
        self.effect_volume = 1.0  # 효과음의 기본 볼륨 설정
    
        
    def sound_init(self):
        # pygame 에서 music vs sound
        # music: 배경음악 재생을 위해 사용
        # sound: 효과음을 위해 사용

        # BG sound.
        pygame.init()

        # Effect sound
        self.sounds = pygame.mixer
        self.sounds.init()

    def BGM_play(self):
        pygame.mixer.music.load(self.path +'bgm.mp3')  # Loading File Into Mixer
        pygame.mixer.music.play(-1)  # Playing It In The Whole Device
        pygame.mixer.music.set_volume(self.bgm_volume)

    def Effect_play(self, name):
        # 현재 BGM 볼륨 저장
        current_volume = pygame.mixer.music.get_volume()
        # BGM 볼륨 줄이기
        pygame.mixer.music.set_volume(0.3 * self.bgm_volume)

        effect = self.sounds.Sound(self.path + name)

        # 효과음 볼륨을 2배로 설정
        effect.set_volume(self.effect_volume * 2.0)

        channel = effect.play()
        print(f"Effect sound {name} played.")
        
        # 효과음 재생이 끝날 때까지 기다렸다가 BGM 볼륨 복원
        while channel.get_busy():
            pygame.time.wait(100)
        pygame.mixer.music.set_volume(current_volume)


class RobotMainNode(Node):
    def __init__(self, robot):
        super().__init__('hi5')

        self.init_variable(robot)
        self.init_serivce_client()
        self.int_service()

        qos_profile_system = QoSPresetProfiles.SYSTEM_DEFAULT.value
        self.capsule_queue = queue.Queue()

        """ self.intruder_subscriber = self.create_subscription(Bool, 'intruder_signal', self.intruder_callback, qos_profile_system) #접근하는 사람 감지
        self.donduruma_subscriber = self.create_subscription(Bool, 'donduruma_pub', self.donduruma_callback, qos_profile_system) """

        self.wait_for_service(self.order_status_client, "order status client")
        self.wait_for_service(self.robot_status_client, "robot status client2")

        self.test_main()

    def init_variable(self, robot):
        self.alive = True
        self._arm = robot
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.state = 'stopped'
        self.sound = Hi5_Sound()
        self.sound.sound_init()
        self.sound.BGM_play()
        self.threads = []  #

        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500

        self.capsule_position = 0 
        self.detect_sealing = None
        self.donduruma_status = 0
        self.temp_signal = False
        self.temp1_signal = None
        self.order_status = None
        self.robot_status = None
        self.robot_status_flag = 1
        self.capsule_time = 0.0
        self.donduruma_detected = None
        self.target_x = None
        self.target_y = None
        self.target_angle = None
        

        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear

        #--------Hi5----------
        self.position_home = [180.00002, -30, 10, 180.00002, 50.000021, 0.0] #angle
        self.position_icecream = [243.1, 134.7, 300, -59.6, 88.5, 29.5] #linear
        self.position_finish = [270.000001, -24.999982, 14.999978, 180.00002, 50.000021, 0.0] #angle
        self.position_jig_A_grab = [-255.2, -133.8, 200, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -127.0, 200, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 200, 5.7, 88.9, -50.1] #linear
        self.position_topping_A = [-194.485367, 165.352158, 300, 15.641404, 89.68411, 143.873541] #Linear
        self.position_topping_B = [-133.771973, 141.502975, 300, 53.655951, 89.68411, 143.873541] #Linear
        self.position_topping_C = [-63.588264, 163.637115, 300, 90, 89.68411, 143.873541] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.5, -25.6, -88.5, 97.8] #linear
        self.Dondurma_start_position = [0.0, -500, 285, 54.735632, 89.999981, -35.264406] #linear
        self.Dondurma_end_position = [0.0, -180, 285, 54.735632, 89.999981, -35.264406] #linear


    def init_serivce_client(self):
        self.order_status_client = self.create_client(OrderCall, 'order_status') #future done callback 필요없음 그냥 일방적인 소통.
        self.robot_status_client = self.create_client(Status, 'robot_status_v2')

    def int_service(self):
        #서비스 만들기
        self.aruco_status_service = self.create_service(SetInt16ById, "aruco_status", self.aruco_callback)  #ACCEPETD FROM VISION2
        self.star_status_service = self.create_service(SetBool, "star_status", self.star_callback)          #ACCEPETD FROM VISION2
        self.order_call_service = self.create_service(OrderCall, "order_call", self.ordercall_callback)     #ACCEPTED FROM KIOSKMANAGER
        self.trash_detect_service = self.create_service(SetBool, "trash_detect", self.trash_detect_callback)#ACCEPETD trash FROM YOLO
        self.trash_position_service = self.create_service(OrderCall, "trash_position_detect", self.trash_position_detect_callback)

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f"{service_name} Service is ready")

#########################################################################################
#serivce callback

    def aruco_callback(self, req, res): # aruco 
        print(req.id)
        if req.id == 0 :
            self.capsule_position = 1
        elif req.id == 1:
            self.capsule_position = 2
        elif req.id == 2:
            self.capsule_position = 3
        else:
            self.get_logger().info("aruco id's error")
        self.capsule_queue.put(self.capsule_position) # queue 형식으로 들어온 capsule 위치 추가 -> 순차적 처리 위함, 처리됐을때 큐에서 제거하는거 추가필요 .get()

        self.queue_contents = list(self.capsule_queue.queue) #queue의 객체를 로깅할때는 기본적으로 메모리주소, -> list화해야함
        self.get_logger().info(f"Queue : {self.queue_contents}")
        self.get_logger().info(f"Aruco Callback Request id: {req.id}, data: {req.data}")
        # 응답 설정
        res.message = "Aruco ID processed successfully"
        self.order_status_call(1)
        return res

    def star_callback(self, req, res):
        if req.data == True:
            self.detect_sealing = True
            self.get_logger().info("Star founded!!!")
            
        else:
            self.detect_sealing = False
            self.get_logger().info("Star not founded!!!")

        self.order_status_call(2)
        res.success = True
        res.message = "detect_sealing_service callback"
        return res

    def ordercall_callback(self, req, res):
        if req.data:
            json_data = json.loads(req.data) #json_data에 주문 정보 담겨있음
            self.get_logger().info("OrderCall Service !!")
            self.get_logger().info(f"{json_data}")

            order = json_data['OR']  # OR가 딕셔너리라고 가정
            self.icecream = order['icecream']
            self.topping = order['topping']
            print(f"icecream: {self.icecream}, topping: {self.topping}")

            # 토핑의 갯수 계산
            self.topping_list = [t.strip() for t in self.topping.split(',')]
            self.topping_count = len(self.topping_list)
            print(self.topping_list)

            self.sound.Effect_play(f'order.mp3')
            self.sound.Effect_play(f'{self.icecream}.mp3')
            for i in self.topping_list:
                self.sound.Effect_play(f'{i}.mp3')
            self.sound.Effect_play(f'makestart.mp3')

            self.order_status_call(0)
            self.robot_status = 1
        else:
            self.get_logger().info("OrdrCall no data")
            self.robot_staus = 0
        
        res.success = True
        res.message = "order call service callback"
        return res
    
    def trash_detect_callback(self, req, res):
        self.get_logger().info(f"trash detect callback active data:{req.data}")
        if req.data:
            self.get_logger().info("trash detected !!!!")
            self.trash_detect = req.data #self.trash_detect <false로 바꿀위치 찾아야함
            self.mode = 0
            res.success = True
            res.message = "trash detect callback is activate!"
        else:
            self.get_logger().info("trash detect callback no req")
            res.success = False
            res.message = "trash detect callback is failed "
        return res

    def trash_position_detect_callback(self, req, res):
        self.get_logger().info(f"trash position detect callback service data: {req.data}")
        if req.data:
            self.get_logger().info("trash posion detected !!!")
            self.pos_x, self.pos_y, self.pos_angle = req.data.split(',')
            self.get_logger().info(f"Trash pos X: {self.target_x}, Y: {self.target_y}, Angle: {self.target_angle}")
            self.mode = 1
            res.success = True
            res.message = "trash position detect callback is activated ! "
        else:
            self.get_logger().info(f"trash position detect callback failed")
            res.success = False
            res.message = "trash position detect callback is failed "
        return res

################################################################################
## client 날리는 함수
            
    def order_status_call(self, data):
        self.get_logger().info(f"OrderStatus : {data}")

        if data == 4 or 0:
            self.robot_status_flag == 1
        else:
            pass

        self.order_status = data
        or_request = OrderCall.Request()
        or_request.data = f"OS, {self.order_status}"
        self.robot_status_call(data)
        self.order_status_client.call_async(or_request) # this is for vision2
    
    def robot_status_call(self, data):
        if self.robot_status_flag == 1:
            self.get_logger().info(f"RobotStatus: {data}")
            if data == 5:
                rs_request = Status.Request()
                rs_request.cmd = "RS"
                rs_request.status = 0 #대기중
            else:
                rs_request = Status.Request()
                rs_request.cmd = "RS"
                rs_request.status = 1 #제조중
            self.robot_status_client.call_async(rs_request) # this is for kiosk
            self.robot_status_flag = 0
        else:
            pass
            

#####################################################
# 동작 method

    def _robot_init(self):
            self._arm.clean_warn()
            self._arm.clean_error()
            self._arm.motion_enable(True)
            self._arm.set_mode(0)
            self._arm.set_state(0)
            time.sleep(1)
            self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.register_state_changed_callback(self._state_changed_callback)
            if hasattr(self._arm, 'register_count_changed_callback'):
                self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code,
                                                                                                self._arm.connected,
                                                                                                self._arm.state,
                                                                                                self._arm.error_code,
                                                                                                ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                       ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    @property
    def order_status(self):
        return self._order_status
    
    @order_status.setter
    def order_status(self, value):
        self._order_status = value
        self.test_main()

    def position_reverse_sealing_fail(self, linear_jig_position = [-257.3, -138.3, 192.1, 68.3, 86.1, -47.0]):
        reverse_position = linear_jig_position.copy()
        reverse_position[2] = reverse_position[2] - 10
        reverse_position[3] = -reverse_position[3]
        reverse_position[4] = -reverse_position[4]
        reverse_position[5] = reverse_position[5] - 180
        return reverse_position

 #-----------------------------------------------Hi5-motion-----------------------------------------------
    def motion_home(self):
        self.capsule_position = 0 

        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # press_up
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return

        self._angle_speed = 20
        self._angle_acc = 200
        
        print('motion_home start')

        code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        self.motion_num = 0
        print('motion_home finish')
        

    def motion_grab_capsule(self):
        print('motion_grab_capsule start')
        # 컵 추출 준비
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        print(self.capsule_position)
        while True:
            if self.capsule_position == 1:
                self.return_capsule_position = self.capsule_position
                # 지그 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=100,
                                                    mvacc=100, wait=False, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                # 지그A 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[179.5, 28.3, 31.4, 113.1, 91.5, 0], speed=100,
                                                    mvacc=100, wait=True, radius=20.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return

                code = self._arm.set_position(*self.position_jig_A_grab, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                break

            elif self.capsule_position == 2:
                self.return_capsule_position = self.capsule_position
                print("들어옴")
                # 지그 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=100,
                                                    mvacc=100, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                print("지그 가는 웨이포인트")
                # 지그B 캡슐 그랩
                code = self._arm.set_position(*self.position_jig_B_grab, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return 
                

                print("지그B 캡슐 그랩")
                break

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gri1pper'):
            return
        time.sleep(1)

        code = self._arm.set_position(z=100, radius=0, speed=100, mvacc=1000, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #실링체크 웨이포인트
        code = self._arm.set_servo_angle(angle=[145, -27, 13.4, 95.7, 80.1, 156.4], speed=100,
                                            mvacc=100, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 1
        print('motion_grab_capsule finish')
    
    def motion_check_sealing(self):
        print('motion_check_sealing start')

        code = self._arm.set_servo_angle(angle=[179.1, -85.6, 13.1, 182.6, -3.2, 180], speed=100,
                                            mvacc=100, wait=True, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 2
        
        self.order_status_call(2)
        print('motion_check_sealing finish')


    def motion_place_capsule(self):
        print('motion_place_capsule start')
        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100


        #토핑 아래로 지나가는 1
        code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #토핑 아래로 지나가는 2
        code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #토핑 아래로 지나 올라옴
        code = self._arm.set_servo_angle(angle=[8.4, -42.7, 23.7, 177.4, 31.6, 3.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #탬핑기 바로 앞
        code = self._arm.set_servo_angle(angle=[8.4, -32.1, 55.1, 96.6, 29.5, 81.9], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #캡슐 삽입
        code = self._arm.set_position(*self.position_before_capsule_place, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #캡슐 내려놓음
        code = self._arm.set_position(*self.position_capsule_place, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # 뒤로빠진 모션
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        self.motion_num = 3
        print('motion_place_capsule finish')
        
        time.sleep(0.5)

    def motion_place_fail_capsule(self):
        print('motion_place_fail_capsule start')
        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        code = self._arm.set_servo_angle(angle=[176, -2, 30.7, 79.5, 98.2, 29.3], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        if self.return_capsule_position == 1:
            code = self._arm.set_position(*[-255.2, -133.8, 250, 68.3, 86.1, -47.0], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.return_capsule_position == 2:
            code = self._arm.set_position(*[-152.3, -127.0, 250, 4.8, 89.0, -90.7], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_B_grab, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gri1pper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        code = self._arm.set_tool_position(z=-70, speed=tcp_speed,
                                            mvacc=tcp_acc, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                            mvacc=100, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        print('motion_place_fail_capsule finish')


    def motion_grab_cup(self):
        print('motion_grab_cup start')

        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return

        # 컵가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #컵 위치로 이동 잡기
        code = self._arm.set_position(*self.position_cup_grab, speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(1)
        
        # 컵 잡고 올리기
        code = self._arm.set_position(z=100, radius=0, speed=150, mvacc=1000, relative=True,
                                      wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[2.9, -28.9, 26.2, 110.2, -26.1, -30.1], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[3, -8.7, 26.2, 107.4, 60.5, 19.7], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self.motion_num = 4
        print('motion_grab_cup finish')


    def motion_helix(self, radius, height, turns, mode, speed=20):
        # 나선의 중점 설정
        if mode == 0:
            topping_A_center = self.position_topping_A
            topping_A_center[2] = topping_A_center[2]+30

            helix_position = topping_A_center
        elif mode == 1:
            topping_B_center = self.position_topping_B
            topping_B_center[2] = topping_B_center[2]+30

            helix_position = topping_B_center
        elif mode == 2:
            topping_C_center = self.position_topping_C
            topping_C_center[2] = topping_C_center[2]+30

            helix_position = topping_C_center
        elif mode == 3:
            icecream_center = self.position_icecream
            icecream_center[2] = icecream_center[2]+60

            helix_position = icecream_center
            

        # t 값 생성
        t_values = np.linspace(0, turns * 2 * np.pi, 1000)  # 100개의 점을 생성

        # x, y, z 좌표 계산
        x_values = radius * np.cos(t_values) + helix_position[0]
        y_values = radius * np.sin(t_values) + helix_position[1]
        z_values = height * t_values + helix_position[2]

        # 시작점 설정
        start_point = (x_values[0], y_values[0], z_values[0])
        end_point = (x_values[-1], y_values[-1], z_values[-1])

        a = [start_point[0], start_point[1], start_point[2], 
                                        helix_position[3], helix_position[4], helix_position[5]]
    
        # 시작 위치로 이동
        code = self._arm.set_position(*[start_point[0], start_point[1], start_point[2], 
                                        helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 나선을 따라 이동
        for x, y, z in zip(x_values, y_values, z_values):
            code = self._arm.set_position(*[x, y, z, helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

        # 끝점으로 이동
        code = self._arm.set_position(*[helix_position[0], helix_position[1], end_point[2],
                                        helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

    def motion_spiral(self, radius, total_height, total_turns=5, speed=20):
        # 현재 로봇 위치 가져오기
        tuple_c = self._arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]

        # 중심 좌표 설정
        cx = current_position[0]
        cy = current_position[1]
        cz = current_position[2]
        num_points = 1000

        # total_height가 양수이면 위로, 음수이면 아래로 원뿔을 생성
        z = np.linspace(0, total_height, num_points)  # 높이가 0에서 total_height까지 증가 또는 감소
        r = radius * (1 - np.abs(z / total_height))  # z의 절대값을 사용하여 반지름이 줄어듦

        theta = np.linspace(0, total_turns * 2 * np.pi, num_points)  # 0에서 total_turns * 2π까지 회전

        # 극 좌표를 직교 좌표로 변환하고 중심 좌표 더하기
        x = r * np.cos(theta) + cx
        y = r * np.sin(theta) + cy
        z = cz + z  # cz를 기준으로 z를 증가 또는 감소시킴

        # 로봇의 위치를 시간에 따라 업데이트하는 예제
        for i in range(num_points):
            current_x = x[i]
            current_y = y[i]
            current_z = z[i]

            # 로봇의 자세는 원래의 자세를 유지 (roll, pitch, yaw)
            roll = current_position[3]
            pitch = current_position[4]
            yaw = current_position[5]

            # 위치 설정 명령
            code = self._arm.set_position(current_x, current_y, current_z, roll, pitch, yaw,
                                          speed=speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

    def motion_make_icecream(self):
        print('motion_make_icecream start')
        # 아이스크림 위치
        code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=150,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self.sound.Effect_play(f'making.mp3')
        # 아이스크림 추출 준비
        code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        code = self._arm.set_position(z=60, radius=0, speed=30, mvacc=100, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        time.sleep(10)

        # self.motion_sprial2()
        # self.motion_spiral3(8, -5, 3, 30)
        self.motion_spiral(radius=15, total_height=-40, total_turns=5, speed=50)

        # self.motion_helix(8, -1, 3, 3, 30)

        time.sleep(5)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        self.motion_num = 5
        print('motion_make_icecream finish')
        self.order_status_call(3)



    def motion_get_topping(self, topping):
        tcp_speed = 50
        tcp_acc = 100

        if topping == 0:
            #-----------토핑 A----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*self.position_topping_A, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 0, 40)

            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            
        elif topping == 1:
            #-----------토핑 B----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 1, 40)

            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            
        elif topping == 2:
            #-----------토핑 C----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*self.position_topping_C, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 2, 40)

            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            

    def motion_topping(self, topping_list):
        print('motion_topping start')
        angle_speed = 100
        angle_acc = 1000
        #토핑 아래로 지나가는
        code = self._arm.set_servo_angle(angle=[16.816311, -37.649286, 4.669835, 96.031508, 82.383711, 41.868891], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 토핑 아래
        code = self._arm.set_servo_angle(angle=[126.573252, -53.387978, 1.068108, 182.537822, 35.668972, -4.317638], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=00.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        for select in topping_list:
            print(select)
            if  select == 'oreo':
                self.motion_get_topping(0)
            elif  select == 'chocoball':
                self.motion_get_topping(1)
            elif  select == 'cereal':
                self.motion_get_topping(2)
            else:
                pass

        # 배출 웨이포인트
        code = self._arm.set_servo_angle(angle=self.position_home, speed=30,
                                         mvacc=100, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=self.position_finish, speed=30,
                                            mvacc=100, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 6
        self.get_logger().info('motion_topping finish')
        self.order_status_call(4)


    
    def motion_Dondurma(self):

        code = self._arm.set_position(*self.Dondurma_start_position, speed=100,
                                            mvacc=5000, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        self.sound.Effect_play(f'Dondurma_start.mp3')
        
        Dondurma_count = random.randrange(2,5)
        print(Dondurma_count)
        Dondurma_y_axis = 500

        for i in range(0,Dondurma_count):
            Dondurma_y_axis = Dondurma_y_axis-abs((self.Dondurma_start_position[1]-self.Dondurma_end_position[1])/Dondurma_count)
            Dondurma_x_axis = random.randrange(-260,260)
            Dondurma_z_axis = random.randrange(240,340)

            # print("{} / {}  y = {} x = {} z = {}" .format(i,Dondurma_count,Dondurma_y_axis,Dondurma_x_axis,Dondurma_z_axis))
            
            if self.dondurma_detected:
                code = self._arm.set_position(*[Dondurma_x_axis, -Dondurma_y_axis, Dondurma_z_axis, self.Dondurma_end_position[3],
                                                self.Dondurma_end_position[4], self.Dondurma_end_position[5]], speed=450, mvacc=5000, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                
                self.sound.Effect_play(f'Dondurma_effect_{i}.mp3')
            print(f'i                      {i}')
            time.sleep(1)
            
        code = self._arm.set_position(*self.Dondurma_end_position,
                                        speed=50, mvacc=5000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self.sound.Effect_play(f'THX')
        time.sleep(10)

        return True

    def motion_stack_icecream(self):
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        self.sound.Effect_play(f'stack_icecream.mp3')
        code = self._arm.set_position(*self.Dondurma_end_position,
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*[60.558903, -156.656799, 285, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=178, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(*[117.877991, -102.960449, 178, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=300, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_servo_angle(angle=[270, -7.800018, 32.599981, 267.599996, 90, -40], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=self.position_home, speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        self.capsule_time = time.time() #돈두르마 감지 x -> 현재시간
        self.order_status_call(5)
        
    def motion_dump_icecream(self):
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        self.sound.Effect_play(f'dump_icecream.mp3')
        code = self._arm.set_servo_angle(angle=[270, -7.800018, 32.599981, 267.599996, 90, -40], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_position(*[117.877991, -102.960449, 300, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=178, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(0.5)
        
        code = self._arm.set_position(*[60.558903, -156.656799, 178, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=250, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_servo_angle(angle=[180, 15.58, 35, 269.87, 80, -19.34], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 220, 70, 130], speed=50,
                                            mvacc=100, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # # 터는 동작
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.set_servo_angle(angle=[135, -6, 11, 230, 60, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 220, 70, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 230, 60, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                            mvacc=100, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
        

    def motion_trash_capsule(self):
        self.motion_num = 8
        print('motion_trash_capsule start')
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100
        
        # 홈에서 템핑가는 토핑밑
        code = self._arm.set_servo_angle(angle=[51.2, -8.7, 13.8, 95.0, 86.0, 17.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        # 홈에서 템핑가는 토핑밑에서 올라옴
        code = self._arm.set_servo_angle(angle=[-16.2, -19.3, 42.7, 82.0, 89.1, 55.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.sound.Effect_play(f'remove_capsule.mp3')
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[-19.9, -19.1, 48.7, 87.2, 98.7, 60.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 템핑 앞
        code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 캡슐 집고 올림
        code = self._arm.set_position(*self.position_capsule_grab, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(z=30, radius=-1, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 캡슐 집고 올림
        code = self._arm.set_position(*[221.9, -5.5, 500.4, -153.7, 87.3, -68.7], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 탬핑에서부터 버리는 동작
        code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 50.4, 78.1, 63.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 뒤집어서 쓰래기통 위
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 터는 동작
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        # time.sleep(2)
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        
        # 털고 복귀하는 토핑 밑
        code = self._arm.set_servo_angle(angle=[28.3, -9.0, 12.6, 85.9, 78.5, 20.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        
        code = self._arm.set_servo_angle(angle=[149.3, -9.4, 10.9, 114.7, 69.1, 26.1], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 홈 위치
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self.motion_num = 8
        self.order_status_call(5)
        print('motion_trash_capsule finish')


    def motion_recovery(self, num):
        angle_speed = 60
        angle_acc = 500
        tcp_speed = 50
        tcp_acc = 100

        if num in [1, 2, 6]:
            print("복구 시작 : 캡슐 잡은 상태")
            self.motion_place_fail_capsule()

        elif num in [3, 4]:
            print("복구 시작 : 캡슐 삽입한 상태")

            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            code = self._arm.set_servo_angle(angle=[-15.503321, -4.641989, 71.499862, 85.082915, 99.572387, 74.310964], speed=30,
                                            mvacc=100, wait=False, radius=10.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            # 캡슐 집고 올림
            code = self._arm.set_position(*self.position_capsule_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(z=25, radius=-1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            self._tcp_speed = 100
            self._tcp_acc = 1000

            # 캡슐 빼기
            code = self._arm.set_position(*[221.9, -15.5, 500.4, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            

            code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 70, 78.1, 63.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=10.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            #토핑 아래로 지나가는 2
            code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=50.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            #토핑 아래로 지나가는 1
            code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=40.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_place_fail_capsule()

        elif num == 5:
            print("복구 시작 : 아이스크림 제조한 상태")
            #토핑 아래로 지나가는
            code = self._arm.set_servo_angle(angle=[16.816311, -37.649286, 4.669835, 96.031508, 82.383711, 41.868891], speed=angle_speed,
                                                mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            # 토핑 아래
            code = self._arm.set_servo_angle(angle=[126.573252, -53.387978, 1.068108, 182.537822, 35.668972, -4.317638], speed=angle_speed,
                                                mvacc=angle_acc, wait=False, radius=00.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            # 배출 웨이포인트
            code = self._arm.set_servo_angle(angle=self.position_home, speed=30,
                                                mvacc=100, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_place_fail_capsule()


    def motion_grap_squeegee(self):
        #스키지 잡으러 가기
        code = self._arm.set_position(*[-249, 128.5, 171, -90, 90, 90], speed=100,
                                                mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        # 스키지 위치
        code = self._arm.set_position(*[-323.5, 128.5, 171, 90, 90, -90], speed=30,
                                                mvacc=100, radius=00.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        # 우측 배드 초기 위치
        code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=100,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        

    def motion_sweeping_right(self):        
        up_speed = 200
        up_acc = 2000
        down_speed = 100
        down_acc = 1000
        code = self._arm.set_position(*[-130 ,  -83  , 400 , 180.00002, -0.544826, 88.453969], speed=up_speed,
                        mvacc=up_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return


        # 2.sweep
        code = self._arm.set_position(y=5, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=-180, y=30, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 3.up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        #6. move to point 3
        code = self._arm.set_position(*[-250, -83, 368.65, 180.00002, -0.544826, 88.453969], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(z=265, speed=down_speed,
                        mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # sweep 
        code = self._arm.set_position(y=60, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # rotate and 7.move to 5 again  
        code = self._arm.set_position(*[-180,  -153,  368.65, -180.00002, 0.544826, -88.453969 ], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # sweep
        code = self._arm.set_position(y=-30, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=-132, y=-10, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 9.up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 10.move to 7 & down
        code = self._arm.set_position(*[-250,  -153.66,  368.65, -180.00002, 0.544826, -88.453969], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=265, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 11.sweep
        code = self._arm.set_position(y=70, speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                    mvacc=up_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #side sweep        
        code = self._arm.set_position(*[-200.0, 64.623001, 368.649994, 180, 0, -0], speed=up_speed,
                                    mvacc=up_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[-290, 169.60968, 265, 180, 0.0, -0.0], speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(x=150, speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, relative=True,wait=True)
        if not self._check_code(code, 'set_position'):
            return


    def motion_sweeping_left(self): 
        up_speed = 200
        up_acc = 2000
        down_speed = 100
        down_acc = 1000

        # 좌측 배드 초기 위치
        code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=up_speed,
                                mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # move to point (1)
        code = self._arm.set_position(*[135,-53.5, 368.65, 180, 0.0, 90], speed=up_speed,
                                mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 2.sweep
        code = self._arm.set_position(y=5, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=185, y=30, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # move to point (3)
        code = self._arm.set_position(*[262,-53.5, 368.65, 180, 0, 90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return              
        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # 1st sweep
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return            
        #rotate & move to point (5)
        code = self._arm.set_position(*[185, -140, 368.65, -180, 0.0, -90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        #down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 8.sweep 53.66에서 수정함 -> 56.66 -> 59.66
        code = self._arm.set_position(y=-30, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=132, y=-10, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #move to point(7)
        code = self._arm.set_position(*[262, -140, 368.65, -180,0,-90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 2st sweep
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return


    def motion_place_squeegee(self):
        # 우측 배드 초기 위치
        code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=100,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 스키지 위치
        code = self._arm.set_position(*[-323.5, 128.5, 200, 90, 90, -90], speed=100,
                                                mvacc=1000, radius=00.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=172, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.set_position(*[-249, 128.5, 172.5, -90, 90, 90], speed=30,
                                                mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_pick_felldown_trash(self, target_x, target_y, deg):
        code = self._arm.set_position(x=target_x, y=target_y, yaw=deg, speed=100,
                                        mvacc=1000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(z=250, speed=30,
                                        mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=300, speed=100,
                                        mvacc=1000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_dump_trash(self):
        angle_speed = 50
        angle_acc = 500
        tcp_speed = 50
        tcp_acc = 100

        time.sleep(3)
        tuple_c = self._arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]
        print(f'current_position[0] {current_position[0]}')

        if current_position[0] > 0:
            # left way point
            code = self._arm.set_servo_angle(angle=[41.185925, -42.989596, 31.248774, -0.0, 74.238371, 176.185954], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # # left way point
            # code = self._arm.set_position(*[80, 70, 350.0, 180, -0.0, -135], speed=self._tcp_speed,
            #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #     return
            # middle
            code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(y=180, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # 터는 동작
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(2)

            # right way point
            code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
        else:
            """ right way point """
            code = self._arm.set_servo_angle(angle=[138.814095, -42.989596, 31.248774, -0.0, 74.238371, 183.814085], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            """ right way point
            # code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=self._tcp_speed,
            #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
            # if not self._check_code(code, 'set_position'):
            #     return
            # middle """
            code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(y=180, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(2)

            # right way point
            code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
    def motion_cleaning(self, mode=None, target_x=None, target_y=None, target_deg=None):
        tcp_speed = 50
        tcp_acc = 100

        time.sleep(3)
        tuple_c = self._arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]
        print(f'current_position[0] {current_position[0]}')
        self.mode = 0

        self.sound.Effect_play(f'cleaning.mp3')

        if self.mode == 0: #Squeeze
            if current_position[0] > 0:
                self.left_to_right()
            else :
                pass
                
            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                        mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_grap_squeegee()
            
            self.motion_sweeping_right()

            self.right_to_left()
            
            self.motion_sweeping_left()

            self.left_to_right()

            self.motion_place_squeegee()

            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                                    mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        elif self.mode == 1: #felldown
            if current_position[0] > 0 and self.target_x > 0:
                # 좌측 배드 초기 위치
                code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            elif current_position[0] < 0 and self.target_x < 0:
                
                # 우측 배드 초기 위치
                code = self._arm.set_position(*[-200,  0, 400, 180, 0, 90], speed=tcp_speed,
                                                mvacc=tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

            elif current_position[0] > 0 and self.target_x < 0:
                self.left_to_right()

                # 우측 배드 초기 위치
                code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=tcp_speed,
                                                mvacc=tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
            elif current_position[0] < 0 and self.target_x > 0:
                self.right_to_left()

                # 좌측 배드 초기 위치
                code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

            self.motion_pick_felldown_trash(self.target_x, self.target_y, self.target_angle)

            self.motion_dump_trash()
            
            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                                    mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return



    def right_to_left(self):
        # right way point
        code = self._arm.set_servo_angle(angle=[138.814095, -42.989596, 31.248774, -0.0, 74.238371, 183.814085], speed=50,
                                        mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # middle
        code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        # left way point
        code = self._arm.set_position(*[80, 70, 350.0, 180, -0.0, -135], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
    def left_to_right(self):
        # # left way point
        code = self._arm.set_servo_angle(angle=[41.185925, -42.989596, 31.248774, -0.0, 74.238371, 176.185954], speed=50,
                                        mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # middle
        code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        # right way point
        code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_tracking(self):

        flag = 1

        while True:
            tuple_c = self._arm.get_position()
            list_c = tuple_c[1]
            current_position = list_c[:6]

            if len(self.tracking_position) == 2:
                print(f'\n\n               로봇         \n\n\n{self.tracking_position[0]} , {self.tracking_position[1]}\n\n\n\n\n')

                a = (self.tracking_position[0] - current_position[0])

                b = (self.tracking_position[1] - current_position[1])
                
                if flag == 1:
                    print('들어옴')
                    code = self._arm.set_position(x=a,y=b, speed=50, relative=True,
                                            mvacc=100, radius=0.0, wait=True)
                    flag = 0
                print(f'{a} , {b},  {(a > 1 or a < -1)}, {(b > 1 or b < -1)}')
                time.sleep(0.5)

                if (a > 1 or a < -1) or (b > 1 or b < -1):
                    print(f'들어옴 {current_position[0]}        {a}')
                    flag = 1
        


    def test_main(self):
        try:
            # self.motion_recovery(5)
            if self.order_status == 0 :
                self.get_logger().info("start home")
                self.motion_home()

            elif self.order_status == 1 :
                self.get_logger().info("start grab capsule")
                self.motion_grab_capsule()
                self.get_logger().info("start sealing check")
                self.motion_check_sealing()

            elif self.order_status == 2 and self.detect_sealing == True:
                self.motion_place_capsule()
                self.motion_grab_cup()
                self.motion_make_icecream()

            elif self.order_status == 2 and self.detect_sealing == False:
                self.motion_place_fail_capsule()

            elif self.order_status == 3:
                self.motion_topping(self.topping_list)

            elif self.order_status == 4:
                if not self.motion_Dondurma():
                    self.motion_stack_icecream()
                else:
                    pass
                self.motion_trash_capsule()
                
            elif self.order_status == 5:
                self.motion_cleaning()

            else:
                pass

        except Exception as e:
            self.get_logger().error(f"Error:{e}")

        
def main(args = None):

    try:
        rclpy.init(args=args)
        executor = MultiThreadedExecutor()
        RobotMainNode.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
        arm = XArmAPI('192.168.1.185', baud_checkset=False)
        sub = RobotMainNode(arm)
        node = SubscriptionServer(sub)

        executor.add_node(sub)
        executor.add_node(node)

        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()

        

if __name__ == '__main__':
    main()