import socket
import threading
import queue
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import OrderCall

HOST = '172.30.1.70'
KIOSK_PORT = 9002

class KioskManager(Node):
    def __init__(self, host, port):
        super().__init__('kiosk_manager_node')
        self.host = host
        self.port = port
        self.data_queue = queue.Queue()  # 데이터 처리를 위한 큐
        self.server_socket = None
        self.client_list = []
        self.running = True  # 노드 실행 여부 플래그

        self.order_call_service = self.create_client(OrderCall, "order_call")
        
        # 서버 소켓 설정 및 수신 대기
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"Listening on {self.host}:{self.port}")

        # 데이터 처리 스레드 시작
        self.data_thread = threading.Thread(target=self.process_data)
        self.data_thread.start()

        # 클라이언트 수락 스레드 시작
        self.accept_thread = threading.Thread(target=self.accept_clients)
        self.accept_thread.start()

        self.wait_for_service(self.order_call_service, "order call service")

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f"{service_name} Service is ready")

    def accept_clients(self):
        while self.running:
            conn, addr = self.server_socket.accept()
            self.client_list.append(conn)
            client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            client_thread.start()
            print(f"{addr}에 대한 스레드 시작됨")

    def handle_client(self, conn, addr):
        print(f"{addr}에서 연결됨")
        try:
            with conn:
                while self.running:
                    recv = conn.recv(1024)
                    if not recv:
                        break
                    print(f"{addr}로부터 수신: {recv.decode()}")
                    # 받은 데이터를 큐에 넣음
                    self.data_queue.put((recv.decode(), conn))
        except Exception as e:
            print(f"클라이언트 처리 중 오류: {e}")
        finally:
            if conn in self.client_list:
                self.client_list.remove(conn)
            conn.close()

    def process_data(self):
        while self.running:
            data, conn = self.data_queue.get()
            try:
                json_data = json.loads(data)
                if "OR" in json_data:
                    self.handle_order_request(json_data, conn)
                    print("CMD : OR ")
                else:
                    print("Unknown")

            except json.JSONDecodeError as e:
                print(f"JSON 파싱 오류: {e}")
            except Exception as e:
                print(f"데이터 처리 중 오류: {e}")

    def handle_order_request(self, order_data, conn):
        print(f"Received order request: {order_data}")
        
        # JSON 데이터를 String 메시지로 변환하여 서비스 요청 생성
        request = OrderCall.Request()
        request.data = json.dumps(order_data)  # JSON 데이터를 문자열로 변환
        
        # 서비스 콜 요청
        future = self.order_call_service.call_async(request)
        future.add_done_callback(lambda future: self.order_call_response(future, conn))

    def client_send(self, msg, conn):
        try:
            conn.sendall(msg.encode())
            print(f"전송 완료: {conn.getpeername()[0]}")
        except Exception as e:
            print(f"전송 중 오류: {e}")
    
    def order_call_response(self, future, conn):
        try:
            response = future.result()
            if response.success:
                msg = f"Order placed successfully."
            else:
                msg = "Failed to place order."
            self.client_send(msg, conn)
        except Exception as e:
            print(f"서비스 콜 응답 처리 중 오류: {e}")
            self.client_send("Error processing order request", conn)

    def shutdown(self):
        self.running = False  # 실행 플래그를 False로 설정하여 스레드 루프를 종료

        # 서버 소켓을 닫음
        if self.server_socket:
            self.server_socket.close()

        # 클라이언트 연결을 닫음
        for client in self.client_list:
            client.close()

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    kiosk_manager = KioskManager(HOST, KIOSK_PORT)

    try:
        rclpy.spin(kiosk_manager)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, shutting down...")
    finally:
        kiosk_manager.shutdown()

if __name__ == '__main__':
    main()
