import socket
import threading
import queue
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hi5_message.srv import OrderCall

# HOST = '192.168.1.9'
HOST = '172.30.1.83'

KIOSK_PORT = 9003

class KioskManager(Node):
    def __init__(self, host, port):
        super().__init__('kiosk_manager_node')
        self.host = host
        self.port = port
        self.data_queue = queue.Queue()  # 데이터 처리를 위한 큐
        self.server_socket = None
        self.client_list = []
        self.running = True  # 노드 실행 여부 플래그

        #ros통신 송신용 client
        self.order_call_rm_service = self.create_client(OrderCall, "order_call")       #hi5통신
        self.order_call_db_service = self.create_client(OrderCall, "order_call_db") #db통신

        #ros통신 수신용 service
        self.order_status_service = self.create_service(OrderCall, "order_status", self.order_status_callback) #hi5 수신

        # 서버 소켓 설정 및 수신 대기
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"Listening on {self.host}:{self.port}")

        # 데이터 처리 스레드 시작
        self.data_thread = threading.Thread(target=self.process_data)
        self.data_thread.start()

        # tcp 통신 클라이언트 수락 스레드 시작
        self.accept_thread = threading.Thread(target=self.accept_clients)
        self.accept_thread.start()

        self.wait_for_service(self.order_call_rm_service, "order call service")
        self.wait_for_service(self.order_call_db_service, "order call db service")
    
    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        self.get_logger().info(f"{service_name} Service is ready")

    def accept_clients(self):
        while self.running:
            self.conn, addr = self.server_socket.accept()
            self.client_list.append(self.conn)
            client_thread = threading.Thread(target=self.handle_client, args=(self.conn, addr))
            client_thread.start()
            print(f"{addr}에 대한 스레드 시작됨")

    def handle_client(self, conn, addr): #tcp로 kiosk gui에서 받은 data
        print(f"{addr}에서 연결됨")
        try:
            with conn:
                while self.running:
                    recv = self.conn.recv(1024)
                    if not recv:
                        break
                    print(f"{addr}로부터 수신: {recv.decode()}")
                    # 받은 데이터를 큐에 넣음
                    self.data_queue.put((recv.decode(), self.conn))
        except Exception as e:
            print(f"클라이언트 처리 중 오류: {e}")
        finally:
            if self.conn in self.client_list:
                self.client_list.remove(self.conn)
            self.conn.close()

    def process_data(self): #queue에 쌓아놨던 order 처리
        while self.running:
            data, self.conn = self.data_queue.get()
            try:
                if "OR" in data:
                    self.send_to_db_request(data)
                    self.send_to_rm_request(data)
                    print("CMD : OR 처리중")
                if "TR" in data:
                    self.send_to_db_request(data)
                    print("CMD : TR 처리중")
                else:
                    print("Unknown CMD")

            except json.JSONDecodeError as e:
                print(f"JSON 파싱 오류: {e}")
            except Exception as e:
                print(f"데이터 처리 중 오류: {e}")
                
    def send_to_rm_request(self, data):
        request = OrderCall.Request()
        request.data = data  # json -> string
        
        future = self.order_call_rm_service.call_async(request)
        future.add_done_callback(lambda future: self.order_call_rm_response(future))#dbm

    def send_to_db_request(self, data): # order 서비스 콜 to db
        if "OR" in data:
            print(f"Received order request: {data}")
            # JSON 데이터를 String 메시지로 변환하여 서비스 요청 생성
            request = OrderCall.Request()
            request.data = data  # json -> string
            
            future = self.order_call_db_service.call_async(request)
            future.add_done_callback(lambda future: self.order_call_db_response(future))#dbmanager response처리
        elif "TR" in data:
            print(f"Received order request: {data}")
            # JSON 데이터를 String 메시지로 변환하여 서비스 요청 생성
            request = OrderCall.Request()
            request.data = data  # json -> string
            
            future = self.order_call_db_service.call_async(request)
            future.add_done_callback(lambda future: self.order_call_db_response(future))#dbmanager response처리
        elif "OS" in data:
            print(f"Received order status request: {data}")
            # JSON 데이터를 String 메시지로 변환하여 서비스 요청 생성
            request = OrderCall.Request()
            request.data = data  # json -> string
            
            future = self.order_call_db_service.call_async(request)
            # future.add_done_callback(lambda future: self.order_call_db_response(future))#dbmanager response처리
        else:
            self.get_logger().info("send to db request error")

    def order_call_rm_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"{response}")

            if response.success:
                print("order call rm Success")
                self.client_send("OR, Success rm order reqeust")
            else:
                self.client_send("OR, Error receive rm order reqeust")
        except Exception as e:
            print(f"서비스 콜 응답 처리 중 오류: {e}")
            self.client_send("ER, Error processing db order request")

    def order_call_db_response(self, future):
        try:
            response = future.result()  # Service call의 결과를 가져옴
            self.get_logger().info(f"Response from service: {response}")

            if response.success:
                # response.message는 이미 JSON 문자열임
                message_str = response.message
                self.get_logger().info(f"Message string: {message_str}")

                # JSON 문자열을 dict로 변환
                message_dict = json.loads(message_str)
                self.get_logger().info(f"Message JSON: {message_dict}")

                if "OR" in message_dict:
                    self.get_logger().info("OR detected in message")
                    message_data = message_dict.get("OR", {})
                    self.get_logger().info(f"Message data: {message_data}")

                    if "orderId" in message_data:
                        self.get_logger().info("Processing order ID")
                        order_id = message_data.get("orderId")
                        self.get_logger().info(f"Order ID: {order_id}")
                        self.client_send(f"OR,{order_id}")
                    else:
                        data = message_data.get("detail",[])
                        msg = '\n'.join(data)
                        self.client_send(f"ER,{msg}")
                        
                elif "TR" in message_dict:
                    self.get_logger().info("TR detected in message")
                    message_data = message_dict.get("TR", {})
                    self.get_logger().info(f"Message data: {message_data}")
                    
                    if "tables" in message_data:
                        self.get_logger().info("Processing tables")
                        tables = message_data.get("tables", [])
                        self.get_logger().info(f"Tables: {tables}")
                        self.client_send(f"TR,{tables}")
                    else:
                        data = message_data.get("detail", [])
                        msg = '\n'.join(data)
                        self.client_send(f"ER,{msg}")
                        
                elif "OS" in message_dict:
                    self.get_logger().info(f"OS message: {response.message}")
                
                else:
                    self.get_logger().info(f"no data:{response.message}")
                    # 추가적인 OS 처리 로직을 여기에 추가

            else:
                self.client_send("ER,Error processing db order request")

        except Exception as e:
            self.get_logger().error(f"서비스 콜 응답 처리 중 오류: {e}")
            self.client_send("ER,Error processing db order request")


    def order_status_callback(self, req, res): 
        try:
            if req.data:
                self.get_logger().info(f"{req.data}")
                self.get_logger().info("Order Status Service !!")
                response = req.data
                cmd, data = response.split(',', 1)
                # data = data.strip()
                self.get_logger().info(f"11111111111111111cmd:{cmd}, status:{data}")
                if cmd == "OS":
                    #send to kiosk gui
                    #for send to db manager
                    message = {
                        "OS": {
                            "status": f"{data}"
                        }
                    }
                    message = json.dumps(message) #dict -> string
                    self.send_to_db_request(message)
                    self.client_send(response)
                else:
                    self.get_logger().info("error in order status callback")

                res.success = True
                res.message = "order status callback"
            else:
                print("robot manager service call error")
                res.success = False
                res.message = "No data received ORDER STATUS"
            return res
        
        except Exception as e:
            print(f"로봇매니저 서비스 콜 응답 처리 중 오류 {e}")
            self.client_send("ER,Error processing robotmanager order request")
            res.success = False
            res.message = f"Order status call failed: {e}"
            return res
    

        
    def client_send(self, msg):
        try:
            self.conn.sendall(msg.encode())
            print(f"전송 완료: {self.conn.getpeername()[0]}")
        except Exception as e:
            print(f"전송 중 오류: {e}")

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
