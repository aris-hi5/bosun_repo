import socket
import threading

# from kioskGUISrv import *

class TCPClient:
    def __init__(self, host, port):
        self.server_address = host
        self.server_port = port
        self.socket = None
        self.connected = False
        self.connect()

        # 시작 시 receive_thread 실행
        self.receive_thread = threading.Thread(target=self.receive)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.status_callback = None
        
    def robot_status_callback(self, callback):
        print("robot_status_callback")
        self.status_callback = callback    

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_address, self.server_port))
            self.connected = True
            print("Connected to server:", self.server_address, "port:", self.server_port)
        except Exception as e:
            print("Error:", e)
            
    def send(self, cmd, data):
        message = f"{cmd},{data}"
        try:
            if self.socket:
                self.socket.send(message.encode())
            else:
                print("Socket is not connected.")
        except Exception as e:
            print("Error:", e)

    def receive(self):
        while self.connected:
            try:
                if self.socket:
                    response = self.socket.recv(1024).decode('utf-8')
                    if response:
                        print(response)
                        cmd, data = response.split(',', 1)
                        if cmd == 'KS':
                            if self.robot_status_callback:
                                self.status_callback(data)
                            print(data)
                                
                        elif cmd == 'RS':
                            # if self.robot_status_callback:
                            #     self.status_callback(data)
                            # print(data)
                            pass
                                
                else:
                    print("Socket is not connected.")
            except Exception as e:
                print("Error:", e)
                self.connected = False
                self.close()
            except KeyboardInterrupt:
                self.close()
                pass
            
    def close(self):
        self.connected = False
        try:
            if self.socket:
                self.socket.close()
                print("Connection closed.")
        except Exception as e:
            print("Error:", e)
