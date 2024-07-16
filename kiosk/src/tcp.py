import socket
import threading

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

    def send(self, data):
        message = f"{data}"
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
                response = self.socket.recv(1024).decode('utf-8')
                if response:
                    print(response)
                    cmd, data = response.split(',', 1)
                    if cmd == 'KS':
                        if self.status_callback:
                            self.status_callback(data)
                        print(data)
                    elif cmd == 'RS':
                        pass  # 로봇 상태 처리는 여기에 추가
                else:
                    print("Disconnected from server.")
                    self.connected = False
                    self.close()
            except Exception as e:
                print("Error:", e)
                self.connected = False
                self.close()

    def close(self):
        self.connected = False
        try:
            if self.socket:
                self.socket.close()
                print("Connection closed.")
        except Exception as e:
            print("Error:", e)

if __name__ == '__main__':
    client = TCPClient('192.168.0.217', 8000)
    try:
        while True:
            message = input("Enter message to send (type 'exit' to quit): ")
            if message.lower() == 'exit':
                break
            client.send('OR', message)
    finally:
        client.close()

