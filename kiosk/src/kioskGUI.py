import os
import sys

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import QCursor
from PyQt5.QtGui import QPixmap

import socket
import time

# image data
import resource_qrc 
import resource_order_qrc
import resource_topping_qrc
import resource_pay_qrc
from tcp import TCPClient

HOST = '172.30.1.60' # bosun ip
PORT = 9012

global ORDER_NO
ORDER_NO = 0


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main_window = uic.loadUi('./data/main.ui', self)
        self.main_window.show()

        self.tcp_server = TCPClient(HOST, PORT) #tcp 선언

        self.main_window.orderBtn.setCursor(QCursor(Qt.PointingHandCursor)) # 버튼으로 마우스 향했을때 클릭가능함을 알리기 위함(커서가 바뀜)
        self.main_window.orderBtn.clicked.connect(self.go_order) # 버튼클릭시 실행되는 함수
        

    def go_order(self):
        print("Label clicked!")
        flavor_window = FlavorWindow(self.tcp_server) # flavorwindow 선언
        flavor_window.show()
        self.main_window.hide()


class FlavorWindow(QMainWindow):
    def __init__(self, tcp_server):
        super().__init__()
        self.flavor_window = uic.loadUi('./data/flavor.ui', self)
        self.selected_flavor = None
        self.tcp_server = tcp_server
        
        self.flavor_window.chocoBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.flavor_window.strawberryBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.flavor_window.mintBtn.setCursor(QCursor(Qt.PointingHandCursor))

        self.flavor_window.chocoBtn.clicked.connect(lambda: self.go_topping('choco')) #버튼 클릭 이벤트와 어떤 버튼을 눌렀는지 연결하는 함수
        self.flavor_window.strawberryBtn.clicked.connect(lambda: self.go_topping('strawberry'))
        self.flavor_window.mintBtn.clicked.connect(lambda: self.go_topping('mint'))
    
    def go_topping(self, flavor):
        print(f"selected flavor = {flavor}")
        topping_window = ToppingWindow(self.tcp_server,flavor) # 맛에 따라 toppingwindow에서 띄우는게 다름
        topping_window.show()
        self.flavor_window.hide()


class ToppingWindow(QMainWindow):
    def __init__(self, tcp_server, flavor):
        super().__init__()
        self.flavor = flavor
        self.list_topping = [] 
        self.tcp_server = tcp_server

        self.topping_window = uic.loadUi('./data/topping.ui', self)

        self.topping_window.oreoBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.topping_window.chocoballBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.topping_window.cerialBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.topping_window.infoBtn.setCursor(QCursor(Qt.PointingHandCursor))

        self.topping_window.choco.hide()
        self.topping_window.strawberry.hide()
        self.topping_window.mint.hide()
        self.topping_window.oreotopping.hide()
        self.topping_window.chocotopping.hide()
        self.topping_window.cerialtopping.hide()

        self.topping_window.oreoBtn.clicked.connect(lambda: self.toggle_topping("oreo", self.topping_window.oreotopping)) # 어떤 토핑을 눌렀는지 전달 
        self.topping_window.chocoballBtn.clicked.connect(lambda: self.toggle_topping("chocoball", self.topping_window.chocotopping))
        self.topping_window.cerialBtn.clicked.connect(lambda: self.toggle_topping("cerial", self.topping_window.cerialtopping))
        self.topping_window.infoBtn.clicked.connect(self.go_info)


        # 맛에 따라 버튼 표시
        if self.flavor == "choco":
            self.topping_window.choco.show()
        elif self.flavor == "strawberry":
            self.topping_window.strawberry.show()   
        elif self.flavor == "mint":
            self.topping_window.mint.show()

    def toggle_topping(self, topping, label_widget): # 버튼을 누를때마다 토핑이 없으면 추가 있으면 제거 
        if label_widget.isHidden():
            label_widget.show()
            self.list_topping.append(topping)
        else:
            label_widget.hide()
            self.list_topping.remove(topping)

        print("Selected topping:", self.list_topping)

    def go_info(self):  #선택한 맛, 토핑을 전달
        info_window = InfoWindow(self.tcp_server, self.flavor, self.list_topping)
        info_window.show()
        self.topping_window.hide()
        
class InfoWindow(QMainWindow): 
    def __init__(self, tcp_server, taste, list_topping):
        super().__init__()
        self.tcp_server = tcp_server
        self.taste = taste
        self.list_topping = list_topping
        global ORDER_NO
        ORDER_NO += 1  # 주문 번호 증가

        self.info_window = uic.loadUi('./data/info.ui', self)
        self.tcp_server.robot_status_callback(self.handle_tcp_response)

        self.info_window.cup.hide()
        self.info_window.choco.hide()
        self.info_window.strawberry.hide()
        self.info_window.mint.hide()
        self.info_window.turkey.hide()
        self.info_window.oreotopping.hide()
        self.info_window.chocotopping.hide()
        self.info_window.cerialtopping.hide()

        self.info_window.orderno.setText(f"Order No: {ORDER_NO}")
        self.info_window.flavor.setText(f"Flavor: {self.taste}" )
        self.info_window.topping.setText(f"Topping: {', '.join(self.list_topping)}")

        self.send_data()
    
    def send_data(self):
        print("send_data called")
        cmd = "OR"
        flavor_index = 0
        if self.taste == "choco":
            flavor_index = 1
        elif self.taste == "strawberry":
            flavor_index = 2
        elif self.taste == "mint":
            flavor_index = 3

        topping_indices = [0, 0, 0]  # Initialize topping indices
        if "cerial" in self.list_topping:
            topping_indices[0] = 1
        if "oreo" in self.list_topping:
            topping_indices[1] = 1
        if "chocoball" in self.list_topping:
            topping_indices[2] = 1
        
        data = f"O-{ORDER_NO}", f"F-{flavor_index}", f"T-{topping_indices[0]}{topping_indices[1]}{topping_indices[2]}"
        print(f"Sending data: {cmd},{data}")
        self.tcp_server.send(cmd, data)

        

    def handle_tcp_response(self, response): # tcp로부터 받은 데이터 처리 -> tcp.py로 가면 자세히 확인가능
        if response == "0":
            pass

        elif response == "1":
            self.info_window.received.hide()
            self.set_cup()

        elif response == "2":
            self.set_flavor()

        elif response == "3":
            self.set_topping()

        elif response == "4":
            self.set_turkey()

        elif response == "5":
            self.restart()
    
    def set_cup(self):
        self.info_window.cup.show()
        
    def set_flavor(self):
        self.info_window.cup.hide()
        if self.taste == "choco":
            self.info_window.choco.show()

        elif self.taste == "strawberry":
            self.info_window.strawberry.show()   

        elif self.taste == "mint":
            self.info_window.mint.show()
        
    def set_topping(self):
        # Show toppings based on the list
        for topping in self.list_topping:
            if topping == "oreo":
                self.info_window.oreotopping.show()

            elif topping == "chocoball":
                self.info_window.chocotopping.show()

            elif topping == "cerial":
                self.info_window.cerialtopping.show()
    
    def set_turkey(self):
        self.info_window.turkey.show()

    def restart(self): # 주문이 끝나고 주문 다시 시작하기 위함 (tcp는 다시 연결안하게 tcp만 전달하면 될듯)
        pass
                    


        


            

            
    




        



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
