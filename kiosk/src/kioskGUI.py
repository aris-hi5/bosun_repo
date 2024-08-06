import os
import sys

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import QCursor, QFont

import time
import json

# image data
import resource_qrc 
import resource_order_qrc
import resource_topping_qrc
import resource_pay_qrc
from tcp import TCPClient

# HOST = '192.168.1.9' # robot ip
# HOST = '192.168.1.7' # wono ip
HOST = '172.30.1.83' # xyz 5g ip

# PORT = 9005
PORT = 9003

class MainWindow(QMainWindow):
    def __init__(self, tcp_server):
        super().__init__()
        self.tcp_server = tcp_server
        self.init_ui()

    def init_ui(self):
        self.main_window = uic.loadUi('/home/bo/git_ws/bosun_repo/kiosk/data/main.ui', self)
        self.main_window.show()

        self.main_window.orderBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.main_window.orderBtn.clicked.connect(self.go_order)

    def go_order(self):
        print("Label clicked!")
        self.flavor_window = FlavorWindow(self.tcp_server)
        self.flavor_window.show()
        self.main_window.hide()

    def restart(self):
        self.main_window.show()

class FlavorWindow(QMainWindow):
    def __init__(self, tcp_server):
        super().__init__()
        self.tcp_server = tcp_server
        self.init_ui()

    def init_ui(self):
        self.flavor_window = uic.loadUi('/home/bo/git_ws/bosun_repo/kiosk/data/flavor.ui', self)
        self.flavor_window.chocoBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.flavor_window.strawberryBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.flavor_window.mintBtn.setCursor(QCursor(Qt.PointingHandCursor))

        self.flavor_window.chocoBtn.clicked.connect(lambda: self.go_topping('choco'))
        self.flavor_window.strawberryBtn.clicked.connect(lambda: self.go_topping('strawberry'))
        self.flavor_window.mintBtn.clicked.connect(lambda: self.go_topping('mint'))

    def go_topping(self, flavor):
        print(f"selected flavor = {flavor}")
        self.topping_window = ToppingWindow(self.tcp_server, flavor)
        self.topping_window.show()
        self.flavor_window.hide()

class ToppingWindow(QMainWindow):
    def __init__(self, tcp_server, flavor):
        super().__init__()
        self.tcp_server = tcp_server
        self.flavor = flavor
        self.list_topping = []
        self.init_ui()

    def init_ui(self):
        self.topping_window = uic.loadUi('/home/bo/git_ws/bosun_repo/kiosk/data/topping.ui', self)
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

        self.topping_window.oreoBtn.clicked.connect(lambda: self.toggle_topping("oreo", self.topping_window.oreotopping))
        self.topping_window.chocoballBtn.clicked.connect(lambda: self.toggle_topping("chocoball", self.topping_window.chocotopping))
        self.topping_window.cerialBtn.clicked.connect(lambda: self.toggle_topping("cereal", self.topping_window.cerialtopping))
        self.topping_window.infoBtn.clicked.connect(self.go_info)

        if self.flavor == "choco":
            self.topping_window.choco.show()
        elif self.flavor == "strawberry":
            self.topping_window.strawberry.show()   
        elif self.flavor == "mint":
            self.topping_window.mint.show()

    def toggle_topping(self, topping, label_widget):
        if label_widget.isHidden():
            label_widget.show()
            self.list_topping.append(topping)
        else:
            label_widget.hide()
            self.list_topping.remove(topping)
        print("Selected topping:", self.list_topping)

    def go_info(self):
        self.info_window = InfoWindow(self.tcp_server, self.flavor, self.list_topping)
        self.info_window.show()
        self.topping_window.hide()

class InfoWindow(QMainWindow):
    def __init__(self, tcp_server, taste, list_topping):
        super().__init__()
        self.tcp_server = tcp_server
        self.taste = taste
        self.list_topping = list_topping
        self.init_ui()

    def init_ui(self):
        self.info_window = uic.loadUi('/home/bo/git_ws/bosun_repo/kiosk/data/info.ui', self)
        self.tcp_server.order_call_callback(self.handle_tcp_response)
        self.tcp_server.order_status_callback(self.handle_tcp_response)

        # self.info_window.received.hide()
        self.info_window.cup.hide()
        self.info_window.choco.hide()
        self.info_window.strawberry.hide()
        self.info_window.mint.hide()
        self.info_window.turkey.hide()
        self.info_window.oreotopping.hide()
        self.info_window.chocotopping.hide()
        self.info_window.cerialtopping.hide()

        self.info_window.orderno.setText(f"Order No: None")
        self.info_window.flavor.setText(f"Flavor: {self.taste}")
        self.info_window.topping.setText(f"Topping: {','.join(self.list_topping)}")

        self.send_data()

    def send_data(self):
        print("send_data called")
        data = {"OR": {"icecream": self.taste, "topping": ','.join(self.list_topping)}}
        json_data = json.dumps(data)
        print(f"Sending data: {json_data}")
        self.tcp_server.send(json_data)

    def handle_tcp_response(self, response):
        print(response)
        cmd, data = response.split(',', 1)
        print(f"cmd: {cmd}, data:{data}")
        if cmd == "OR":
            self.orderId = data.strip()
            self.info_window.orderno.setText(f"Order No: 004")
            self.info_window.ordercheck.hide()
        elif cmd == "OS":
            data = data.strip()
            if data == "0":
                pass
            elif data == "1":
                self.set_cup()
            elif data == "2":
                self.set_flavor()
            elif data == "3":
                self.set_topping()
            elif data == "4":
                self.set_turkey()
            elif data == "5":
                self.restart()
                print("이용자 수령완료")
            else:
                print("data error")
        elif "ER" in response:
            print(response)
            self.info_window.ordercheck.setText(f"{data}")
            font = QFont()
            font.setPointSize(15)
            # self.info_window.received.setFont(font)
        else:
            print("invalid cmd")

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
        for topping in self.list_topping:
            if topping == "oreo":
                self.info_window.oreotopping.show()
            elif topping == "chocoball":
                self.info_window.chocotopping.show()
            elif topping == "cereal":
                self.info_window.cerialtopping.show()

    def set_turkey(self):
        self.info_window.turkey.show()

    def restart(self):
        print("restart")
        self.info_window.hide()
        main_window.restart()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    tcp_server = TCPClient(HOST, PORT)
    main_window = MainWindow(tcp_server)  # 전역 변수로 선언
    main_window.show()
    sys.exit(app.exec_())
