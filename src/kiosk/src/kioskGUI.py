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
import resource_serve_qrc
import resource_table_qrc
from tcp import TCPClient
from dotenv import load_dotenv

# HOST = '192.168.1.9' # robot ip
# HOST = '192.168.1.7' # wono ip
# HOST = '172.30.1.51'  # xyz 5g ip

load_dotenv()
HOST = os.getenv('HOST_IP')

# PORT = 9005
PORT = 9003


def get_ui_path(relative_path):
    """프로젝트의 base 디렉토리를 기준으로 상대 경로를 반환"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, '../data/ui', relative_path)


class Order():
    def __init__(self):
        self.icecream = None
        self.toppings = None
        self.table = None


class MainWindow(QMainWindow):
    def __init__(self, tcp_server):
        super().__init__()
        self.tcp_server = tcp_server
        self.init_ui()

    def init_ui(self):
        self.main_window = uic.loadUi(get_ui_path('main.ui'), self)
        self.main_window.show()

        self.main_window.orderBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.main_window.orderBtn.clicked.connect(self.go_order)

    def go_order(self):
        print("Label clicked!")
        self.serving_window = ServingWindow(self.tcp_server)
        self.serving_window.show()
        self.main_window.hide()

    def restart(self):
        self.main_window.show()


class ServingWindow(QMainWindow):
    def __init__(self, tcp_server):
        super().__init__()
        self.tcp_server = tcp_server
        self.order = Order()
        self.init_ui()

    def init_ui(self):
        self.serving_window = uic.loadUi(get_ui_path('serve.ui'), self)
        self.serving_window.show()

        self.serving_window.eat_here_Btn.setCursor(
            QCursor(Qt.PointingHandCursor))
        self.serving_window.takeout_Btn.setCursor(
            QCursor(Qt.PointingHandCursor))

        self.serving_window.eat_here_Btn.clicked.connect(self.go_table)
        self.serving_window.takeout_Btn.clicked.connect(
            self.go_flavor)

    def go_table(self):
        print("eat here clicked")
        self.TableWindow = TableWindow(self.tcp_server, self.order)
        self.TableWindow.show()
        self.serving_window.hide()

    def go_flavor(self):
        print("take out clicked")
        self.order.table = 0
        self.FlavorWindow = FlavorWindow(self.tcp_server, self.order)
        self.FlavorWindow.show()
        self.serving_window.hide()


class TableWindow(QMainWindow):
    def __init__(self, tcp_server, order):
        super().__init__()
        self.tcp_server = tcp_server
        self.order = order
        self.init_ui()

    def init_ui(self):
        self.table_window = uic.loadUi(get_ui_path('table.ui'), self)
        self.table_window.show()

        self.table_window.Table1_Btn.setCursor(QCursor(Qt.PointingHandCursor))
        self.table_window.Table2_Btn.setCursor(QCursor(Qt.PointingHandCursor))
        self.table_window.Table3_Btn.setCursor(QCursor(Qt.PointingHandCursor))

        self.table_window.Table1_Btn.clicked.connect(
            lambda: self.select_table(1))
        self.table_window.Table2_Btn.clicked.connect(
            lambda: self.select_table(2))
        self.table_window.Table3_Btn.clicked.connect(
            lambda: self.select_table(3))

    def select_table(self, table_number):
        self.order.table = table_number
        print(f"Selected table: {table_number}")
        self.go_flavor()

    def go_flavor(self):
        print("Navigating to flavor selection!")
        self.FlavorWindow = FlavorWindow(self.tcp_server, self.order)
        self.FlavorWindow.show()
        self.table_window.hide()


class FlavorWindow(QMainWindow):
    def __init__(self, tcp_server, order):
        super().__init__()
        self.tcp_server = tcp_server
        self.order = order
        self.init_ui()

    def init_ui(self):
        self.flavor_window = uic.loadUi(get_ui_path('flavor.ui'), self)
        self.flavor_window.chocoBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.flavor_window.strawberryBtn.setCursor(
            QCursor(Qt.PointingHandCursor))
        self.flavor_window.mintBtn.setCursor(QCursor(Qt.PointingHandCursor))

        self.flavor_window.chocoBtn.clicked.connect(
            lambda: self.go_topping('choco'))
        self.flavor_window.strawberryBtn.clicked.connect(
            lambda: self.go_topping('strawberry'))
        self.flavor_window.mintBtn.clicked.connect(
            lambda: self.go_topping('mint'))

    def go_topping(self, flavor):
        print(f"Selected flavor: {flavor}")
        self.order.icecream = flavor
        self.topping_window = ToppingWindow(
            self.tcp_server, self.order)  # 테이블 번호도 전달
        self.topping_window.show()
        self.flavor_window.hide()


class ToppingWindow(QMainWindow):
    def __init__(self, tcp_server, order):
        super().__init__()
        self.tcp_server = tcp_server
        self.order = order
        self.list_topping = []
        self.init_ui()

    def init_ui(self):
        self.topping_window = uic.loadUi(get_ui_path('topping.ui'), self)
        self.topping_window.oreoBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.topping_window.chocoballBtn.setCursor(
            QCursor(Qt.PointingHandCursor))
        self.topping_window.cerialBtn.setCursor(QCursor(Qt.PointingHandCursor))
        self.topping_window.infoBtn.setCursor(QCursor(Qt.PointingHandCursor))

        self.topping_window.choco.hide()
        self.topping_window.strawberry.hide()
        self.topping_window.mint.hide()
        self.topping_window.oreotopping.hide()
        self.topping_window.chocotopping.hide()
        self.topping_window.cerialtopping.hide()

        self.topping_window.oreoBtn.clicked.connect(
            lambda: self.toggle_topping("oreo", self.topping_window.oreotopping))
        self.topping_window.chocoballBtn.clicked.connect(
            lambda: self.toggle_topping("chocoball", self.topping_window.chocotopping))
        self.topping_window.cerialBtn.clicked.connect(
            lambda: self.toggle_topping("cereal", self.topping_window.cerialtopping))
        self.topping_window.infoBtn.clicked.connect(self.go_info)

        if self.order.icecream == "choco":
            self.topping_window.choco.show()
        elif self.order.icecream == "strawberry":
            self.topping_window.strawberry.show()
        elif self.order.icecream == "mint":
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
        # InfoWindow 초기화 시 table_number를 전달합니다.
        self.order.toppings = self.list_topping
        self.info_window = InfoWindow(
            self.tcp_server, self.order)
        self.info_window.show()
        self.topping_window.hide()


class InfoWindow(QMainWindow):
    def __init__(self, tcp_server, order):
        super().__init__()
        self.tcp_server = tcp_server
        self.order = order
        self.init_ui()

    def init_ui(self):
        self.info_window = uic.loadUi(get_ui_path('info.ui'), self)
        self.tcp_server.order_call_callback(self.handle_tcp_response)
        self.tcp_server.order_status_callback(self.handle_tcp_response)

        self.info_window.cup.hide()
        self.info_window.choco.hide()
        self.info_window.strawberry.hide()
        self.info_window.mint.hide()
        self.info_window.turkey.hide()
        self.info_window.oreotopping.hide()
        self.info_window.chocotopping.hide()
        self.info_window.cerialtopping.hide()

        self.info_window.orderno.setText(f"Order No: None")
        self.info_window.flavor.setText(f"Flavor: {self.order.icecream}")
        self.info_window.topping.setText(f"Topping: {', '.join(self.order.toppings)}")
        if self.order.table == 0:
            self.info_window.tableno.setText("Takeout")
        else:
            self.info_window.tableno.setText(f"Table No: {self.order.table}")  # 테이블 번호 표시

        self.send_data()

    def send_data(self):
        print("send_data called")
        
        order_data = {"OR": {"icecream": self.order.icecream, 
                             "topping": ','.join(self.order.toppings), 
                             "table": self.order.table}}
        json_order_data = json.dumps(order_data) + "\n" # 구분자 추가
        print(f"Sending data: {json_order_data}")
        self.tcp_server.send(json_order_data)

    def handle_tcp_response(self, response):
        cmd, data = response.split(',', 1)
        print(f"cmd: {cmd}, data:{data}")
        if cmd == "OR":
            self.orderId = data.strip()
            self.info_window.orderno.setText(f"Order No : {self.orderId}")
            self.info_window.ordercheck.hide()
        elif cmd == "TR":
            pass
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
        else:
            print("invalid cmd")

    def set_cup(self):
        self.info_window.cup.show()

    def set_flavor(self):
        self.info_window.cup.hide()
        if self.order.icecream == "choco":
            self.info_window.choco.show()
        elif self.order.icecream == "strawberry":
            self.info_window.strawberry.show()
        elif self.order.icecream == "mint":
            self.info_window.mint.show()

    def set_topping(self):
        for topping in self.order.toppings:
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
