import os
import sys

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import QCursor, QFont

import resources_test_rc


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main_window = uic.loadUi('/home/bo/git_ws/bosun_repo/kiosk/test.ui', self)
        self.main_window.show()
        self.main_window.showFullScreen()

        # self.main_window.orderBtn.setCursor(QCursor(Qt.PointingHandCursor)) # 버튼으로 마우스 향했을때 클릭가능함을 알리기 위함(커서가 바뀜)
        # self.main_window.orderBtn.clicked.connect(self.go_order) # 버튼클릭시 실행되는 함수
        


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())