import sys
import socket
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt

# Import the generated UI class
from ui_main_window import Ui_MainWindow


class RadarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.x = 0
        self.y = 0

    def setPoint(self, x, y):
        self.x = x
        self.y = y
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        width = self.width()
        height = self.height()
        painter.translate(width / 2, height / 2)

        # Draw Cartesian axes with outlines
        self.drawCartesianAxes(painter, width, height)

        # Draw the blue, rounded point
        painter.setPen(QPen(Qt.GlobalColor.blue, 1))
        painter.setBrush(QBrush(Qt.GlobalColor.blue))
        pointSize = 10
        painter.drawEllipse(self.x - pointSize / 2, self.y - pointSize / 2, pointSize, pointSize)

    def drawCartesianAxes(self, painter, width, height):
        painter.setPen(QPen(Qt.GlobalColor.black, 1))
        painter.drawLine(-width / 2, 0, width / 2, 0)
        painter.drawLine(0, -height / 2, 0, height / 2)
        # Draw axis labels
        for step in range(-width // 2, width // 2, 40):
            painter.drawText(step, -10, str(step))
        for step in range(-height // 2, height // 2, 40):
            if step != 0:  # Skip the center
                painter.drawText(10, step, str(step))


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.commandSocket = None
        self.setupUi(self)
        self.setWindowTitle("Radar Movement Simulation")

        # Connection status label
        self.statusBar = self.statusBar()
        self.updateStatusBar("Disconnected")

        self.pushButton_4.clicked.connect(lambda: self.sendCommand("run_full_turn"))
        self.pushButton_5.clicked.connect(lambda: self.sendCommand("run_full_turn"))
        # Additional buttons can be added here

        self.show()

        # Start listening for robot data
        threading.Thread(target=self.listen_for_robot_data, daemon=True).start()

    def updateStatusBar(self, message):
        self.statusBar.showMessage(message)

    def listen_for_robot_data(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('0.0.0.0', 8000))
            s.listen()
            conn, addr = s.accept()
            with conn:
                self.updateStatusBar(f"Connected to {addr}")
                self.commandSocket = conn
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    x, y = map(int, data.decode().strip().split(','))
                    self.radarWidget.setPoint(x, y)

    def sendCommand(self, command):
        if self.commandSocket:
            self.commandSocket.sendall(f"{command}\n".encode())


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())
