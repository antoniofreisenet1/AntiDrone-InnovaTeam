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
        # Order : 4, 5, 7, 6, 8, 3, 2, 1
        # Connection status label
        self.statusBar = self.statusBar()
        self.updateStatusBar("Disconnected")

        self.pushButton.setText("Disconnect")
        self.pushButton_4.setText("Shoot ball")
        self.pushButton_5.setText("Start classic scan")
        self.pushButton_7.setText("Get to neutral position")
        self.pushButton_6.setText("Reset Motors (be careful)")
        self.pushButton_8.setText("Detectionless scan")

        self.pushButton_4.clicked.connect(lambda: self.sendCommand("run_full_turn"))
        self.pushButton_5.clicked.connect(lambda: self.sendCommand("scan"))
        self.pushButton_7.clicked.connect(lambda: self.sendCommand("neutral"))
        self.pushButton_6.clicked.connect(lambda: self.sendCommand("reset"))
        self.pushButton_8.clicked.connect(lambda: self.sendCommand("no_detection_scan"))
        self.pushButton.clicked.connect(self.on_disconnectButton_clicked)
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
                self.name.setText("Mindstorms EV3 Robot")
                self.IP.setText(str(addr))
                self.commandSocket = conn
                while True:
                    data = conn.recv(1024)
                    print(data)
                    if not data:
                        break
                    lines = data.decode().strip().split('\n')
                    for data_packet in lines:
                        if data_packet:  # Ensure the line is not empty
                            x, y, D = map(int, data_packet.split(','))
                            self.radarWidget.setPoint(x, y)

    def sendCommand(self, command):
        if self.commandSocket:
            self.commandSocket.sendall(f"{command}\n".encode())
            if command == "scan":
                self.updateStatusBar("Scanning/Scanned")

    def on_disconnectButton_clicked(self):
        self.sendCommand("disconnect")
        self.commandSocket.close()
        self.updateStatusBar("Disconnected")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())
