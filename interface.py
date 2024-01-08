from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QDialog, QApplication
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtWidgets
import cv2

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1171, 531)
        self.horizontalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(560, 130, 411, 151))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.search = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.search.setObjectName("search")
        self.horizontalLayout.addWidget(self.search)
        self.fire = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.fire.setObjectName("fire")
        self.horizontalLayout.addWidget(self.fire)
        self.cameraLabel = QtWidgets.QGraphicsView(Dialog)
        self.cameraLabel.setGeometry(QtCore.QRect(600, 300, 256, 192))
        self.cameraLabel.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.cameraLabel.setAcceptDrops(True)
        self.cameraLabel.setObjectName("cameraLabel")
        self.video_capture = cv2.VideoCapture(0)  # Replace '0' with your camera index or video file path
        self.video_timer = QtCore.QTimer(Dialog)
        self.video_timer.timeout.connect(self.updateVideoFeed)
        self.video_timer.start(30)
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(572, 20, 391, 89))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.status = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.status.setObjectName("status")
        self.horizontalLayout_2.addWidget(self.status)
        self.searching = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.searching.setObjectName("searching")
        self.horizontalLayout_2.addWidget(self.searching)
        self.target = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.target.setObjectName("target")
        self.horizontalLayout_2.addWidget(self.target)
        self.shooting = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.shooting.setObjectName("shooting")
        self.horizontalLayout_2.addWidget(self.shooting)
        self.verticalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(80, 330, 201, 181))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.distance = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.distance.setObjectName("distance")
        self.horizontalLayout_6.addWidget(self.distance)
        self.dist_numb = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.dist_numb.setObjectName("dist_numb")
        self.horizontalLayout_6.addWidget(self.dist_numb)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.elevation = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.elevation.setObjectName("elevation")
        self.horizontalLayout_5.addWidget(self.elevation)
        self.elev_numb = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.elev_numb.setObjectName("elev_numb")
        self.horizontalLayout_5.addWidget(self.elev_numb)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.orientation = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.orientation.setObjectName("orientation")
        self.horizontalLayout_3.addWidget(self.orientation)
        self.orient_numb = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.orient_numb.setObjectName("orient_numb")
        self.horizontalLayout_3.addWidget(self.orient_numb)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.switchButton = QtWidgets.QRadioButton(Dialog)
        self.switchButton.setGeometry(QtCore.QRect(60, 110, 95, 20))
        self.switchButton.setObjectName("switchButton")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def updateVideoFeed(self):
        ret, frame = self.video_capture.read()
        if ret:
            # Convert OpenCV frame to QImage
            height, width, channel = frame.shape
            bytesPerLine = 3 * width
            qImg = QImage(frame.data, width, height, bytesPerLine, QImage.Format_RGB888)

            # Convert QImage to QPixmap
            pixmap = QPixmap.fromImage(qImg)

            # Display the QPixmap in the QGraphicsView
            scene = QGraphicsScene()
            pixy_item = QGraphicsPixmapItem(pixmap)
            scene.addItem(pixy_item)
            self.cameraLabel.setScene(scene)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.search.setText(_translate("Dialog", "Search"))
        self.fire.setText(_translate("Dialog", "Fire"))
        self.status.setText(_translate("Dialog", "Status"))
        self.searching.setText(_translate("Dialog", "Searching"))
        self.target.setText(_translate("Dialog", "Target"))
        self.shooting.setText(_translate("Dialog", "Shooting"))
        self.distance.setText(_translate("Dialog", "Distance"))
        self.dist_numb.setText(_translate("Dialog", "TextLabel"))
        self.elevation.setText(_translate("Dialog", "Elevation"))
        self.elev_numb.setText(_translate("Dialog", "TextLabel"))
        self.orientation.setText(_translate("Dialog", "Orientation"))
        self.orient_numb.setText(_translate("Dialog", "TextLabel"))
        self.switchButton.setText(_translate("Dialog", "RadioButton"))

    def closeEvent(self, event):
        # Release the video capture when closing the application
        self.video_capture.release()


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
