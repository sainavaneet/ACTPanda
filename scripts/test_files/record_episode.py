import sys
import rospy
import cv2
import cv_bridge
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Image
import h5py
import numpy as np

class CameraController:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = None  # Start with no subscriber
        self.current_frame = None
        self.data = []
        self.start_camera()  # Start camera on initialization

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
            self.data.append(cv_image)
        except cv_bridge.CvBridgeError as e:
            print(e)

    def start_camera(self):
        if self.subscriber is None:
            self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
            print("Camera started.")

    def stop_camera(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
            print("Camera stopped.")

    def save_data(self, directory='/home/navaneet/Desktop/GITHUB/act_franka/dataset'):
        # Ensure the directory exists
        directory = os.path.expanduser(directory)
        if not os.path.exists(directory):
            os.makedirs(directory)
        
        # Find the next available file name
        index = 0
        while os.path.exists(os.path.join(directory, f'episode_{index}.hdf5')):
            index += 1
        filename = os.path.join(directory, f'episode_{index}.hdf5')

        with h5py.File(filename, 'w') as f:
            f.create_dataset('images', data=np.array(self.data), compression="gzip")
        self.data = []  # Clear data after saving
        print(f"Data saved to {filename}.")

class App(QWidget):
    def __init__(self, camera_controller):
        super().__init__()
        self.camera_controller = camera_controller
        self.initUI()

    def initUI(self):
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(640, 480)  # Adjust to your camera resolution

        start_button = QPushButton('Start Recording', self)
        start_button.clicked.connect(self.start_recording)

        stop_button = QPushButton('Stop Recording', self)
        stop_button.clicked.connect(self.stop_recording)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(start_button)
        layout.addWidget(stop_button)

        self.setLayout(layout)
        self.setWindowTitle('Camera Control')
        self.show()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30)  # Refresh rate in milliseconds

    def start_recording(self):
        self.camera_controller.data = []  # Reset data to start fresh

    def stop_recording(self):
        self.camera_controller.stop_camera()
        self.camera_controller.save_data()
        self.camera_controller.start_camera()  # Restart camera after saving

    def update_image(self):
        if self.camera_controller.current_frame is not None:
            height, width, channel = self.camera_controller.current_frame.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.camera_controller.current_frame.data, width, height, bytesPerLine, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qImg))

def main():
    rospy.init_node('camera_control_node', anonymous=True)
    app = QApplication(sys.argv)
    camera_controller = CameraController()
    ex = App(camera_controller)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
