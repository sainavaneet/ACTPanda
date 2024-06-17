import sys
import rospy
import cv2
import cv_bridge
import os
import h5py
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Image
from panda_robot import PandaArm
from utils.config.config import *

class CameraController:
    def __init__(self, panda_arm):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.current_frame = None
        self.data = []
        self.joint_positions = []
        self.panda_arm = panda_arm

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.data.append(cv_image)
            # Fetch all joint angles including the gripper
            self.joint_positions.append(self.panda_arm.angles(include_gripper=True))
        except cv_bridge.CvBridgeError as e:
            print(e)

    def save_data(self):
        directory = DATASET_DIR
        if not os.path.exists(directory):
            os.makedirs(directory)
        index = 0
        while os.path.exists(os.path.join(directory, f'episode_{index}.hdf5')):
            index += 1
        file_path = os.path.join(directory, f'episode_{index}.hdf5')
        with h5py.File(file_path, 'w') as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            img_grp = obs.create_group('images')
            img_grp.create_dataset('top', data=np.array(self.data), dtype='uint8', chunks=(1, 480, 640, 3))
            obs.create_dataset('joint_positions', data=np.array(self.joint_positions))
        self.data = []
        self.joint_positions = []
        print(f"Data saved to {file_path}.")

class App(QWidget):
    def __init__(self, camera_controller):
        super().__init__()
        self.camera_controller = camera_controller
        self.initUI()

    def initUI(self):
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(640, 480)
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
        self.timer.start(30)

    def start_recording(self):
        print("<!---------Camera Started Recording----------!>")
        self.camera_controller.data = []
        self.camera_controller.joint_positions = []

    def stop_recording(self):
        print("<!---------Camera Stop Recording----------!>")
        self.camera_controller.save_data()

    def update_image(self):
        if self.camera_controller.current_frame is not None:
            height, width, channel = self.camera_controller.current_frame.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.camera_controller.current_frame.data, width, height, bytesPerLine, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qImg))

def main():
    rospy.init_node('camera_control_node', anonymous=True)
    panda_arm = PandaArm()  # Ensure PandaArm is properly initialized
    app = QApplication(sys.argv)
    camera_controller = CameraController(panda_arm)
    ex = App(camera_controller)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
