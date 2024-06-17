import sys
import rospy
import numpy as np
import random
import cv2
import cv_bridge
import os
import h5py
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Image
from panda_robot import PandaArm
from panda_kinematics import PandaWithPumpKinematics
from franka_interface import GripperInterface
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty
from constants import INITIAL_JOINTS, DATASET_DIR
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

class CameraController:
    def __init__(self, panda_arm):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.current_frame = None
        self.data = []
        self.joint_positions = []
        self.actions = []
        self.panda_arm = panda_arm
        self.recording = False
        self.gripper_state = 0  # Initialize gripper state
        self.box_positions = []

    def image_callback(self, msg):
        if self.recording:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.data.append(self.current_frame)
            joint_angles = list(self.panda_arm.angles(include_gripper=False))
            joint_angles.append(self.gripper_state)
            self.joint_positions.append(joint_angles)

    def log_action(self, action, gripper_state):
        if self.recording:
            action = action.tolist() + [gripper_state]
            self.actions.append(action)

    def start_recording(self):
        self.recording = True
        self.data = []
        self.joint_positions = []
        self.actions = []
        self.box_positions = []


    def stop_recording(self, save=True):
        self.recording = False
        if save:
            self.save_data()

    def log_box_position(self, x, y, z):
        if self.recording:
            self.box_positions.append([x, y, z])

    def save_data(self):
        if not self.data:
            print("No data to save, skipping...this episode")
            return
        episode_idx = 0
        directory = DATASET_DIR
        if not os.path.exists(directory):
            os.makedirs(directory)
        while os.path.exists(os.path.join(directory, f'episode_{episode_idx}.hdf5')):
            episode_idx += 1
        file_path = os.path.join(directory, f'episode_{episode_idx}.hdf5')
        data_dict = {
            'camera_names': ['top'],
            '/observations/images/top': self.data,
            '/observations/qpos': self.joint_positions,
            '/action': self.actions,
            '/observations/box_positions': self.box_positions
        }
        with h5py.File(file_path, 'w') as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            images = obs.create_group('images')
            for cam_name in data_dict['camera_names']:
                image_data = np.array(data_dict[f'/observations/images/{cam_name}'], dtype='uint8')
                images.create_dataset(cam_name, data=image_data, dtype='uint8', chunks=(1, 480, 640, 3))
            qpos = np.array(data_dict['/observations/qpos'], dtype='float64')
            obs.create_dataset('qpos', data=qpos)
            box_positions = np.array(data_dict['/observations/box_positions'], dtype='float64')
            obs.create_dataset('box_positions', data=box_positions)
            action_data = np.array(data_dict['/action'], dtype='float64')
            root.create_dataset('action', data=action_data)
        print(f"Data saved to {file_path}.")

class PandaRobotTask:
    def __init__(self, camera_controller, panda_arm):
        self.panda_arm = panda_arm
        self.camera_controller = camera_controller
        self.kinematics = PandaWithPumpKinematics()
        self.gripper_interface = GripperInterface()
        self.initial_positions = INITIAL_JOINTS
        self.success_count = 0
        self.new_x = 0.3
        self.new_y = 0.5
        self.set_box_position(self.new_x, self.new_y, 0.025)
        self.success_threshold = 0.05
        self.endpoint = np.array([0.6, -0.4, 0.16])
        self.operate_gripper(0)  # Ensure gripper is closed initially

        self.gripper_state = 0  # Initial gripper state

    def reset_episode(self):
        self.camera_controller.stop_recording(save=False)
        self.set_box_position(0.3 + random.uniform(0, 0.3), 0.5 + random.uniform(-0.3, 0), 0.025)
        self.panda_arm.move_to_joint_position(self.initial_positions)
        if self.success_count < 50:
            self.perform_task()

    def perform_task(self):
        while self.success_count < 50:
            self.panda_arm.move_to_joint_position(self.initial_positions)
            self.operate_gripper(0)

            self.camera_controller.start_recording()
            self.camera_controller.log_box_position(self.new_x, self.new_y, 0.025)

            pre_pick_pos, pick_pos = self.update_box_pos(self.new_x, self.new_y)
            orientation_quat = np.array([np.pi, np.pi/2, 0, 0], dtype=np.float64)
            current_joint_angles = np.array(list(self.panda_arm.joint_angles().values()), dtype=np.float64)
            solution = self.solve_kinematics(current_joint_angles, pre_pick_pos, orientation_quat)

            if solution is None:
                self.reset_episode()
                continue

            self.move(solution, 1)
            self.operate_gripper(1)
            rospy.sleep(1)

            solution = self.solve_kinematics(current_joint_angles, pick_pos, orientation_quat)
            if solution is None:
                self.reset_episode()
                continue

            self.move(solution, 0)
            self.panda_arm.exec_gripper_cmd(0.028 , 0.1)



            self.place_in_basket()
            self.gripper_state = 0

            self.camera_controller.stop_recording(save=False)
            if self.check_success():
                print("Episode successful: Box is successfully placed.")
                self.camera_controller.save_data()
                self.success_count += 1
            else:
                print("Episode failed: Not recording the episode")

            self.new_x, self.new_y = 0.3 + random.uniform(0, 0.3), 0.5 + random.uniform(-0.3, 0)
            self.set_box_position(self.new_x, self.new_y, 0.025)
            self.panda_arm.move_to_joint_position(self.initial_positions)

    def set_box_position(self, x, y, z):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = SetModelStateRequest()
        state_msg.model_state.model_name = 'block_red_2'
        state_msg.model_state.pose.position.x = x
        state_msg.model_state.pose.position.y = y
        state_msg.model_state.pose.position.z = z
        set_state(state_msg)

    def update_box_pos(self, new_x, new_y):
        z = 0.025
        pre_pick_pos = np.array([new_x, new_y, z + 0.045], dtype=np.float64)
        pick_pos = np.array([new_x, new_y, z - 0.003], dtype=np.float64)
        return pre_pick_pos, pick_pos

    def solve_kinematics(self, joint_positions, position, quat):
        return self.kinematics.ik(joint_positions, position, quat)

    def move(self, solution, gripper_state):
        if solution is not None:
            self.camera_controller.log_action(solution, gripper_state)
            self.panda_arm.move_to_joint_position(solution.tolist())

    def get_gripper_width(self):
        joint_angles = list(self.panda_arm.angles(include_gripper=True))
        gripper_width = abs(joint_angles[-1] - joint_angles[-2])

        return gripper_width
    

    def operate_gripper(self, action):
        if action == 1:
            self.gripper_interface.open()
            self.get_gripper_width()
            
        elif action == 0:
            self.gripper_interface.close()

    def check_success(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state_req = GetModelStateRequest(model_name='block_red_2')
        state_res = get_state(state_req)
        box_pos = np.array([state_res.pose.position.x, state_res.pose.position.y, state_res.pose.position.z])
        return np.linalg.norm(self.endpoint - box_pos) < self.success_threshold

    def place_in_basket(self):
        joint_positions = np.zeros(7)
        pos = np.array([0.6, -0.4, 0.4])
        ori = np.array([np.pi, np.pi/2, 0.0, 0.0])
        solution = self.solve_kinematics(joint_positions, pos, ori)
        self.move(solution , 0)

       
        pos2 = np.array([0.6, -0.4, 0.16])
        solution = self.solve_kinematics(joint_positions, pos2, ori)
        rospy.sleep(1)
        self.move(solution, 1)
        self.operate_gripper(1)

def main():
    rospy.init_node('camera_control_node', anonymous=True)
    panda_arm = PandaArm()
    camera_controller = CameraController(panda_arm)
    robot_task = PandaRobotTask(camera_controller, panda_arm)
    robot_task.perform_task()

if __name__ == '__main__':
    main()

