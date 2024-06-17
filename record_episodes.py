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
from utils.config.config import *
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from threading import Lock

import tqdm

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
        self.gripper_width = OPEN_GRIPPER_POSE
        self.box_positions = []

    def image_callback(self, msg):
        if self.recording: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.data.append(self.current_frame)
            joint_angles = list(self.panda_arm.angles(include_gripper=False))
            joint_angles.append(self.gripper_width)  
            self.joint_positions.append(joint_angles)

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

    def log_failed_box_positions(self, x, y, z):
        # Open a file in append mode
        with open('failed_positions.txt', 'a') as file:
            # Write the positions to the file
            file.write(f'Failed Position - X: {x}, Y: {y}, Z: {z}\n')

    def save_data(self):
        target_length = 390  

        if not self.data:
            print("No data to save, skipping... this episode")
            return

        # Truncate or pad data to match the target_length
        self.data = (self.data[:target_length] if len(self.data) > target_length
                     else self.data + [self.data[-1]] * (target_length - len(self.data)))
        self.joint_positions = (self.joint_positions[:target_length] if len(self.joint_positions) > target_length
                                else self.joint_positions + [self.joint_positions[-1]] * (target_length - len(self.joint_positions)))
        # Save the episode data
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
            '/action': self.joint_positions,
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
            print("="*50)
            print(f"Image data shape: {image_data.shape}")
            print(f"Joint positions shape: {qpos.shape}")
            print(f"Box positions shape: {box_positions.shape}")
            print(f"Action data shape: {action_data.shape}")
        print(f"Data saved to {file_path}.")
        print("="*50)
        

class PandaRobotTask:
    def __init__(self, camera_controller, panda_arm):
        self.panda_arm = panda_arm
        self.camera_controller = camera_controller
        self.kinematics = PandaWithPumpKinematics()
        self.gripper_interface = GripperInterface()
        self.initial_positions = INITIAL_JOINTS
        self.success_count = 0
        self.new_x = 0.5
        self.new_y = 0.5
        self.set_box_position(self.new_x, self.new_y, BOX_Z)
        self.success_threshold = OPEN_GRIPPER_POSE
        self.endpoint = np.array([0.6, -0.4, 0.16])
        self.operate_gripper(GRASP, GRIPPER_FORCE)  # Ensure gripper is OPEN_GRIPPER_POSE initially
        self.gripper_width = GRASP  # Initial gripper width

    def reset_episode(self):
        self.camera_controller.stop_recording(save=False)
        self.new_x, self.new_y = 0.3 + random.uniform(0, 0.2), 0.5 + random.uniform(-0.2, 0)
        self.panda_arm.move_to_joint_position(self.initial_positions)
        if self.success_count < TOTAL_EPISODES:
            self.perform_task()

    def perform_task(self):
        while self.success_count < TOTAL_EPISODES:
            self.panda_arm.move_to_joint_position(self.initial_positions)
            self.operate_gripper(INITAL_GRIPPER_POSE, GRIPPER_FORCE)  # OPEN_GRIPPER_POSE gripper

            self.camera_controller.start_recording()
            self.camera_controller.log_box_position(self.new_x, self.new_y, BOX_Z)

            pre_pick_pos, pick_pos = self.update_box_pos(self.new_x, self.new_y)
            orientation_quat = np.array([np.pi, np.pi/2, 0, 0], dtype=np.float64)
            current_joint_angles = np.array(list(self.panda_arm.joint_angles().values()), dtype=np.float64)
            solution = self.solve_kinematics(current_joint_angles, pre_pick_pos, orientation_quat)

            if solution is None:
                self.reset_episode()
                continue

            self.move(solution)
            self.operate_gripper(OPEN_GRIPPER_POSE, GRIPPER_FORCE)  # Ensure gripper is OPEN_GRIPPER_POSE initially



            solution = self.solve_kinematics(current_joint_angles, pick_pos, orientation_quat)
            if solution is None:
                self.reset_episode()
                continue

            self.move(solution)
            self.operate_gripper(GRASP, GRIPPER_FORCE)  # Close gripper with specified width and GRIPPER_FORCE

            self.place_in_basket()

            self.camera_controller.stop_recording(save=False)
            if self.check_success():
                print("Episode successful: Box is successfully placed.")
                self.camera_controller.save_data()
                self.success_count += 1
            else:
                self.camera_controller.log_failed_box_positions(self.new_x , self.new_y , BOX_Z)
                print("<>"*TOTAL_EPISODES)
                print("Episode failed: Not recording the episode")
                print("<>"*TOTAL_EPISODES)


            # self.new_x, self.new_y = 0.3 + random.uniform(0, 0.2), 0.5 + random.uniform(-0.2, 0)
            self.set_box_position(self.new_x, self.new_y, BOX_Z)
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
        z = BOX_Z
        pre_pick_pos = np.array([new_x, new_y, z + 0.045], dtype=np.float64)
        pick_pos = np.array([new_x, new_y, z - 0.003], dtype=np.float64)
        return pre_pick_pos, pick_pos

    def solve_kinematics(self, joint_positions, position, quat):
        return self.kinematics.ik(joint_positions, position, quat)

    def operate_gripper(self, width, f):
        self.gripper_width = width  # Update the local gripper width
        self.camera_controller.gripper_width = width  
        self.panda_arm.exec_gripper_cmd(width, f)
        

    def move(self, solution):
        if solution is not None:
            self.panda_arm.move_to_joint_position(solution.tolist())# Log the action after update


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
        self.move(solution)


        pos2 = np.array([0.6, -0.4, 0.16])
        solution = self.solve_kinematics(joint_positions, pos2, ori)
        rospy.sleep(1)
        self.move(solution)
        self.operate_gripper(OPEN_GRIPPER_POSE , GRIPPER_FORCE)

def main():
    rospy.init_node('camera_control_node', anonymous=True)
    panda_arm = PandaArm()
    camera_controller = CameraController(panda_arm)
    robot_task = PandaRobotTask(camera_controller, panda_arm)
    robot_task.perform_task()

if __name__ == '__main__':
    main()
