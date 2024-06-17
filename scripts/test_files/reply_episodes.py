import h5py
import os
import time
import cv2
import numpy as np
from panda_robot import PandaArm  # Check if this import path is correct
from constants import *
import rospy
import threading
from franka_interface import GripperInterface
from gazebo_msgs.srv import SetModelState, SetModelStateRequest



class ReplayEpisodes:
    def __init__(self ):
        rospy.init_node('replay_episode', anonymous=True)
        self.data_dir = DATASET_DIR
        self.panda_arm = self.connect_to_robot()
        self.gripper_interface = GripperInterface()
        if self.panda_arm:
            self.reset_to_initial_position()

    def reset_to_initial_position(self):
        try:
            self.panda_arm.move_to_joint_position(INITIAL_JOINTS)
            rospy.sleep(2)  # Ensure the robot has enough time to reach the initial positions
        except Exception as e:
            print(f"Error moving robot to initial position: {e}")

    def read_file(self, index):
        cam_name = 'top'
        dataset_file = os.path.join(self.data_dir, f'episode_{index}.hdf5')
        with h5py.File(dataset_file, 'r') as root:
            qpos, actions, images, box_positions = [], [], [], []
            if '/action' in root and '/observations/qpos' in root:
                actions = root['/action'][:]
                qpos = root['/observations/qpos'][:]
                images_rec = root[f'/observations/images/{cam_name}']
                images = [im[:] for im in images_rec]
                if '/observations/box_positions' in root:
                    box_positions = root['/observations/box_positions'][:]
        return qpos, actions, images, box_positions

    def move_robot(self, action):
        try:
            print(action)
            joint_positions = action[:-1]  # All except the last elementa
            gripper_state = action[-1]  # Last element
            self.panda_arm.move_to_joint_position(joint_positions)  # Move robot joints
            self.operate_gripper(gripper_state)

        except Exception as e:
            print(f"Error moving robot: {e}")

    def operate_gripper(self, state):
        self.panda_arm.exec_gripper_cmd(state , GRIPPER_FORCE)            


    def set_box_position(self, x, y, z):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = SetModelStateRequest()
        state_msg.model_state.model_name = 'block_red_2'
        state_msg.model_state.pose.position.x = x
        state_msg.model_state.pose.position.y = y
        state_msg.model_state.pose.position.z = z
        set_state(state_msg)

    def play_episode(self, index):
        qpos, actions, images, box_positions = self.read_file(index=index)
        
        print('+' + '-'*40 + '+')
        print('|{:^40}|'.format(''))
        print('|' + '{:^40}'.format("qpos shape: {}".format(qpos.shape)) + '|')
        print('|' + '{:^40}'.format("action shape: {}".format(actions.shape)) + '|')
        print('|' + '{:^40}'.format("images length: {}".format(len(images))) + '|')
        print('|{:^40}|'.format(''))
        print('+' + '-'*40 + '+')


        if len(images) < len(actions):
            images.append(images[-1])  # In case there are fewer images than actions

        if len(box_positions) > 0:
            self.set_box_position(*box_positions[0])

        self.reset_to_initial_position()  
        for i in range(0, len(actions), 5):
            if not self.panda_arm:
                break
            self.move_robot(actions[i])

        cv2.destroyAllWindows()

    def connect_to_robot(self):
        try:
            arm = PandaArm()  # Initialize your PandaArm class
            return arm
        except Exception as e:
            print(f"Problem connecting to robot: {e}")
            return None

def main(args=None):
    re = ReplayEpisodes()
    re.play_episode(50)  

if __name__ == '__main__':
    main()
