import rospy
import numpy as np
import random
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty
from panda_robot import PandaArm
from panda_kinematics import PandaWithPumpKinematics
from franka_interface import GripperInterface
from constants import INITIAL_JOINTS

class PandaRobotTask:
    def __init__(self):
        rospy.init_node("panda_arm_task_node")
        self.panda_arm = PandaArm()
        self.kinematics = PandaWithPumpKinematics()
        self.gripper_interface = GripperInterface()
        self.initial_positions = INITIAL_JOINTS
        self.new_x = 0.3
        self.new_y = 0.5
        self.set_box_position(0.5, self.new_y, 0.025)
        self.panda_arm.move_to_joint_position(self.initial_positions)

    def move(self, solution):
        if solution is not None:
            self.panda_arm.move_to_joint_position(solution.tolist())

    def operate_gripper(self, action):
        if action == 'open':
            self.gripper_interface.open()
        elif action == 'close':
            self.gripper_interface.close()

    def solve_kinematics(self, joint_positions, position, quat):
        return self.kinematics.ik(joint_positions, position, quat)

    def place_in_basket(self):
        joint_positions = np.zeros(7)  # Or use the current robot joint states if available
        pos = np.array([0.6, -0.4, 0.4])
        ori = np.array([np.pi, np.pi/2, 0.0, 0.0])
        solution = self.solve_kinematics(joint_positions, pos, ori)
        self.move(solution)
        
        pos2 = np.array([0.6, -0.4, 0.16])
        solution = self.solve_kinematics(joint_positions, pos2, ori)
        rospy.sleep(1)
        self.move(solution)
        self.operate_gripper('open')

    def reset_simulation(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def set_box_position(self, x, y, z):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state_msg = SetModelStateRequest()
            state_msg.model_state.model_name = 'block_red_2'
            state_msg.model_state.pose.position.x = x
            state_msg.model_state.pose.position.y = y
            state_msg.model_state.pose.position.z = z
            state_msg.model_state.pose.orientation.w = 0
            set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def update_box_pos(self, new_x, new_y):
        z = 0.025
        pre_pick_pos = np.array([new_x, new_y, z + 0.045], dtype=np.float64)
        pick_pos = np.array([new_x, new_y, z - 0.003], dtype=np.float64)
        return pre_pick_pos, pick_pos

    def perform_task(self):
        while not rospy.is_shutdown():
            pre_pick_pos, pick_pos = self.update_box_pos(self.new_x, self.new_y)
            orientation_quat = np.array([np.pi, np.pi/2, 0, 0], dtype=np.float64)

            # Fetch current joint angles and convert to numpy array
            current_joint_angles = np.array(list(self.panda_arm.joint_angles().values()), dtype=np.float64)
            
            solution = self.solve_kinematics(current_joint_angles, pre_pick_pos, orientation_quat)
            self.move(solution)
            self.operate_gripper('open')

            rospy.sleep(1)
            # Move to pick position and grasp
            solution = self.solve_kinematics(current_joint_angles, pick_pos, orientation_quat)
            self.move(solution)
            success =self.panda_arm.exec_gripper_cmd(0.028 , 0.1)
            # success = self.gripper_interface.grasp(width=0.028, force=0.1, speed=None, epsilon_inner=0.01, epsilon_outer=0.01)
            if not success:
                print("Grasp failed, resetting the episode.")
                self.reset_episode()
                continue
            
            # Execute basket placement
            self.place_in_basket()

            # Reset simulation and change box position
            self.new_x, self.new_y = 0.3 + random.uniform(0, 0.3), 0.5 + random.uniform(-0.3, 0)
            self.set_box_position(self.new_x, self.new_y, 0.025)
            self.panda_arm.move_to_joint_position(self.initial_positions)


if __name__ == "__main__":
    robot_task = PandaRobotTask()
    robot_task.perform_task()
