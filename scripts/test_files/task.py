import rospy
import numpy as np
from panda_robot import PandaArm
from panda_kinematics import PandaWithPumpKinematics
from franka_interface import GripperInterface



def initialize_robot():
    """
    Initializes the ROS node and the PandaArm with default joint positions.
    """
    rospy.init_node("panda_arm_test_node")
    panda_arm = PandaArm()
    kinematics = PandaWithPumpKinematics()
    initial_positions = [0, -0.7, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]
    panda_arm.move_to_joint_position(initial_positions)
    return panda_arm, kinematics

def move(panda_arm, solution):
    """
    Moves the PandaArm to the specified joint position.
    """
    if solution is not None:
        panda_arm.move_to_joint_position(solution.tolist())
    else:
        print("No valid IK solution found, cannot move to the desired position.")

def operate_gripper(panda_arm, action):
    """
    Operates the gripper to either 'open' or 'close'.
    """
    gripper_interface = GripperInterface()
    if action == 'open':
        gripper_interface.open()
    elif action == 'close':
        gripper_interface.close()
    else:
        raise ValueError("Action must be 'open' or 'close'")

def solve_kinematics(kinematics, initial_joint_positions, position, quat):
    """
    Solves the inverse kinematics for the given position and orientation.
    """
    solution = kinematics.ik(initial_joint_positions, position, quat)
    return solution

def home_gripper(panda_arm):
    """
    Homes the gripper joints.
    """
    panda_arm.get_gripper().home_joints()

def place_in_basket(panda_arm, kinematics):
    """
    Places an object into a basket by moving to a predefined position and opening the gripper.
    """
    joint_positions = np.zeros(7)  # Or use the current robot joint states if available
    pos = np.array([0.6, -0.4, 0.4])
    ori = np.array([np.pi, np.pi/2, 0.0, 0.0])
    solution = solve_kinematics(kinematics, joint_positions, pos, ori)
    move(panda_arm, solution)
    
    pos2 = np.array([0.6, -0.4, 0.16])
    solution = solve_kinematics(kinematics, joint_positions, pos2, ori)
    rospy.sleep(1)
    move(panda_arm, solution)
    operate_gripper(panda_arm, 'open')

def perform_task(panda_arm, kinematics):
    gripper_interface = GripperInterface()

    """
    Performs a task where the PandaArm moves to two different positions and operates the gripper.
    """
    joint_positions = np.zeros(7)
    position1 = np.array([0.6, 0.0, 0.04])
    orientation1 = np.array([np.pi, np.pi/2, 0, 0])
    operate_gripper(panda_arm, 'close')
    solution = solve_kinematics(kinematics, joint_positions, position1, orientation1)
    move(panda_arm, solution)
    operate_gripper(panda_arm, 'open')
    rospy.sleep(5)
    position2 = np.array([0.6, 0.0, 0.27])
    orientation2 = np.array([np.pi, np.pi/2, 0.0, 0.0])

    solution = solve_kinematics(kinematics, solution, position2, orientation2)
    move(panda_arm, solution)
    gripper_interface.grasp(width=0.028, force=0.1, speed=None, epsilon_inner=0.01, epsilon_outer=0.01)
    rospy.sleep(1)
    place_in_basket(panda_arm, kinematics)
    operate_gripper(panda_arm, 'open')

if __name__ == "__main__":
    panda_arm, kinematics = initialize_robot()
    perform_task(panda_arm, kinematics)
