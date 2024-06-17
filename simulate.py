import pandas as pd
from panda_robot import PandaArm
import rospy

rospy.init_node("Moving angles")

panda = PandaArm()

panda.move_to_neutral()

# Read the CSV file
file_path = 'actions.csv'  # Replace with the correct path to your CSV file
data = pd.read_csv(file_path, header=None)

# Process each row, publish every 10th row
for index, row in data.iterrows():
    if index % 5 == 0:
        joint_positions = row[:7].tolist()
        gripper_position = row[7]

        panda.move_to_joint_position(joint_positions)
        panda.exec_gripper_cmd(gripper_position)
