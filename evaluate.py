import torch
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pickle
import pandas as pd
from panda_robot import PandaArm
from training.utils import *
from config.config import POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from tqdm import tqdm

# Image processing and handling
bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS Image message to OpenCV image (BGR format)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
        rospy.logerr(e)
    return cv_image

def capture_image():
    msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=10)  # Added timeout for reliability
    return image_callback(msg)

# Configuration
cfg = TASK_CONFIG
policy_config = POLICY_CONFIG
train_cfg = TRAIN_CONFIG
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

if __name__ == "__main__":
    rospy.init_node('panda_robot_controller', anonymous=True)

    # Initialize the PandaArm
    panda = PandaArm()
    panda.move_to_neutral()

    # Load the policy
    ckpt_path = os.path.join(train_cfg['checkpoint_dir'], train_cfg['eval_ckpt_name'])
    policy = make_policy(policy_config['policy_class'], policy_config)
    policy.load_state_dict(torch.load(ckpt_path, map_location=device))
    policy.to(device)
    policy.eval()
    rospy.loginfo(f'Policy loaded from {ckpt_path}')

    stats_path = os.path.join(train_cfg['checkpoint_dir'], 'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda act: act * stats['action_std'] + stats['action_mean']

    num_rollouts = 4
    for _ in range(num_rollouts):
        obs_replay = []
        action_replay = []

        for t in tqdm(range(cfg['episode_len']) , desc="Rollouts"):
            position = np.array(panda.angles())  # Simplified joint angles capture
            gripper_width = panda.gripper_state()['position'][0]  # Assuming it returns an array

            pos_numpy = np.concatenate((position, [gripper_width]))
            pos = pre_process(pos_numpy)
            pos_tensor = torch.from_numpy(pos).float().to(device).unsqueeze(0)

            # Capture image
            image = capture_image()

            # Ensure the image is in the correct format
            image_tensor = torch.from_numpy(image).permute(2, 0, 1).float().to(device).unsqueeze(0)

            # Get action from policy
            action_tensor = policy(pos_tensor, image_tensor)
            actions = post_process(action_tensor.detach().cpu().numpy())
            # print(actions.shape)
            actions = actions.reshape(100, 8)  # Adjust shape according to your setup

            # Save actions to CSV
        df = pd.DataFrame(actions)
        df.to_csv('actions.csv',index = False)
