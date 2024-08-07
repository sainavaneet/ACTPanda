import os
# fallback to cpu if mps is not available for specific operations
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = "1"
import torch

# data directory


# checkpoint directory
CHECKPOINT_DIR = 'checkpoints/'

# device
device = 'cpu'
if torch.cuda.is_available(): device = 'cuda'
#if torch.backends.mps.is_available(): device = 'mps'
os.environ['DEVICE'] = device

DATASET_DIR = "/home/navaneet/Desktop/ACTPanda/processed_data"

DATA_DIR = DATASET_DIR

INITIAL_JOINTS = [0, -0.7, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]

OPEN_GRIPPER_POSE = 0.04

GRASP = 0.028

GRIPPER_FORCE = 0.11

INITAL_GRIPPER_POSE = 0.005

TOTAL_EPISODES = 50

BOX_Z = 0.025


TASK_CONFIG = {
    'dataset_dir': DATA_DIR,
    'episode_len': 400,
    'state_dim': 8,
    'action_dim': 8,
    'cam_width': 640,
    'cam_height': 480,
    'camera_names': ['top'],
    'camera_port': 50
}


# policy config
POLICY_CONFIG = {
    'lr': 1e-5,
    'device': device,
    'num_queries': 100,
    'kl_weight': 100,
    'hidden_dim': 512,
    'dim_feedforward': 3200,
    'lr_backbone': 1e-5,
    'backbone': 'resnet18',
    'enc_layers': 4,
    'dec_layers': 7,
    'nheads': 8,
    'camera_names': ['top'],
    'policy_class': 'ACT',
    'temporal_agg': False
}

# training config
TRAIN_CONFIG = {
    'seed': 42,
    'num_epochs': 10000,
    'batch_size_val': 8,
    'batch_size_train': 8,
    'eval_ckpt_name': 'policy_last.ckpt',
    'checkpoint_dir': CHECKPOINT_DIR
}