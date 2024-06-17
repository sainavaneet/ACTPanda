import pickle
import config

# Load the stats file
with open('/home/navaneet/Desktop/ACT_NAVANEET/checkpoints/task1/dataset_stats.pkl', 'rb') as f:
    stats = pickle.load(f)

# Print the keys in the dictionary
print(stats.keys())
