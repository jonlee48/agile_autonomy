import numpy as np
import pandas as pd

csv_file = "/home/jonathan/datasets/agile_autonomy_dataset/forest/7ms_straight/all_train_forest/train/rollout_21-02-06_23-32-09/trajectories/trajectories_bf_00000072.csv"
print("csv file")
df = pd.read_csv(csv_file)
csv_array = df.to_numpy()
print(csv_array.shape)
# print(csv_array)

npy_file = "/home/jonathan/datasets/agile_autonomy_dataset/forest/7ms_straight/all_train_forest/train/rollout_21-02-06_23-32-09/trajectories/trajectories_bf_00000072.npy"
array = np.load(npy_file)
print("npy file")
print(array.shape)
print(array)

print(np.allclose(csv_array, array))