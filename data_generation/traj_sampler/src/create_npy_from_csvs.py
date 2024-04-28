import numpy as np
import matplotlib.pyplot as plt
import os
import math


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def plot_trajectory(data):
    plt.figure()
    x = data[0:10]
    y = data[10:20]
    yaw = data[30:40]
    plt.plot(x, y, '-o', label='Trajectory Positions')
    for i in range(len(x)):
        dx = 0.2 * np.cos(yaw[i])  # Reduced arrow size
        dy = 0.2 * np.sin(yaw[i])  # Reduced arrow size
        print(yaw[i], dx, dy)
        plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.1,
                  color='red')  # Adjusted arrow size
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Trajectory with Yaw Angles')
    plt.legend()
    plt.axis('equal')  # Ensure equal aspect ratio
    plt.show()


root_dir = "/home/soham/dev/IRL/project/agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation"
rollouts_paths = [os.path.join(root_dir, d) for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d)) and d.startswith("rollout")]

for rollouts_path in rollouts_paths:
    trajectories_path = os.path.join(rollouts_path, "trajectories")
    npy_names = [f for f in os.listdir(trajectories_path) if f.startswith("trajectories_bf_") and f.endswith(".npy")]
    ids = [npy_name[len("trajectories_bf_"):-len(".npy")] for npy_name in npy_names]
    quat_csv_names = [f"perc_aware_trajectories_bf_{id_}.csv" for id_ in ids]
    for npy_name, quat_csv_name in zip(npy_names, quat_csv_names):
        if not (os.path.exists(os.path.join(trajectories_path, npy_name)) and
                os.path.exists(os.path.join(trajectories_path, quat_csv_name))):
            continue
        npy_data = np.load(os.path.join(trajectories_path, npy_name))
        quat_data = np.loadtxt(os.path.join(trajectories_path, quat_csv_name), delimiter=',')[:, 4:]
        if npy_data.shape[0] != quat_data.shape[0]:
            continue
        yaw_data = np.zeros((quat_data.shape[0], int(quat_data.shape[1] / 4)))
        for i in range(yaw_data.shape[0]):
            for j in range(yaw_data.shape[1]):
                yaw_data[i, j] = euler_from_quaternion(quat_data[i][4 * j + 1],
                                                       quat_data[i][4 * j + 2],
                                                       quat_data[i][4 * j + 3],
                                                       quat_data[i][4 * j + 0])[2]
                # print(quat_data[i][4 * i:4 * (i + 1)], yaw_data[i, j])
                # assert False
        combined_data = np.column_stack((npy_data[:, :30], yaw_data, npy_data[:, 30]))
        np.save(os.path.join(trajectories_path, f"perc_aware_{npy_name}"), combined_data)
        # plot_trajectory(combined_data[0, :])













