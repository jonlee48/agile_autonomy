# Training Procedure Notes

## Train your own navigation policy

There are two ways in which you can train your own policy. One easy and one more involved.
The trained checkpoint can then be used to control a physical platform (if you have one!).

### Use pre-collected dataset

The first method, requiring the least effort, is to use a dataset that we pre-collected. The dataset can be found at [this link](https://zenodo.org/record/5517791/files/agile_autonomy_dataset.tar.xz?download=1). This dataset was used to train the model we provide and collected at an average speed of 7 m/s. To do this, adapt the file [train\_settings.yaml](planner_learning/config/train_settings.yaml) to point to the train and test folder and run:

```bash
cd agile_autonomy_ws
source catkin_aa/devel/setup.bash
conda activate tf_24
python train.py --settings_file=config/train_settings.yaml
```

#### Visualizing the training set
You can visualize all rollouts in the train directory, or just a single rollout.
```
cd agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation/viz_utils

python visualize_trajectories.py --data_dir ~/datasets/agile_autonomy_dataset/forest/7ms_straight/all_train_forest/train --start_idx 0 --time_steps 100 --pc_cutoff_z 3.0 --max_traj_to_plot 100

python visualize_trajectories.py --data_dir ~/datasets/agile_autonomy_dataset/objects/7ms_straight/all_train_objects/train --start_idx 0 --time_steps 100 --pc_cutoff_z 3.0 --max_traj_to_plot 100
```

#### Training set options

The training set contains multiple valid training and testing splits for the forest environment.

There are multiple options for training within the provided dataset: forest only, objects only, and both. Set the `train_dir` and `val_dir` to the corresonding paths.

Forest only
```
# 1) first training set (80 rollouts)
agile_autonomy_dataset/forest/7ms_straight/all_train_forest/train/
# 2) or second training set (81 rollouts)
agile_autonomy_dataset/forest/7ms_straight/all_train_forest/train_second/
# 3) or both training sets
agile_autonomy_dataset/forest/7ms_straight/all_train_forest/

# validation set
agile_autonomy_dataset/forest/7ms_straight/test/
```

Objects only
```
# 1) first training set (81 rollouts)
agile_autonomy_dataset/objects/7ms_straight/all_train_objects/train/
# 2) or second training set (81 rollouts)
agile_autonomy_dataset/objects/7ms_straight/all_train_objects/train_second/
# 3) or third training set (81 rollouts)
agile_autonomy_dataset/objects/7ms_straight/all_train_objects/train_second/
# 4) or all training sets
agile_autonomy_dataset/objects/7ms_straight/all_train_objects/

# validation set
agile_autonomy_dataset/objects/7ms_straight/test/
```

Both forest and objects
```
# all training sets
agile_autonomy_dataset/train_all_7ms/train

# validation set
agile_autonomy_dataset/train_all_7ms/test
```

## Analyzing data

Model checkpoints and tensorboard are also saved to the `log_dir` path.

Start tensorboard. It contains plots for space loss and trajectory cost loss.
```
tensorboard --logdir /tmp/train_straight_7ms/.../train
```

E.g. last saved checkpoint for epoch 150 in 
```
/tmp/train_straight_7ms/20240406-132426/train/ckpt-46
```


## Test the trajectory

Launch the simulation (deactivate conda environments for this)
```bash
cd agile_autonomy_ws
source catkin_aa/devel/setup.bash
roslaunch agile_autonomy simulation.launch
```

Change the path to the model checkpoint in `agile_autonomy/planner_learning/config/test_settings.yaml` under attribute `resume_file`.

E.g. `resume_file: "/tmp/train_straight_7ms/20240406-132426/train/ckpt-46"`

Run the Network in an other terminal:
```bash
cd agile_autonomy_ws
source catkin_aa/devel/setup.bash
conda activate tf_24
cd catkin_aa/src/agile_autonomy/planner_learning
python test_trajectories.py --settings_file=config/test_settings.yaml
```

Open RViz to visualize the rollout. In the pane on the rhs, set the view type to FPV and the target frame to `hummingbird/base_link`.

Statistics are contained in `log_dir` under experiment_metrics.json including distance traveled, number of crashes, and closest distances to obstacles.

These can be printed using
```
cd agile_autonomy_ws
source catkin_aa/devel/setup.bash
conda activate tf_24
cd catkin_aa/src/agile_autonomy/planner_learning
python print_avg_metrics_poles.py --data_dir="/tmp/trash_reactive/20240408-165045/tree_5_obj_5/"
```

The output will look something like 
```
Number of loaded rollouts is 32
Average expert_usage is 19.512582126554893
Average number_crashes is 1.0
Average travelled_dist is 22.938725074692847
Average closest_distance is 0.26665869691128385
Success Rate is 0.375
Avg. closest distances from poles when success is 0.5133018981491919
```