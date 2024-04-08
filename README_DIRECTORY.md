# Agile Autonomy Directory
Paths to notable files that provide useful functionality or configurations that we may want to customize.

## data_generation/

### agile_autonomy/

#### parameters/
- `flightmare.yaml` controls unity environment and camera parameters
- `default.yaml` sets `enable_yawing: True` which we might want to set to false to train a baseline with no yawing. Also contains `test_time_velocity: 7` which we may want to adjust

#### launch/
- `simulation.launch` by default sets `use_mpc=true`, we can experiment with setting this arg to false. Also can use 12 labelled threads instead of 8 (on CPUs with more cores)

### traj_sampler/

#### parameters/
- `mpc_params.yaml` contains a copy of mpc parameters which is loaded into `generate_label_8.launch` which is then invoked in `simulation.launch`

### viz_utils/
`visualize_trajectories.py` plots trajectory rollouts in open3d
```
python visualize_trajectories.py --data_dir /PATH/TO/rollout_21-09-21-xxxx --start_idx 0 --time_steps 100 --pc_cutoff_z 2.0 --max_traj_to_plot 100
```

### agile_autonomy_utils/src
- `trajectory_ext.cpp` sets the heading based on velocity-tracking yaw if `yawing_enabled_` is set (line 371 and 591)
    - This might need to be changed so we can predict more than just velocity-tracking yaw (unless velocity-tracking yaw is fine, and we're just considering a visibility score)
    - Also includes `fitPolynomialCoeffs`

---

## logging_utils/
`print_avg_metrics_poles.py` prints a bunch of metrics including success rate, averages, number of crashes, closest distances from poles with success

## planner_learning/
`test_trajectories.py` calls `perform_testing()` with the given settings file
```
python test_trajectories.py --settings_file=config/test_settings.yaml
```

### config/
In these yaml files, I update the `state_dim` from `state_dim: 3` to `state_dim: 4` to account for x, y, z, and yaw.

Note the location of `log_dir` goes to `/tmp` in case you want to look at that later.
- `dagger_settings.yaml` by default starts from the checkpoint and trains off data in expert folder (`../data_generation/data`) which is currently empty. Includes other training parameters.
- `deploy_settings.yaml` I don't think we need to use this - looks like it's used for real-world hardware tests
- `openloop_settings.yaml` I don't see this being called anywhere
- `test_settings.yaml` chnages to `dagger_settings.yaml` need to be propagated here. Also the new checkpoint path will need to be updated, and `perform_global_planning` in `default.yaml` set back to false to make the simulation faster.
- `train_settings.yaml` since we're changing the network architecture, we will not need this as instead we'll train from scratch (ie. with `dagger_settings.yaml`)
    - that might not be true since we're only changing the expert, and only how the expert scores its trajectories
    - need to update `train_dir` and `val_dir` to point to pre-collected dataset
    - has a parameter called `track_global_traj` which is set to False by default, but does this need to be enabled?
- `settings.py` takes these yaml files and creates nice classes

- Difference betwen `dagger_settings` and `train_settings`?
    - See section "Train your own navigation policy" in the Agile Autonomy readme
    - train just uses the pre-collected dataset
        - `python train.py --settings_file=config/train_settings.yaml`
    - dagger requires data collection and enabling global planning
        - `python dagger_training.py --settings_file=config/dagger_settings.yaml`

### data/train/
- `rollout_DATE_TIME/` path to images and trajectories from training? (Q: why is this here and not in the data_generation directory? A: because the data_generation dir is only for generating expert trajectories which can be gathered before hand)

### To modify if we're using training (ie pre-collected dataset)
#### src/PlannerLearning/models/
- `plan_learner.py` defines the `PlanLearner` class which takes the `state_dim` variable from `train_settings.yaml` and uses it to initialize two `TrajectoryCostLoss` for train and validation
- `utils.py` defines the `TrajectoryCostLoss` which takes in the `state_dim` and has a function `add_pointclouds` which is called with the train/validation point cloud passed to it
    - `TrajectoryCostLoss` is not what we want, we want to edit `MixtureSpaceLoss` which uses a `DiscretePositionLoss`

#### To modify if we're using dagger_training
- `PlanLearning` extends `PlanBase`
- `PlanBase` implements the following
    - `trajectory_decision()` which takes the network predictions and converts it to a trajectory with `_convert_to_traj()`
    - `_convert_to_traj()` creates a quadrotor_msgs.msgs.TrajectoryPoint out of the network prediction

- If using the train script, update `data_loader.py` so PlanDataset.load_trajectory uses a label_length of 4.

#### To modify for testing
```
test_trajectories.py
```
- In `dagger_training.py`, `Trainer.perform_testing()` calls `PlanLearning` and we check if `maneuver_complete` and ask for `experiment_report()`



# Yaw angle considerations

We should consider the `TrajectoryType`. Either `GENERAL` or `ACCELERATION`, depending on if we want orientation to be determined from headings
- This affects `data_generation/agile_autonomy_utils/include/agile_autonomy_utils/generate_reference.h::computeReferenceTrajectoryPosBased`

# Questions
1. What is alpha in planner_learning?
2. Can we just use the training script? Are the training labels only 3 dim?
3. What are the modes in MixtureSpaceLoss
    - The mode in train_settings.yaml is "modes of distribution"
    - The mode in test_settings.yaml is "number of trajectories predicted"


# Changelog/todo
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/models/data_loader.py::load_trajectory` so that the label_length corresponds to config.state_dim
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/PlannerBase.py::_convert_to_traj()` so that the network prediction output also sets the heading of the trajectory if the state_dim is 4. This is used by the `_generate_plan()` callback which is run at the network_frequency to publish the trajectories for the controller to track when running in dagger or testing mode.
- [ ] multi_traj.execute flag should be set to True for all of test mode
- [ ] change the file formatting of trajectories/trajectories_{}_{:08d}.npy to be x,y,z,yaw
    - trajectories are ordered from highest ranking to lowest in the trajectories file
    - line 267 of data_loader.py, `traj_set[:k] = all_traj[:k, -1]`, the last index of each trajectory is ignored - it is the score
    - append yaw angle after x,y,z

- [ ] make sure controller isn't using velocity-tracking yaw, otherwise there's no point to predicting a yaw
- [ ] predict_state_number