# Change log
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/models/data_loader.py::load_trajectory` so that the data_loader label_length corresponds to config.state_dim
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/PlannerBase.py::_convert_to_traj()` set the heading to the predicted yaw angle
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/models/utils.py::_convert_to_traj_sample` to convert yaw predictions into a trajectory
- [x] modified `_convert_to_traj_sample` to save yaw angle to csv and extract yaw angle from rotation matrix

During training `data_loader.py::load_trajectory` is used to build the dataset from pre-collected data. 
- [x] change the file formatting of `trajectories/trajectories_{}_{:08d}.csv` to be x,y,z,yaw
- [x] modified `catkin_aa/src/agile_autonomy/planner_learning/src/PlannerLearning/models/plan_learner.py::PlanLearner.test` to save yaw angle to csv when either state_dim is 4 or config.save_trajectory_yaw is True
- [x] Set the heading when active yawing is enabled on line 374 of `catkin_aa/src/agile_autonomy/data_generation/agile_autonomy_utils/src/trajectory_ext.cpp`. 
I created a new boolean called `active_yawing_enabled_`, which is set in the various yaml files and loaded when the corresponding training/testing python scripts are run. 
Note, this is different than `yawing_enabled_` which uses velocity-tracking yaw. When both velocity-tracking yaw and active yawing are enabled, the heading set by active yaw takes precedent. 
# TrajectoryExtHeading
The TrajectoryExt class is used to project the predicted positions onto the space of polynomial b-splines
- [x] `catkin_aa/src/agile_autonomy/data_generation/agile_autonomy_utils/src/trajectory_ext.cpp`Added new polynomial reference trajectory for heading angle
The `_generate_plan()` callback is run at the network_frequency (15 Hz) which runs inference on the network and converts the prediction into a trajectory. 
`_convert_to_traj` publishes `quadrotor_msgs.msgs.MultiTrajectory` which the controller subscribes to and tracks when the `multi_traj.execute` flag is set to True.
- [ ] make sure controller isn't using velocity-tracking yaw, otherwise there's no point to predicting a yaw