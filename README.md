# State Action Record Ros2
This repo is used to collect state-action pairs with Ros2 for robot learning from demonstration (LfD). 

The recorded data will be stored in [h5py format](https://docs.h5py.org/en/stable/quick.html).

## Prerequisite
- Install [Ros2 Humble](https://docs.ros.org/en/humble/Installation.html) 
- Install [h5py format](https://docs.h5py.org/en/stable/quick.html)
- (Optional) Install [frankalib](https://docs.h5py.org/en/stable/quick.html) (If Franka arm is used)

## Configuration and Naming Rules
1. ***Data Subscribing Structure***
```
│── data_class (e.g. 'act', 'obs', defined by a yaml file)
│   ├── subscriber (e.g. 'left_ppcp')
│   │   └── signal_type (e.g. 'pose')
│   │   └── ...
│   ├── ...
├── ...
```
- signal_type: signals from other node to record
- subscriber: subcribes to a topic, the msg from which can contain several signal_types
- data_class: name of a `yaml` file, where several subscribers can be defined
2. ***Naming of data in dataset***
- In the recorded h5py dataset, each signal_type will be named as:
```
<data_class>/<name of subscriber>_<name of signal_type>
e.g.: obs/left_ppcp_pose
```
3. ***What to configure?***
- `config.yaml`: contains general parameters (check comments inside), which can also be configured with argparser.
- `obs/act.yaml`: here you can specify the names of subscriber (e.g. 'left_ppcp') and the names of signal_type (e.g. pose). Please leave the other names of parameter unchanged. Please also refer to the examples and comments in `obs/act.yaml` to check the parameters to be configured.
***NOTICE***: Right now, only array-like msg attribute (e.g. `msg.data=[0,1,2]`) is supperted. Might have better compatibility in later versions...



## Getting Started
1. Create a workspace and a src folder in the workspace (e.g. `mkdir state_action_record_ws && cd vr_ws && mkdir src`). Clone the repo under `src` then go to workspace.
2. Finish configuring the `yaml` files (check ***Configuration and Naming Rules*** above).
3. Build the package `colcon build` and source it `source install/setup.bash`.
4. (Optional) Set start node condition in the method `start_node` in `state_actiion_record.py`. In the current code, the action signal `left_pose_pose` needs a starting position before recording. You can specify your own starting condition or set the `need_start_condition` in `config.yaml` as `False`.
5. Start the recording node:
```
ros2 run state_action_record state_action_record
```
- (optional) with argparser, e.g.:
```
ros2 run state_action_record state_action_record --save_path "src/state_action_record_ros2/state_action_record/datasets/dataset_name.hdf5"
```
6. Stop recording by `ctrl+c`. The dataset will be saved in `src/state_action_record_ros2/state_action_record/datasets/`. You can specify `save_path` in `config.yaml`.