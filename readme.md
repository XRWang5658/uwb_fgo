# UWB Fusion Workspace

UWB-based factor-graph optimizer (ROS + GTSAM) with dual anchors, Butterworth filtering, and robust FOV-aware noise.

## Prerequisites
- ROS (e.g., Noetic) and catkin tools
- GTSAM (>=4.0)
- C++14 toolchain, Eigen, tf/geometry, nav_msgs, geometry_msgs, std_msgs
- rviz

Install common deps (Ubuntu/ROS Noetic example):
```bash
sudo apt-get update
sudo apt-get install -y python3-catkin-tools ros-noetic-tf ros-noetic-geometry-msgs \
  ros-noetic-nav-msgs ros-noetic-std-msgs ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-rviz libeigen3-dev
# Install GTSAM from packages if available:
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

## Clone & Build
```bash
# Get the workspace
git clone <repo-url> uwb_ws
cd uwb_ws

# Initialize src if empty
mkdir -p src

# If this repo already contains src, just build:
catkin build uwb_fusion
source devel/setup.bash
```

If you use `catkin_make` instead:
```bash
catkin_make
source devel/setup.bash
```

## Run
```bash
roslaunch uwb_fusion run_uwb.launch
```
`run_uwb.launch` sets static TFs for anchors, starts the optimizer and baseline visualizer, and loads RViz.

Key parameters (see `src/uwb_fgo/src/uwb_fusion/launch/run_uwb.launch`):
- `topic_uwb_left`, `topic_uwb_right`: raw UWB topics (default `/uwb/sensor1_data`, `/uwb/sensor2_data`)
- `map_frame`: fixed frame (default `map`)
- Noise: `noise_range_sigma`, `noise_angle_sigma`, `noise_elev_sigma`, `fov_limit_deg`
- LPF: `lpf_cutoff_freq`, `lpf_sample_freq`
- Fusion timing: `measurement_sync_window`, `update_rate`, `keyframe_time_thresh`

## Package Layout
- `src/uwb_fgo/src/uwb_fusion/`
  - `launch/run_uwb.launch` — main launch
  - `src/uwb_gtsam_optimizer.cpp` — optimizer, LPF, factor graph
  - `config/uwb_test.rviz` — RViz config

## Tips
- Make sure your UWB drivers publish to the configured topics.
- Adjust LPF sample frequency to match your sensor rate.
- Source `devel/setup.bash` in each new terminal before running. 