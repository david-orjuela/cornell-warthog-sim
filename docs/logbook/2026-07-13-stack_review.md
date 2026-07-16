#7-13-26_day_2_stack_review.md

# Stack Notes

## Clearpath Simulator Readme
- ROS2 **Jazzy**, Gazebo **Harmonic**. Need to find out compatible versions/distros with Warthog/current repo, as well as if I need a Linux OS or can get by on Windows.
- What is `robot.yaml`? Is that the Warthog model (or any model, but in my case).
- `Orchard` world parameter will be used for initial clearpath simulator run

### Setup

Prerequisites:
  - Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

#### Gazebo Harmonic

See [Gazebo Installation](https://gazebosim.org/docs/latest/ros_installation/) for more information
on installing Gazebo.

```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

#### Workspace

```
mkdir ~/clearpath_ws/src -p
cd ~/clearpath_ws/src
git clone https://github.com/clearpathrobotics/clearpath_simulator.git
cd ~/clearpath_ws
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
```

#### Setup path

```
mkdir ~/clearpath
```

Copy your `robot.yaml` into `~/clearpath`

### Launch

```
ros2 launch clearpath_gz simulation.launch.py
```
Will end up using:
```
ros2 launch clearpath_gz simulation.launch.py world:=orchard
```


## Nav2 Overview
- Uses behavior trees (BT)
- BT allows for customized and intelligent navigation behavior by orchestrating servers
- Provides all major robotic navigation needs: perception, planning, control, localization, visualization, etc.
- Cost maps? Local, global? Need to understand deeper what this means and how it connects. I'm sure it relates to how a robot decides what to do -- optimal path is simply least cost, optimal behavior is least cost, etc.
- From diagram: controller server uses local costmap, planner server uses global costmap -- why the difference?
- Smoother server has "costmap sub., footprint sub.", two bidirectional arrows to BT Nav Server...

## ROS_gz Repo
[x] Read

## Read *Benchmark of visual and 3D lidar SLAM systems in simulation environment for vineyards*
- Read the vineyard benchmark and scenario-execution paper. Decide on likely metrics: launch success, map creation success, plan success, completion rate, collisions, time-to-goal, path length, CPU/GPU notes, and qualitative failure categories.

- RTAB achieves superior performance with the lowest RMSE across all scenarios compared to other systems