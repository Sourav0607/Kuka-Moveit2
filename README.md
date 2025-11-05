# KUKA KR10 R1420 - MoveIt2 Integration

A complete ROS 2 workspace for simulating and controlling the KUKA KR10 R1420 industrial robot with MoveIt2 motion planning.

## Overview

This project provides a full ROS 2 implementation for the KUKA KR10 R1420 robot, including URDF description, Gazebo simulation, ros2_control integration, and MoveIt2 motion planning capabilities.

## Packages

- **`kuka_description`** - Robot URDF/xacro files, meshes, and Gazebo launch files
- **`kuka_controllers`** - ros2_control configuration and controller launch files
- **`kuka_moveit`** - MoveIt2 configuration for motion planning and manipulation
- **`kuka_remote`** - Remote interface for sending goals and controlling the robot

## Features

✅ Complete URDF model built from KUKA KR10 R1420 STL files  
✅ Gazebo simulation with ros2_control integration  
✅ MoveIt2 motion planning with RViz visualization  
✅ Configured joint trajectory controllers  
✅ Remote interface for programmatic control  

## Prerequisites

- **ROS 2** (Humble or Iron recommended)
- **MoveIt2**
- **Gazebo** (Ignition/Gazebo for Iron+)
- **ros2_control** and controller packages

```bash
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
```

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
mkdir -p ~/kuka_ws/src
cd ~/kuka_ws/src
git clone https://github.com/Toxic2417/Kuka-Moveit2.git .
```

2. Install dependencies:
```bash
cd ~/kuka_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Gazebo Simulation
```bash
ros2 launch kuka_description gazebo.launch.py
```

### Launch Controllers
```bash
ros2 launch kuka_controllers controllers.launch.py
```

### Launch MoveIt2 Planning
```bash
ros2 launch kuka_moveit moveit.launch.py
```

### Use Remote Interface
```bash
ros2 launch kuka_remote remote_interface.launch.py
```

## Quick Start

For a complete simulation with motion planning:

1. **Terminal 1** - Start Gazebo simulation:
   ```bash
   ros2 launch kuka_description gazebo.launch.py
   ```

2. **Terminal 2** - Launch MoveIt2:
   ```bash
   ros2 launch kuka_moveit moveit.launch.py
   ```

3. Use the MoveIt2 RViz interface to plan and execute trajectories

## Project Structure

```
kuka_ws/
├── src/
│   ├── kuka_description/    # URDF, meshes, Gazebo launch
│   ├── kuka_controllers/    # Controller configurations
│   ├── kuka_moveit/         # MoveIt2 config files
│   └── kuka_remote/         # Remote control interface
├── build/                   # Build artifacts (ignored)
├── install/                 # Installation files (ignored)
└── log/                     # Build logs (ignored)
```

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is open-source. Please check individual package licenses for details.

## Maintainer

**Sourav** - sourav.hawaldar@gmail.com

---

**Repository:** [Sourav0607/Kuka-Moveit2](https://github.com/Sourav0607/Kuka-Moveit2.git)
