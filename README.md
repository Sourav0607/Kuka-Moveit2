# KUKA KR10 R1420 - MoveIt2 Integration

A complete ROS 2 workspace for simulating and controlling the KUKA KR10 R1420 industrial robot with MoveIt2 motion planning.

## Overview

This project provides a full ROS 2 implementation for the KUKA KR10 R1420 robot, including URDF description, Gazebo simulation, ros2_control integration, MoveIt2 motion planning capabilities, and manual teach mode functionality.

## Packages

- **kuka_description** - Robot URDF/xacro files, meshes, and Gazebo launch files
- **kuka_controllers** - ros2_control configuration and controller launch files with trajectory tolerances
- **kuka_moveit** - MoveIt2 configuration for motion planning and manipulation
- **kuka_remote** - Remote interface for sending task goals via action server
- **kuka_msgs** - Custom action definitions for robot tasks
- **kuka_manual_teach** - Interactive manual teach mode for saving and replaying robot poses

## Features

- Complete URDF model built from KUKA KR10 R1420 STL files
- Gazebo simulation with ros2_control integration
- MoveIt2 motion planning with RViz visualization
- Configured joint trajectory controllers with goal tolerances
- Remote interface for programmatic control via ROS 2 actions
- Manual teach mode for pose teaching and playback  

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
git clone https://github.com/Sourav0607/Kuka-Moveit2.git .
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

### Use Task Server (Remote Interface)
```bash
ros2 run kuka_remote task_server
```

### Use Manual Teach Mode
```bash
ros2 run kuka_manual_teach manual_teach_node
```

Commands in manual teach mode:
- 's' - Save current joint state from RViz interactive markers
- 'm' - Manually enter joint values
- 'space' - Execute all saved poses in sequence
- 'l' - List all saved poses
- 'c' - Clear all saved poses
- 'q' - Quit

## Quick Start

For a complete simulation with motion planning:

1. **Terminal 1** - Start Gazebo simulation:
   ```bash
   ros2 launch kuka_description gazebo.launch.py
   ```
2. **Terminal 2** - Launch controllers:
   ```bash
   ros2 launch kuka_controllers controllers.launch.py
   ```

3. **Terminal 3** - Launch MoveIt2:
   ```bash
   ros2 launch kuka_moveit moveit.launch.py
   ```


3. Use the MoveIt2 RViz interface to plan and execute trajectories, or use manual teach mode to save and replay poses

## Project Structure

```
kuka_ws/
├── src/
│   ├── kuka_description/    # URDF, meshes, Gazebo launch
│   ├── kuka_controllers/    # Controller configurations with tolerances
│   ├── kuka_moveit/         # MoveIt2 config files
│   ├── kuka_remote/         # Remote control via action server
│   ├── kuka_msgs/           # Custom action definitions
│   └── kuka_manual_teach/   # Manual teach mode node
├── build/                   # Build artifacts (ignored)
├── install/                 # Installation files (ignored)
└── log/                     # Build logs (ignored)
```

## Key Configuration Files

- `kuka_controllers/config/kuka_controllers.yaml` - Joint trajectory controller with goal tolerances
- `kuka_moveit/config/kuka.srdf` - Semantic robot description with collision pairs
- `kuka_moveit/config/kinematics.yaml` - KDL kinematics solver configuration
- `kuka_msgs/action/KukaTask.action` - Custom action definition for task execution

## Troubleshooting

### Joint State Issues
If manual teach mode cannot read joint states:
- Verify `/joint_states` topic is publishing: `ros2 topic echo /joint_states`
- Check Gazebo simulation is running
- Use 'm' command to manually enter joint values

### Execution Failures
If pose execution aborts:
- Goal tolerances are configured in `kuka_controllers/config/kuka_controllers.yaml`
- Restart MoveIt launch after modifying controller config
- Check joint limits in `kuka_moveit/config/joint_limits.yaml`

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is open-source. License details to be determined.

## Maintainer

**Sourav Hawaldar**
- Email: sourav.hawaldar@gmail.com
- GitHub: [Sourav0607](https://github.com/Sourav0607)

## Repository

[Sourav0607/Kuka-Moveit2](https://github.com/Sourav0607/Kuka-Moveit2)
