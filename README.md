# KUKA KR10 R1420 - MoveIt2 Integration with Vision & Gripper

A complete ROS 2 workspace for simulating and controlling the KUKA KR10 R1420 industrial robot with MoveIt2 motion planning, 2-finger parallel gripper, RGB camera, and vision-based box detection.

## Overview

This project provides a full ROS 2 implementation for the KUKA KR10 R1420 robot, featuring:
- Complete robot description with 2-finger parallel gripper (mimic joints)
- Gazebo Ignition simulation with physics and sensors
- MoveIt2 motion planning for arm and gripper control
- RGB camera integration for object detection
- OpenCV-based color detection for red, green, and blue boxes
- Action server for task execution
- Manual teach mode for pose programming

## Packages

- **kuka_description** - Robot URDF/xacro files with gripper, meshes, and Gazebo world files
- **kuka_controllers** - ros2_control configuration for arm and gripper controllers
- **kuka_moveit** - MoveIt2 configuration with separate planning groups for arm and gripper
- **kuka_remote** - Task server (action server) and box detector node
- **kuka_msgs** - Custom action definitions (KukaTask)
- **kuka_manual_teach** - Interactive teach pendant for saving and replaying poses
- **models** - Gazebo models (colored boxes, tables, RGB camera)

## Features

### Robot & Gripper
- Complete URDF model built from KUKA KR10 R1420 STL files
- 2-finger parallel gripper with mimic joints 
- Separate MoveIt planning groups: `arm` (6-DOF) and `gripper` (2-finger)
- Named gripper states: `gripper_open` (0.0) and `gripper_closed` (0.06)

### Simulation
- Gazebo Ignition 6.16.0 simulation environment
- Custom world with 6 tables and 3 colored boxes (red, green, blue)
- ros2_control integration with JointTrajectoryController
- Physics simulation with realistic dynamics

### Vision System
- RGB camera (640x480, 60° FOV) mounted 1m above workspace
- Real-time image streaming via ROS 2 topics (`/camera`, `/camera_info`)
- OpenCV-based HSV color detection for box identification
- Bounding box visualization with OpenCV window
- JSON detection output to `/box_detections` topic

### Motion Planning
- MoveIt2 with OMPL and CHOMP planners
- KDL kinematics solver
- Collision checking and scene management
- RViz interactive markers for manual planning
- Configurable joint trajectory tolerances

### Control Interfaces
- **Task Server**: Action server for executing predefined arm poses
- **Manual Teach**: Interactive mode for teaching poses via keyboard
- **MoveGroupInterface**: Programmatic C++ API for motion control

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04)
- **MoveIt2**
- **Gazebo Ignition** (tested with 6.16.0)
- **ros2_control** and controller packages
- **OpenCV** and **cv_bridge** for vision
- **Python 3** with numpy

```bash
# Install ROS 2 Humble dependencies
sudo apt install ros-humble-moveit \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ros-gz-bridge \
                 ros-humble-cv-bridge \
                 python3-opencv
```

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
mkdir -p ~/kuka_ws/src
cd ~/kuka_ws/src
git clone https://github.com/Sourav06072417/Kuka-Moveit2.git .
```

2. Install dependencies:
```bash
cd ~/kuka_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

### Standard Workflow (3 Terminals)

**Terminal 1** - Launch Gazebo Simulation:
```bash
cd ~/kuka_ws
source install/setup.bash
ros2 launch kuka_moveit gazebo.launch.py
```
This starts Gazebo with the robot, gripper, tables, colored boxes, and RGB camera.

**Terminal 2** - Launch Controllers:
```bash
cd ~/kuka_ws
source install/setup.bash
ros2 launch kuka_moveit controllers.launch.py
```
This spawns the arm and gripper trajectory controllers.

**Terminal 3** - Launch MoveIt2 with RViz:
```bash
cd ~/kuka_ws
source install/setup.bash
ros2 launch kuka_moveit moveit.launch.py
```
This starts MoveIt2 planning pipeline and RViz visualization.

### Vision System (Optional 4th Terminal - Work under progress)

Run the box detector to identify colored boxes:
```bash
cd ~/kuka_ws
source install/setup.bash
ros2 run kuka_remote box_detector.py
```

This will:
- Subscribe to `/camera` topic
- Detect red, green, and blue boxes using HSV color filtering
- Publish detections to `/box_detections` topic (JSON format)
- Display OpenCV window with bounding boxes and labels

**Detection Output Format:**
```json
{
  "red": {"x": 377, "y": 240, "width": 50, "height": 52, "center_x": 402, "center_y": 266, "area": 2600},
  "green": {"x": 320, "y": 240, ...},
  "blue": {"x": 265, "y": 240, ...}
}
```

### Task Server (Action Interface)

Run predefined tasks via action server:
```bash
# Terminal 4
ros2 launch kuka_remote remote_interface.launch.py

# Terminal 5 - Send task commands ( Use gripper planning from RVIZ to pick objects make sure to use ONPL planner)
ros2 action send_goal /task_server kuka_msgs/action/KukaTask "{task_number: 0}"  # Home position
ros2 action send_goal /task_server kuka_msgs/action/KukaTask "{task_number: 1}"  # Pick position
ros2 action send_goal /task_server kuka_msgs/action/KukaTask "{task_number: 2}"  # Another position
```

### Manual Teach Mode

Interactive pose teaching and playback:
```bash
ros2 run kuka_manual_teach manual_teach_node --ros-args -p use_sim_time:=true
```

**Commands:**
- `s` - Save current joint state (move robot in RViz using interactive markers first)
- `m` - Manually enter joint values (6 joint angles in radians)
- `space` - Execute all saved poses in sequence
- `l` - List all saved poses with joint values
- `c` - Clear all saved poses
- `q` - Quit

**Workflow:**
1. Use RViz interactive markers to move the robot to desired pose
2. Press `s` to save the current position
3. Repeat for multiple waypoints
4. Press `space` to execute the entire trajectory

## Quick Start Example

```bash
# Terminal 1
ros2 launch kuka_moveit gazebo.launch.py

# Terminal 2 (wait for Gazebo to load)
ros2 launch kuka_moveit controllers.launch.py

# Terminal 3 (wait for controllers)
ros2 launch kuka_moveit moveit.launch.py

# Terminal 4 (optional - vision)
ros2 run kuka_remote box_detector.py

# Now use RViz to plan and execute motions!
```

## Project Structure

```
kuka_ws/
├── src/
│   ├── kuka_description/
│   │   ├── urdf/              # Robot URDF with gripper
│   │   ├── meshes/            # STL files for visualization
│   │   ├── worlds/            # kuka_world.sdf (tables, boxes, camera)
│   │   └── launch/            # gazebo.launch.py
│   ├── kuka_controllers/
│   │   ├── config/            # Controller YAML (arm + gripper)
│   │   └── launch/            # controllers.launch.py
│   ├── kuka_moveit/
│   │   ├── config/
│   │   │   ├── kuka.srdf      # Semantic description (arm + gripper groups)
│   │   │   ├── kinematics.yaml
│   │   │   └── joint_limits.yaml
│   │   └── launch/            # moveit.launch.py
│   ├── kuka_remote/
│   │   ├── src/               # task_server.cpp
│   │   ├── scripts/           # box_detector.py
│   │   └── launch/            # remote_interface.launch.py
│   ├── kuka_msgs/
│   │   └── action/            # KukaTask.action
│   ├── kuka_manual_teach/
│   │   └── src/               # manual_teach.cpp
│   └── models/
│       ├── RedBox/            # Red box model (0.05x0.05x0.05m)
│       ├── GreenBox/          # Green box model
│       ├── BlueBox/           # Blue box model
│       ├── Table/             # Table model
│       └── RGBCamera/         # Camera sensor model
├── build/                     # Build artifacts (gitignored)
├── install/                   # Installation files (gitignored)
└── log/                       # Build logs (gitignored)
```

## Key Configuration Files

### MoveIt Configuration
- `kuka.srdf` - Defines planning groups (`arm`, `gripper`), gripper states, collision pairs
- `kinematics.yaml` - KDL solver config for arm group
- `joint_limits.yaml` - Velocity and acceleration limits

### Controllers
- `kuka_controllers.yaml` - Joint trajectory controller with goal tolerances
  - `arm_controller`: Controls 6 arm joints
  - `gripper_controller`: Controls gripper fingers

### Vision Parameters
- Camera: 640x480 resolution, 60° FOV, 30 FPS
- HSV Ranges:
  - Red: H: 0-10 / 160-180, S: 100-255, V: 70-255
  - Green: H: 40-80, S: 100-255, V: 70-255
  - Blue: H: 90-130, S: 100-255, V: 70-255

### World Setup
- Robot spawn: (0.5, 0.0, 1.0)
- Tables: 6 tables arranged in 2x3 grid
- Boxes: 3 colored boxes at (1.5, -0.15/0.0/0.15, 1.025)
- Camera: Positioned at (1.5, 0.0, 2.025) pointing downward

## Topics & Services

### Published Topics
- `/joint_states` - Current robot joint positions
- `/camera` - RGB camera images (sensor_msgs/Image)
- `/camera_info` - Camera calibration info
- `/box_detections` - JSON string with detected box positions

### Action Servers
- `/task_server` - Execute predefined tasks (kuka_msgs/action/KukaTask)

### Planning Groups
- `arm` - 6-DOF arm (joint_a1 through joint_a6)
- `gripper` - 2-finger gripper (gripper_left_finger_joint, gripper_right_finger_joint)

## Troubleshooting

### Manual Teach Node - Clock Synchronization Error
**Error:** `Didn't receive robot state with recent timestamp`

**Solution:** Run with simulation time enabled:
```bash
ros2 run kuka_manual_teach manual_teach_node --ros-args -p use_sim_time:=true
```

### Gripper Not Moving
- Verify gripper controller is loaded: `ros2 control list_controllers`
- Check gripper joints in planning group: `gripper_left_finger_joint`, `gripper_right_finger_joint`
- Gripper range: 0.0 (open) to 0.06 (closed)

### Camera Not Publishing
- Check ros_gz_bridge is running: `ros2 node list | grep bridge`
- Verify camera topic: `ros2 topic hz /camera`
- Ensure Gazebo sensors plugin is loaded in world file

### Box Detector Not Finding Boxes
- Check camera image: `ros2 run rqt_image_view rqt_image_view /camera`
- Verify lighting conditions in Gazebo
- Adjust HSV ranges in `box_detector.py` if needed
- Ensure OpenCV window appears (may need X11 forwarding if remote)

### MoveIt Planning Failures
- Increase planning time in RViz
- Check for collision objects blocking path
- Verify joint limits in `joint_limits.yaml`
- Try different planner (OMPL RRTConnect vs CHOMP)

### Controller Execution Failures
**Error:** `Aborted: Trajectory execution failed`

**Solution:** Adjust goal tolerances in `kuka_controllers/config/kuka_controllers.yaml`:
```yaml
constraints:
  goal_time: 2.0
  joint_a1: {goal: 0.05}
  # ... increase tolerance values
```

## Technical Details

### Gripper Implementation
- **Type:** 2-finger parallel gripper
- **Actuation:** Mimic joints (right finger mirrors left)
- **Joint:** `gripper_left_finger_joint` (primary), `gripper_right_finger_joint` (mimic)
- **Range:** 0.0 to 0.06 meters
- **Control:** Separate trajectory controller (`gripper_controller`)

### Vision Pipeline
1. Gazebo renders camera images (640x480 @ 30Hz)
2. ros_gz_bridge transfers images to ROS 2 (`/camera` topic)
3. box_detector.py processes images:
   - Convert BGR to HSV color space
   - Apply color masks for red/green/blue
   - Morphological operations (erosion + dilation)
   - Find contours and bounding boxes
   - Filter by minimum area (100 pixels)
4. Publish detections as JSON string
5. Display annotated image in OpenCV window

### Coordinate Systems
- **World Frame:** Fixed reference frame
- **Camera Frame:** Origin at (1.5, 0.0, 2.025), pointing down (-Z)
- **Box Positions:** y-spacing of 0.15m, z-height of 1.025m (table surface)

## Development

### Adding New Tasks
Edit `kuka_remote/src/task_server.cpp`:
```cpp
if (goal_handle->get_goal()->task_number == 3) {
  arm_joint_goal_ = {j1, j2, j3, j4, j5, j6};
  // gripper_joint_goal_ = {0.0};  // Optional gripper control
}
```

### Modifying Detection Colors
Edit HSV ranges in `kuka_remote/scripts/box_detector.py`:
```python
lower_color = np.array([H_min, S_min, V_min])
upper_color = np.array([H_max, S_max, V_max])
```

### Changing Robot Spawn Position
Edit `kuka_description/launch/gazebo.launch.py`:
```python
arguments=["-x", "0.5", "-y", "0.0", "-z", "1.0"]
```

## Contributing

Contributions are welcome! Areas for enhancement:
- [ ] Automated pick-and-place pipeline
- [ ] Depth camera integration (RGBD)
- [ ] Gripper force control
- [ ] Additional box colors and shapes
- [ ] Trajectory optimization
- [ ] ROS 2 Iron/Jazzy compatibility

Please submit issues or pull requests on GitHub.

## License

This project is open-source under the MIT License.

## Contact

**Sourav Hawaldar**
- GitHub: [Sourav0607](https://github.com/Sourav0607)
- Repository: [Kuka-Moveit2](https://github.com/Sourav0607/Kuka-Moveit2)

## Citation

If you use this project in your research, please cite:
```
@software{kuka_moveit2_2025,
  author = {Hawaldar, Sourav},
  title = {KUKA KR10 R1420 - MoveIt2 Integration with Vision & Gripper},
  year = {2025},
  url = {https://github.com/Sourav0607/Kuka-Moveit2}
}
```

---

**Last Updated:** November 2025  
**ROS 2 Version:** Humble  

