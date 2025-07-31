# Denso VM-60B1 Dual-Arm ROS 2 + MoveIt2

This package provides ROS 2 launch files, URDF models, and Python scripts for controlling a **dual-arm Denso VM-60B1 industrial robot** in both **simulation** and **real hardware** modes using MoveIt2.  
It supports **coordinated dual-arm planning**, **waypoint-based motion execution**, and **URDF/Xacro-based robot description**.

This work is based on the official [Denso Robotics ROS2 repository](https://github.com/DENSORobot/denso_robot_ros2) with significant modifications for:
- Dual-arm robot configuration
- CSV waypoint trajectory execution
- Coordinated `dual_arm` MoveIt2 planning group
- Additional utilities for testing and simulation

---

## Features
- **Unified Dual-Arm MoveIt2 Control**: Both arms can be planned/executed together or individually.
- **Waypoint Execution**: Load joint-space waypoints from CSV files.
- **Simulation & Real Hardware Modes**: Easily switch via a launch argument.
- **Custom URDF/Xacro**: Full kinematic chain for two 6-DOF arms.

---

## Download & Installation

### 1. Clone the repository into your ROS 2 workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/<your-repo-name>.git
```

### 2. Install dependencies
```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs
pip install pymoveit2
```

### 3. Build the package
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

### Simulation Mode
To launch the dual-arm robot in **RViz**:
```bash
ros2 launch manip_facts_lab dual_arm_bringup.launch.py sim:=true
```

To launch in **Gazebo + RViz**:
```bash
ros2 launch manip_facts_lab dual_arm_gazebo.launch.py
```

---

### Real Hardware Mode
1. Connect both RC8A controllers and your PC to the same Gigabit Ethernet switch.
2. Set IP addresses:
   - Left arm: `192.168.17.21`
   - Right arm: `192.168.17.22`
   - PC: `192.168.17.100`
3. Launch:
```bash
ros2 launch manip_facts_lab dual_arm_bringup.launch.py sim:=false
```

---

### Waypoint Execution from CSV
```bash
ros2 run manip_facts_lab dual_follow_waypoints --ros-args -p csv_filename:=waypoints/example.csv
```

**Example CSV format:**
```csv
left_joint_1,left_joint_2,left_joint_3,left_joint_4,left_joint_5,left_joint_6,
right_joint_1,right_joint_2,right_joint_3,right_joint_4,right_joint_5,right_joint_6
0,0,0,0,0,0, 0,0,0,0,0,0
0.1,0.2,0,0,0,0, 0.1,0.2,0,0,0,0
```

---

## URDF & Planning Groups
- **Dual-arm group**: `dual_arm`
- **Left arm**: `left_arm`
- **Right arm**: `right_arm`
- End effectors: `left_J6`, `right_J6`

---

## Dependencies
- ROS 2 Humble or later
- [MoveIt2](https://moveit.ai/)
- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2)
- Gazebo (optional for simulation)
- Official [Denso ROS2 drivers](https://github.com/DENSORobot/denso_robot_ros2)

---

## License & Credits
- Original Denso ROS 2 repository: [DENSORobotics/ros2_denso_robot](https://github.com/DENSORobotics/ros2_denso_robot)
- Modifications for dual-arm planning and waypoint execution by **<Clark Abourjeily>**.
- License: See [LICENSE](LICENSE).