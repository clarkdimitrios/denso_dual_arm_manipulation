# Denso VM-60B1 Dual-Arm ROS 2 + MoveIt2

This package provides ROS 2 launch files, URDF models, and Python scripts for controlling a **dual-arm Denso VM-60B1 industrial robot** in both **simulation** and **real hardware** modes using MoveIt2.  
It supports **coordinated dual-arm planning**, **waypoint-based motion execution**, and **URDF/Xacro-based robot descriptions**.

This work is based on the official [Denso Robotics ROS2 repository](https://github.com/DENSORobot/denso_robot_ros2) with significant modifications for:
- Added support for VM-60B1 robot model
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
git clone https://github.com/clarkdimitrios/denso_manipulation.git
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
To launch the dual-arm robot in **RViz** and **Gazebo**:
```bash
ros2 launch manip_facts_lab comb_multi_robot_bringup.launch.py sim:=true
```

---

### Real Hardware Mode
1. Connect both RC8A controllers and your PC to the same Ethernet switch.
2. Set IP addresses:
   - Right arm: `192.168.17.20`
   - Left arm: `192.168.17.21`
   - PC: `192.168.17.100/24` (on the same subnet, setting your PC's address as Executable Token using the Teach Pendant)
3. Set both controllers to AUTO Mode using the Teach Pendant and its special key.
4. Ensure each controller has a pendantless dummy plug or a teach pendant plugged in to stay in AUTO Mode.
5. Launch:
```bash
ros2 launch manip_facts_lab comb_multi_robot_bringup.launch.py
```

### Virtual Walls/Objects
For safety, you can add virtual walls in the robot's workspace by editing `config/virtual_walls.yaml`. This restricts MoveIt2 from planning trajectories colliding with these walls/objects.

#### Notes:
- If the hardware setup is not complete (arm calibration, wire shorts for emergency signals and auto-enable), refer to DENSO documentation.
- If virtual fences are enabled using the Teach Pendant or Wincaps III, it may restrict operations. Can easily be disabled.
- Recommended: change settings to only allow Manual operation whenever the robot collides or enters restricted areas.

---

### Waypoint Execution from CSV
```bash
ros2 launch manip_facts_lab dual_waypoints_launch csv_filename:=waypoints_0.csv
```

**Example CSV format:**
```csv
left_joint_1,left_joint_2,left_joint_3,left_joint_4,left_joint_5,left_joint_6,
right_joint_1,right_joint_2,right_joint_3,right_joint_4,right_joint_5,right_joint_6
0,0,0,0,0,0, 0,0,0,0,0,0
0.1,0.2,0,0,0,0, 0.1,0.2,-0.5,0,0,0
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
- Modifications for dual-arm planning and waypoint execution by **<clarkdimitrios>**.
- License: See [LICENSE](LICENSE).