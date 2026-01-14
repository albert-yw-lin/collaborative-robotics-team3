# Collaborative Robotics 2026

This repository contains software for controlling the **TidyBot2** mobile robot with bimanual **WX200** 6-DOF arms, developed for Professor Monroe Kennedy's 2026 Collaborative Robotics Class.

## Installation (Ubuntu 22.04)

### 1. Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install required ROS2 packages
sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher ros-humble-rviz2

# Install colcon build tools
sudo apt install -y python3-colcon-common-extensions
```

### 2. Install uv (Python Package Manager)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc  # or restart your terminal
```

### 3. Install System Dependencies

```bash
sudo apt install -y libgl1-mesa-dev libglfw3-dev libegl1-mesa-dev
```

### 4. Clone and Setup

```bash
# Clone the repository
git clone https://github.com/armlabstanford/collaborative-robotics-2026.git
cd collaborative-robotics-2026

# Install Python dependencies
uv sync

# Build ROS2 workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Quick Start

### Option 1: Standalone MuJoCo Simulation (No ROS2)

```bash
cd simulation/scripts

# Bimanual arm demo with camera control
uv run python test_move.py

# Object manipulation demo
uv run python pick_up_block.py
```

### Option 2: Full ROS2 Simulation

**Terminal 1 - Launch simulation:**
```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

This opens RViz2 and MuJoCo viewer.

**Terminal 2 - Run test scripts:**
```bash
cd ros2_ws
source setup_env.bash

# Test base movement
ros2 run tidybot_bringup test_base.py

# Test bimanual arms
ros2 run tidybot_bringup test_arms.py

# Test camera pan-tilt
ros2 run tidybot_bringup test_camera.py

# Advanced state machine example
ros2 run tidybot_bringup example_state_machine.py
```

**Launch Options:**
```bash
# Disable RViz
ros2 launch tidybot_bringup sim.launch.py use_rviz:=false

# Disable MuJoCo viewer
ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=false
```

## Repository Structure

```
collaborative-robotics-2026/
├── simulation/                  # Standalone MuJoCo simulation
│   ├── scripts/                 # test_move.py, pick_up_block.py
│   └── assets/                  # MuJoCo models and meshes
│
└── ros2_ws/                     # ROS2 workspace
    ├── setup_env.bash           # Environment setup script
    └── src/
        ├── tidybot_bringup/     # Launch files & test scripts
        ├── tidybot_description/ # URDF/XACRO robot model
        ├── tidybot_msgs/        # Custom ROS2 messages
        ├── tidybot_mujoco_bridge/  # MuJoCo-ROS2 bridge
        └── tidybot_control/     # Arm/base controllers
```

## Robot Specifications

**TidyBot2** is a mobile manipulation platform consisting of:
- **Mobile Base**: Kobuki or Create3 base with 3 DOF (x, y, theta)
- **Arms**: 2x WX200 5-DOF manipulators (550mm reach, 200g payload)
  - Waist (base rotation): ±175°
  - Shoulder (lift): -108° to 114°
  - Elbow (bend): -123° to 92°
  - Wrist angle: -100° to 123°
  - Wrist rotate: ±175°
- **Grippers**: 2x Robotiq 2F-85 adaptive parallel jaw (85mm max opening)
- **Camera**: Pan-tilt RealSense D435 (RGB + Depth)

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Base velocity |
| `/left_arm/command` | ArmCommand | Left arm control |
| `/right_arm/command` | ArmCommand | Right arm control |
| `/left_gripper/command` | GripperCommand | Left gripper |
| `/right_gripper/command` | GripperCommand | Right gripper |
| `/camera/pan_tilt` | PanTilt | Camera orientation |
| `/joint_states` | sensor_msgs/JointState | Joint feedback |

## Troubleshooting

**MuJoCo rendering issues:**
```bash
sudo apt install -y mesa-utils
glxinfo | grep "OpenGL version"
```

**Python import errors:**
```bash
# Always source environment first
source ros2_ws/setup_env.bash
```

**colcon build fails:**
```bash
# Ensure ROS2 is sourced before building
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build
```

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [uv Documentation](https://docs.astral.sh/uv/)
- [TidyBot Paper](https://arxiv.org/abs/2305.05658)
- [TidyBot2 Paper](https://arxiv.org/pdf/2412.10447)
- [Interbotix WX200 Specs](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx200.html)

## Authors

Alex Qiu & Matt Strong - Stanford ARM Lab
