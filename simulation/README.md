# TidyBot2 MuJoCo Simulation

This directory contains MuJoCo simulation files for controlling the TidyBot2 mobile robot with a WX250S 6-DOF arm.

## Overview

The TidyBot2 robot consists of:
- **Mobile Base**: 3 degrees of freedom (x, y, rotation)
- **WX250S Arm**: 6 degrees of freedom (waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
- **Robotiq 2F-85 Gripper**: Parallel jaw gripper with adaptive linkage mechanism

## Installation

### Using uv (recommended)

```bash
# From the repository root
cd /path/to/collaborative-robotics-2026

# Install dependencies with uv
uv sync

# Activate the virtual environment
source .venv/bin/activate  # On Linux/macOS
# or
.venv\Scripts\activate  # On Windows
```

### Manual Installation

```bash
pip install mujoco>=3.0.0 numpy>=1.24.0
```

## Running the Simulation

```bash
cd simulation/scripts
python drive_tidybot_wx250s.py
```

This will:
1. Load the TidyBot2 robot with WX250S arm in MuJoCo
2. Open the MuJoCo viewer window
3. Drive the robot forward 1 meter
4. Perform a wave motion with the arm (5 sec raise, 5 sec wave, then return home)
5. Print progress messages to the console

Close the viewer window to exit the simulation.

## Robot Control API

### Mobile Base Control

The base has 3 position-controlled actuators:

```python
# Get actuator IDs
base_ctrl_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_x")
base_ctrl_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_y")
base_ctrl_theta = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_th")

# Set target positions (meters for x/y, radians for theta)
data.ctrl[base_ctrl_x] = 1.0      # Move to x=1.0m
data.ctrl[base_ctrl_y] = 0.5      # Move to y=0.5m
data.ctrl[base_ctrl_theta] = 0.0  # Face forward (0 radians)
```

### Arm Control

The WX250S arm has 6 position-controlled joints:

```python
# Get actuator IDs
arm_ctrl_waist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "waist")
arm_ctrl_shoulder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder")
arm_ctrl_elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "elbow")
arm_ctrl_forearm_roll = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "forearm_roll")
arm_ctrl_wrist_angle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wrist_angle")
arm_ctrl_wrist_rotate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wrist_rotate")

# Set target joint angles (radians)
data.ctrl[arm_ctrl_shoulder] = -0.5  # Raise shoulder
data.ctrl[arm_ctrl_elbow] = 1.0      # Bend elbow
data.ctrl[arm_ctrl_wrist_angle] = -0.5  # Angle wrist
```

**Joint Ranges** (from URDF):
- `waist`: -3.05 to 3.05 rad (~±175°)
- `shoulder`: -1.88 to 1.99 rad (~-108° to 114°)
- `elbow`: -2.15 to 1.61 rad (~-123° to 92°)
- `forearm_roll`: -3.05 to 3.05 rad (~±175°)
- `wrist_angle`: -1.75 to 2.15 rad (~-100° to 123°)
- `wrist_rotate`: -3.05 to 3.05 rad (~±175°)

### Gripper Control

The gripper uses a single actuator controlling both fingers:

```python
# Get gripper actuator ID
gripper_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")

# Set gripper position (0 = open, 255 = closed)
data.ctrl[gripper_ctrl] = 0      # Fully open
data.ctrl[gripper_ctrl] = 255    # Fully closed
data.ctrl[gripper_ctrl] = 128    # Half-closed
```

### Reading Joint States

```python
# Get joint IDs for reading state
base_joint_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_x")
shoulder_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder")

# Read current positions
current_x = data.qpos[base_joint_x]           # Current x position (meters)
current_shoulder = data.qpos[shoulder_joint]  # Current shoulder angle (radians)

# Read current velocities
current_x_vel = data.qvel[base_joint_x]       # Current x velocity (m/s)
```

## File Structure

```
simulation/
├── README.md                     # This file
├── scripts/
│   └── drive_tidybot_wx250s.py  # Demo control script
└── assets/
    ├── mujoco/
    │   ├── scene_wx250s.xml     # Scene configuration (floor, lighting)
    │   └── tidybot_wx250s.xml   # Robot model definition
    └── meshes/
        ├── base/                # Mobile base meshes
        ├── wx250s/              # WX250S arm segment meshes
        └── 2f85/                # Robotiq 2F-85 gripper meshes
```

## Extending the Simulation

### Adding Objects

Edit `assets/mujoco/scene_wx250s.xml` to add objects to the scene:

```xml
<worldbody>
  <!-- Existing floor and lighting -->

  <!-- Add a red cube -->
  <body name="cube" pos="0.5 0 0.1">
    <geom type="box" size="0.05 0.05 0.05" rgba="1 0 0 1"/>
  </body>
</worldbody>
```

### Creating Custom Control Scripts

Use `drive_tidybot_wx250s.py` as a template:

1. Load the model: `model = mujoco.MjModel.from_xml_path("../assets/mujoco/scene_wx250s.xml")`
2. Get actuator IDs for the joints you want to control
3. Set control targets in the main loop: `data.ctrl[actuator_id] = target_value`
4. Step the simulation: `mujoco.mj_step(model, data)`
5. Sync the viewer: `viewer.sync()`

### Implementing Inverse Kinematics

For end-effector position control, you'll need to implement inverse kinematics (IK). Consider using:
- PyKDL or IKPy for analytical/numerical IK
- MuJoCo's built-in IK functions
- Optimization-based IK using MuJoCo's Jacobian

## Troubleshooting

**Issue**: `Error opening file 'mobile_wx250s_1_base.stl'`
- **Solution**: Ensure you're running the script from `simulation/scripts/` directory

**Issue**: `ValueError: XML Error: required attribute missing`
- **Solution**: Check that all XML files have been correctly copied and paths are correct

**Issue**: Robot appears distorted or parts are misaligned
- **Solution**: Verify mesh files are all present in `simulation/assets/meshes/`

## Next Steps

- **Real Hardware Control**: See the root `README.md` for instructions on controlling the real robot
- **Advanced Control**: Implement trajectory planning, obstacle avoidance, or teleoperation
- **Data Collection**: Record demonstrations for imitation learning
- **Curriculum Development**: Create teaching materials and lab assignments

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [TidyBot2 Paper](https://arxiv.org/abs/2305.05658)
- [Interbotix WX250S Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html)
- [Robotiq 2F-85 Gripper](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
