# 🌲 Research Extension: CHR – Palletrone Aerial Manipulator

> Mobile Robotics Lab (MRL), Seoul National University of Science and Technology (Seoultech)

> **Note**\
> This ROS 2 workspace extends the **Palletrone**, a fully actuated multirotor
> aerial robot, into the **Cone Harvest Robot (CHR)** by integrating a harvesting
> arm, OptiTrack-based external position feedback, whole-body DLS inverse
> kinematics, and experiment data logging.

The existing Palletrone flight controller and its validated operating procedure
are preserved as much as possible. CHR commands are added through the existing
ROS 2/PX4 interface rather than by restructuring the flight-control stack.

## Palletrone baseline

- Fully actuated configuration with four thrusters and four tilt servos
- Cascade position-attitude control architecture
- Torque disturbance observer (DOB), enabled by default for this experiment
- MOCE 2.0 model-based online center-of-mass estimation
- Micro XRCE-DDS bridge between ROS 2 topics and PX4 uORB topics

## CHR extensions

- OptiTrack rigid-body measurements forwarded to PX4 external vision
- Dynamixel arm driver with airborne zero capture and hardware joint limits
- Safety-gated manual whole-body DLS-IK for Palletrone XYZ and arm J1--J3
  commands
- TCP position/roll/pitch teleoperation plus a configurable arm-init and
  Palletrone X/Z diagonal trajectory
- Automatic custom-mode authorization after `ARM_READY`; `G` starts the
  predefined motion and `M` remains an emergency inhibit/re-enable switch
  that never changes arm torque
- Separate flight and CHR rosbags with CSV conversion and MATLAB decryptors
- Read-only RViz monitoring from the ground-station `chr_gsc_ws`

## Build

```bash
cd ~/Desktop/chr_onboard_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The OptiTrack integration requires the ROS 2 branch of IMRCLab
`motion_capture_tracking`, including its submodules. Additional setup and PX4
EKF2 requirements are documented in
[`px4_mocap_bridge/README.md`](px4_mocap_bridge/README.md).

## Experiment operation

The complete terminal layout, startup sequence, keyboard controls, rosbag
locations, shutdown procedure, and CSV/MATLAB workflow are maintained in the
Korean experiment manual:

**[EXPERIMENT_MANUAL_KO.md](EXPERIMENT_MANUAL_KO.md)**

Follow that manual in order during hardware experiments. In particular, start
the arm driver only after stable hover, keep `chr_controller` as the sole PX4
custom-command publisher, and stop motion with `M` before landing.

## Main packages

- `chr_controller`: frame conversion, safety gates, manual whole-body DLS-IK,
  configurable arm-init/base-diagonal trajectory, and command routing
- `dynamixel_arm_driver`: arm zero capture, torque ownership, feedback, and
  hardware limit enforcement
- `keyboard_teleop_chr`: TCP target, predefined trajectory toggle, DOB toggle,
  and motion emergency inhibit/re-enable
- `px4_mocap_bridge`: OptiTrack-to-PX4 external-position bridge
- `px4_ros_com`, `px4_msgs`: validated Palletrone ROS 2/PX4 interface
- `quadrotor_controller`: Palletrone tilt-servo control
- `data_logging`: bag conversion and separate Palletrone/CHR MATLAB analysis
