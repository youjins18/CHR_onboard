# CHR onboard workspace

Temporary aerial-manipulator experiment workspace combining the verified
Palletrone onboard stack, OptiTrack external position, arm hardware, and CHR
whole-body DLS kinematics.

## Process ownership

The experiment deliberately uses separate terminals instead of one monolithic
launch file:

1. Micro XRCE-DDS Agent
2. Palletrone tilt Dynamixel (`quadrotor_controller/dynamixel_fix`)
3. Palletrone flight + OptiTrack + flight bag
4. Ground-station `chr_visualization`
5. Arm Dynamixel + airborne zero capture + torque-on
6. CHR controller + CHR/arm bag
7. Keyboard EE command

`v3_optitrack_flight_launcher.launch.py` disables the legacy
`custom_trajectory_generation` node. `chr_controller` is therefore the only
publisher of PX4 custom position/velocity commands.

Arm torque belongs only to `dynamixel_arm_driver`. Starting
`arm_driver.launch.py` during stable hover captures the hanging pose as
`q=[0,0,0,0]`, writes the current position as the hold target, and then enables
torque. The controller and keyboard never enable or disable torque. Keyboard
starts with the arm/DLS experiment authorized and waits for `ARM_READY`; `M`
acts as an emergency tracking inhibit/re-enable switch without changing arm
torque.

Palletrone torque DOB is ON by default as soon as the terminal `#3` flight node
starts. Terminal `#7` preserves that state. `B` remains available as an explicit
DOB ON/OFF toggle.

## Build

```bash
cd ~/Desktop/chr_onboard_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The OptiTrack stack requires the ROS 2 branch of IMRCLab
`motion_capture_tracking`, including its submodules. See
`src/px4_mocap_bridge/README.md` for installation and PX4 EKF2 settings.

## Terminal commands

Follow [EXPERIMENT_MANUAL_KO.md](EXPERIMENT_MANUAL_KO.md) for checks and exact
timing. The process commands are:

```bash
#1
MicroXRCEAgent udp4 -p 8888

#2
ros2 run quadrotor_controller dynamixel_fix

#3
ros2 launch px4_mocap_bridge v3_optitrack_flight_launcher.launch.py \
  hostname:=192.168.0.100 rigid_body_name:=drone

#4 (GSC, in ~/Desktop/chr_gsc_ws)
ros2 launch chr_visualization chr_visualization.launch.py

#5 (only after stable hover)
ros2 launch dynamixel_arm_driver arm_driver.launch.py \
  torque_on_at_startup:=true

#6
ros2 launch chr_controller temporary_experiment.launch.py

#7
ros2 run keyboard_teleop_chr keyboard_teleop \
  --ros-args --params-file \
  ~/Desktop/chr_onboard_ws/src/keyboard_teleop_chr/config/teleop.yaml
```

When terminal `#7` starts, no additional `M` input is required. The keyboard
initializes the TCP target from feedback and authorizes the experiment. The
controller waits until all of the following are true before it automatically
enters whole-body DLS tracking:

- current odometry, vehicle status, arm-joint feedback, and TCP target
- Palletrone armed with no failsafe
- arm Dynamixel torque enabled
- stable hover for the configured duration
- a valid captured command origin

Press `M` once to inhibit DLS tracking and stop an active trajectory while
leaving arm torque enabled. Press `M` again to re-authorize automatic tracking;
the same safety gates must be satisfied again. `G` independently toggles the
Lissajous TCP trajectory, and `B` toggles the Palletrone torque DOB.

The arm feedback/command rate is 100 Hz. Limits relative to the airborne zero
pose are J1 = -90..+90 degrees, J2 = 0..30 degrees, and J3 = -90..+90 degrees.

## Recording

Both bags use `~/Desktop/chr_onboard_ws/src/data_logging/bags`:

- `palletrone_rosbag2_...`: starts in terminal #3; flight and OptiTrack data.
- `chr_rosbag2_...`: starts in terminal #6; arm, TCP, DLS, and command data.

Use `src/data_logging/rosbag_to_csv.py` on each bag separately, then use
`Palletrone_Decryptor.m` for the flight CSV and `CHR_Decryptor.m` for the CHR
CSV.

## Package ownership

- `px4_msgs`, `px4_ros_com`, and `px4-offboard`: validated Palletrone onboard
  code; upstream contributor metadata is retained.
- `px4_mocap_bridge`: OptiTrack reception and PX4 external-position bridge;
  upstream contributor metadata is retained.
- `quadrotor_controller`: Palletrone tilt Dynamixel control.
- `DynamixelSDK`: validated Palletrone workspace version.
- `dynamixel_arm_driver`: arm communication, zero capture, torque, feedback,
  and hardware joint-limit enforcement.
- `chr_controller`: frame conversion, safety gate, DLS-IK, and command routing.
- `keyboard_teleop_chr`: EE target, Lissajous trajectory, DOB, and DLS toggle.
