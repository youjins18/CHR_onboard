# CHR onboard workspace

Temporary aerial-manipulator experiment workspace combining the verified
Palletrone onboard stack with the hardware-parameter-updated CHR whole-body
DLS kinematics.

## Safety model

- The arm driver starts with torque **off**.
- `M` in `keyboard_teleop_chr` enables the arm experiment. It requests torque
  after the first hover gate, then enables Palletrone custom mode and DLS
  automatically only after the controller reports `ARM_READY`.
- Torque-on is forwarded only after PX4 is armed, feedback is fresh, no
  failsafe is present, and linear/angular speed stay under the configured
  hover thresholds for three seconds.
- After torque-on, the hover timer is cleared and a second full stable-hover
  window is required before DLS commands are accepted. The EE target is reset
  to the first measured pose in the newly zeroed arm frame.
- Mid-air torque-off is denied by default. After `M` is turned off, the arm
  holds its last command until PX4 reports disarmed; then torque is disabled.
- Stopping `dynamixel_arm_driver` normally attempts torque-off. Do not stop or
  kill that process in flight: a graceful stop drops the arm, while a crash or
  power/USB fault cannot guarantee either torque state.
- `chr_controller` must be the only publisher to PX4 custom position/velocity
  topics. Do not run `custom_trajectory_generation` or `px4-offboard` during
  this experiment.

## Build

```bash
cd ~/Desktop/chr_onboard_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Set the arm adapter's stable path in
`src/dynamixel_arm_driver/config/arm_hardware.yaml`. A
`/dev/serial/by-id/...` path is preferred. No workspace udev rule is installed,
because a generic FTDI rule can accidentally claim the Palletrone tilt-servo
adapter as the arm adapter. Arm position feedback and commands run at 100 Hz.
The enforced arm limits are J1 = -90..+90 degrees, J2 = 0..30 degrees, and
J3 = -90..+90 degrees relative to the torque-on zero pose.

## Run

현장 실험 전에는 [한국어 실험 운용 매뉴얼](EXPERIMENT_MANUAL_KO.md)을
위에서부터 체크한다.

Start the non-interactive stack. This also starts rosbag immediately and saves
the run under `src/data_logging/bags/YYYYMMDD_HHMMSS_chr`:

```bash
ros2 launch chr_controller temporary_experiment.launch.py
```

Then start the keyboard in a dedicated terminal:

```bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
ros2 run keyboard_teleop_chr keyboard_teleop \
  --ros-args --params-file \
  ~/Desktop/chr_onboard_ws/src/keyboard_teleop_chr/config/teleop.yaml
```

Recommended experiment order:

1. With propellers removed, verify the arm adapter path, Dynamixel IDs,
   directions, torque-off startup, zero capture, and all three joint limits.
2. Launch the non-interactive stack and confirm that rosbag reports recording.
   Also confirm arm torque is off, PX4/arm feedback is fresh, and no competing
   custom-command publisher (`custom_trajectory_generation` or `px4-offboard`)
   is running.
3. Start `keyboard_teleop_chr` in its own terminal. Confirm all six published
   flags are initially zero.
4. Arm and take off using the already-validated Palletrone procedure, with
   `M` and `G` still off. Establish a stable hover.
5. After `/chr/controller_state` changes from `WAITING_FOR_HOVER` to `HOLD`,
   press `M` once. The driver captures the current arm pose as zero before it
   enables torque.
6. Wait for fresh zeroed joint/EE feedback and the second full stable-hover
   gate. When the state reports `ARM_READY`, the keyboard automatically enables
   Palletrone custom mode and DLS. The initial EE target is the measured pose,
   so enabling the controller does not request a step.
7. If required for the run, enable DOB with `B`.
8. First command a very small manual EE displacement and verify base and joint
   directions. Then press `G` to run the conservative predefined EE path.
9. Press `G` again to freeze the trajectory target. Press `M` to stop DLS;
   flight offsets slew back to zero while arm torque stays on during flight.
10. Land and disarm using the normal flight procedure. The pending `M`-off
    request then disables arm torque. Confirm `/cone/torque_enabled` is false
    before stopping the keyboard and the non-interactive stack. End the launch
    with `Ctrl-C` and wait for rosbag to finish writing `metadata.yaml`.

The keyboard keeps the legacy six-field message layout as
`[DOB, custom(auto), trajectory, 0, 0, arm_experiment_request]`. Only EE xyz,
EE rpy, `Space`, `P`, `G`, `B`, and `M` are accepted as controls.

## Package ownership

- `px4_msgs` and `px4_ros_com`: copied unchanged from the validated Palletrone
  onboard code; their upstream contributor metadata is unchanged.
- `px4-offboard`: retained but not launched for this experiment.
- `quadrotor_controller`: control code is unchanged; package contributor is
  `youjins18 <chldbwls08@seoultech.ac.kr>`.
- `DynamixelSDK`: the validated Palletrone workspace version. ROBOTIS authors
  remain credited and `youjins18` is listed as an additional contributor.
- `keyboard_teleop_chr`: EE target/trajectory and experiment toggles only.
- `chr_controller`: frame conversion, safety state machine, DLS-IK, command
  slew limits, and command routing.
- `dynamixel_arm_driver`: motor communication, torque service, feedback, and
  final hardware joint-limit enforcement.
