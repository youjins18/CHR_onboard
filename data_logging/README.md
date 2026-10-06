# CHR data logging

`temporary_experiment.launch.py` starts rosbag immediately by default. It saves
each run as `data_logging/bags/YYYYMMDD_HHMMSS_chr` and records the topics shown
in the standalone command below. End the launch with `Ctrl-C` and wait until
rosbag finishes writing `metadata.yaml`.

Override the automatic name when a descriptive run ID is useful:

```bash
ros2 launch chr_controller temporary_experiment.launch.py \
  bag_name:=20261007_run01_hover_ee
```

For a standalone recording without the experiment launch, run the following
from the Git/workspace source root. `ros2 bag record` refuses to overwrite an
existing output directory.

```bash
cd ~/Desktop/chr_onboard_ws/src
RUN_NAME=20261007_run01_hover_ee

ros2 bag record -o "data_logging/bags/${RUN_NAME}" \
  /data_logging_msg /chr/data_logging_msg \
  /fmu/out/vehicle_odometry /fmu/out/vehicle_status \
  /cone/joint_states /cone/joint_commands /chr/state/tcp_pose \
  /chr/target/tcp_pose /chr/controller_state /cone/torque_enabled \
  /control_mode_flags /fmu/in/custom_command_position_mode \
  /fmu/in/custom_command_velocity_mode
```

Do not close the terminal or kill the process before rosbag exits cleanly.

Convert the two array topics into a run-specific CSV directory:

```bash
cd ~/Desktop/chr_onboard_ws/src
RUN_NAME=20261007_run01_hover_ee  # Use the bag directory name created by launch.

python3 data_logging/rosbag_to_csv.py \
  "data_logging/bags/${RUN_NAME}" \
  --output "data_logging/csv/${RUN_NAME}"
```

`Palletrone_Decryptor.m` is the supplied legacy decoder and remains unchanged.
`CHR_Decryptor.m` accepts the generated `chr_data_logging_msg.csv`.

## `/chr/data_logging_msg` schema

All offsets below are zero-based `data[]` indices.

| Index | Value |
|---:|---|
| 0 | controller state code |
| 1..7 | armed, hover stable, torque request, torque enabled, custom mode, trajectory, DOB |
| 8..10 | measured base position ENU [m] |
| 11 | measured base yaw ENU [rad] |
| 12..14 | commanded base position ENU [m] |
| 15 | commanded base yaw ENU [rad] |
| 16..18 | measured J1..J3 [rad] |
| 19..21 | commanded J1..J3 [rad] |
| 22..24 | measured TCP position ENU [m] |
| 25..28 | measured TCP quaternion x,y,z,w |
| 29..31 | target TCP position ENU [m] |
| 32..35 | target TCP quaternion x,y,z,w |
| 36..41 | position residual, orientation residual, min singular value, condition number, iterations, converged |
| 42..45 | PX4 NED position/yaw offset command |
| 46..48 | feedback fresh, target fresh, tracking active |
| 49..51 | base velocity ENU [m/s] |
| 52 | angular velocity norm [rad/s] |

The current publisher emits 53 values (`data[0]` through `data[52]`).
