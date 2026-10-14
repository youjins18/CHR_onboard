# data_logging

The split experiment creates two timestamped bags under
`~/Desktop/chr_onboard_ws/src/data_logging/bags`.

```text
palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/  # terminal #3
chr_rosbag2_YYYY_MM_DD-HH_MM_SS/         # terminal #6
```

The Palletrone bag contains flight logging, PX4 status/odometry, raw OptiTrack
poses, and the converted external-vision input. The CHR bag contains arm
feedback/commands, torque state, TCP target/state, controller state, DLS
logging, mode flags, and whole-body commands sent to PX4.

After both launches have stopped cleanly, convert each bag separately:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
python3 rosbag_to_csv.py bags/chr_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

Each CSV is created inside its source bag directory. Use the corresponding
MATLAB script:

```matlab
Palletrone_Decryptor  % select the Palletrone CSV
CHR_Decryptor         % select the CHR CSV
```

EE local-z yaw is intentionally absent from the CHR target and DLS residual.
Palletrone base yaw remains an available whole-body output coordinate.
