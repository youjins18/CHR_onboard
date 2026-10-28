# data_logging

The experiment creates one timestamped bag under
`~/Desktop/chr_onboard_ws/src/data_logging/bags`.

```text
palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

The bag contains flight logging, PX4 status/odometry, raw OptiTrack poses,
external-vision input, and the separate J1 measured/commanded angles.

After recording has stopped cleanly, convert the bag:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

The CSV is created inside its source bag directory. Plot it with:

```matlab
Palletrone_Decryptor  % select the Palletrone CSV
```
