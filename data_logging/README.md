# data_logging

`temporary_experiment.launch.py` records the Palletrone and CHR topics
automatically. Each run creates a timestamped bag under
`~/Desktop/chr_onboard_ws/src/data_logging/bags`.

After stopping the experiment, open a new terminal and convert the bag:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/rosbag2_YYYY_MM_DD-HH_MM_SS/
```

One CSV is created inside the bag directory:

```text
bags/rosbag2_YYYY_MM_DD-HH_MM_SS/rosbag2_YYYY_MM_DD-HH_MM_SS.csv
```

Open MATLAB and run the two decryptors separately. Select the same CSV for
both scripts.

```matlab
Palletrone_Decryptor
CHR_Decryptor
```

- `Palletrone_Decryptor.m`: position/attitude, torque/DOB, motors, and servos.
- `CHR_Decryptor.m`: arm joints, TCP xyz/roll/pitch, and DLS diagnostics.

EE local-z yaw is intentionally absent from the CHR target and DLS residual.
Palletrone base yaw remains an available whole-body output coordinate.
