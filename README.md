# CHR onboard workspace

ROS 2 workspace for the Palletrone flight experiment with a single J1 pendulum
bar. 

## Structure

- `px4_ros_com`, `px4_msgs`: Palletrone ROS 2/PX4 interface
- `px4_mocap_bridge`: OptiTrack external-vision bridge
- `dynamixel_controller`: four tilt servos, plus optional J1 
- `keyboard_teleop_chr`: flight-mode flags; J1 trajectory flag is reserved
- `data_logging`: bag-to-CSV conversion and MATLAB plots

## Tree 

```bash
├── data_logging
│   ├── bags
│   ├── matlab
│   │   └── Palletrone_Decryptor.m
│   ├── __pycache__
│   ├── README.md
│   └── rosbag_to_csv.py
├── dynamixel_controller
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       ├── dynamixel_fix.cpp
│       ├── dynamixel_fix_with_arm.cpp
│       └── dynamixel_reboot.cpp
├── DynamixelSDK
├── keyboard_teleop_chr
│   ├── CMakeLists.txt
│   ├── config
│   │   └── teleop.yaml
│   ├── launch
│   │   └── keyboard_teleop.launch.py
│   ├── package.xml
│   └── src
│       └── keyboard_teleop.cpp
├── px4_fmu-v6c_default.px4
├── px4_mocap_bridge
│   ├── config
│   │   ├── bridge.yaml
│   │   └── optitrack.yaml
│   ├── launch
│   │   ├── bridge.launch.py
│   │   ├── optitrack_px4.launch.py
│   │   └── v3_optitrack_flight_launcher.launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── px4_mocap_bridge
│   ├── README.md
│   ├── resource
│   ├── setup.cfg
│   ├── setup.py
│   └── test
├── px4_msgs
├── px4-offboard
├── px4_ros_com
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── offboard_control_launch.yaml
│   │   ├── sensor_combined_listener.launch.py
│   │   └── v3_flight_launcher.launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── px4_ros_com
│   ├── README.md
│   ├── scripts
│   ├── src
│   │   ├── examples
│   │   │   ├── advertisers
│   │   │   ├── listen_and_speak_ros
│   │   │   ├── listeners
│   │   │   ├── offboard
│   │   │   └── offboard_py
│   │   └── lib
│   └── test
└── README.md

```
