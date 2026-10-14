# CHR 임시 비행·EE tracking 실험 운용 매뉴얼

이 매뉴얼은 LattePanda onboard workspace와 GSC의 `chr_visualization`을 사용해
Palletrone 비행 후, 공중에서 늘어진 arm 자세를 `0000`으로 잡고 DLS-IK를
시험하는 순서만 다룬다.

## 핵심 구조

| 터미널 | 실행 위치 | 담당 기능 |
|---|---|---|
| `#1` | LattePanda (SSH) | Micro XRCE-DDS Agent |
| `#2` | LattePanda (SSH) | Palletrone tilt Dynamixel |
| `#3` | LattePanda (SSH) | Palletrone flight controller, OptiTrack, flight rosbag |
| `#4` | Ground Station Computer | `chr_visualization`/RViz 감시 |
| `#5` | LattePanda (SSH) | Arm Dynamixel, 공중 zero 캡처, torque-on |
| `#6` | LattePanda (SSH) | `chr_controller`, CHR/arm rosbag |
| `#7` | LattePanda (SSH) | `keyboard_teleop_chr` |

1. `#1`, `#2`, `#3`을 먼저 실행하고 
2. `#4`로 상태를 확인한 뒤 Palletrone을
3. arm/이륙시킨다.
4. 안정 hover가 만들어진 뒤에만 `#5`, `#6`, `#7`을 차례로 실행한다.

## GSC Terminal setting 

### #1 — Micro XRCE-DDS Agent

```bash
alias sshpal='ssh palletrone@192.168.10.152' 
alias cw='source ~/Desktop/chr_onboard_ws/install/setup.bash'
alias agent='sudo chmod 666 /dev/tty* && MicroXRCEAgent serial --dev /dev/ttyPX4 -b 921600'
```

### #2 — Palletrone tilt Dynamixel

```bash
sshpal 
cw
alias dyn_fix='source /opt/ros/humble/setup.bash && source ~/chr_onboard_ws/install/local_setup.bash && ros2 run quadrotor_controller dynamixel_fix' ## 다이나믹셀
```

### #3 — Palletrone flight controller + OptiTrack + flight bag

```bash
sshpal 
cw
alias chr_pt='os2 launch px4_mocap_bridge v3_optitrack_flight_launcher.launch.py'
```

이 launch가 실행하는 것은 다음뿐이다.

- `px4_ros_com/listen_and_speak_ros`
- `motion_capture_tracking_node`
- `px4_mocap_bridge`
- Palletrone/OptiTrack rosbag

Flight bag은 3초 뒤 자동으로 시작되며 다음 위치에 저장된다.

```text
~/Desktop/chr_onboard_ws/src/data_logging/bags/
  palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

`#3` 터미널은 launch 화면을 그대로 유지한다. 다음 topic 확인 명령은 `#4`
터미널에서 RViz를 시작하기 전에 실행한다.

### #4 — GSC의 chr_visualization

이 터미널은 Onboard SSH가 아니라 GSC NUC에서 실행한다.

```bash
alias cgw='source ~/Desktop/chr_gsc_ws/install/setup.bash'  
ros2 topic hz /poses
alias chr_vis='ros2 launch chr_visualization chr_visualization.launch.py'
```

### Palletrone arm 및 hover

1. 기존에 검증한 Palletrone 절차로 arm한다.
2. 안정적으로 hover할 때까지 기다린다.
3. OptiTrack/PX4 위치가 튀거나 rigid body가 유실되면 다음 단계로 가지 않는다.

### #5 — Arm Dynamixel zero 캡처 + torque-on

```bash
sshpal 
cw
alias chr_dyn='ros2 launch dynamixel_arm_driver dynamixel_driver.launch.py'
```

실행 순간 driver가 각 모터의 현재 위치를 읽어 그 늘어진 자세를 `q=[0,0,0,0]`으로 저장하며 `/cone/torque_enabled`가 `true`이다. 

### #6 — CHR controller + arm/CHR bag

```bash
sshpal 
cw
alias chr_conall='ros2 launch chr_controller temporary_experiment.launch.py'
```

이 launch는 `chr_controller_node`와 CHR rosbag만 실행한다. 

CHR bag은 즉시 다음 위치에 만들어진다.

```text
~/Desktop/chr_onboard_ws/src/data_logging/bags/
  chr_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

Controller가 PX4 상태, odometry, zeroed joint state, torque state를 받은 뒤 hover
조건을 3초간 만족하면 `/chr/controller_state`가 `ARM_READY`가 된다. 

### #7 — Keyboard teleop

```bash
alias chr_teleop='ros2 run keyboard_teleop_chr keyboard_teleop'
```

| 기능 | 키 | 동작 |
|---|---|---|
| EE X | `W` / `S` | `+/- 0.01 m` |
| EE Y | `A` / `D` | `+/- 0.01 m` |
| EE Z | `R` / `F` | `+/- 0.01 m` |
| EE roll | `I` / `K` | `+/- 3 deg` |
| EE pitch | `J` / `L` | `+/- 3 deg` |
| 현재 TCP로 target 재설정 | `Space` | trajectory도 정지 |
| target 출력 | `P` | 상태만 출력 |
| Lissajous trajectory | `G` | ON/OFF |
| Palletrone DOB | `B` | ON/OFF |
| Arm/DLS 비상 억제/재허가 | `M` | DLS 및 trajectory 중지/재허가, torque에는 영향 없음 |


## 실험 종료 및 Bag 변환

Flight bag과 CHR bag은 별도 파일이다. 각각 변환한다.

```bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
python3 rosbag_to_csv.py bags/chr_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

파일 가져오기 
```bash
scp palletrone2@192.168.10.129:~/rosbag2_2026_01_13-23_17_19/data.csv ~/Desktop/rosbag2csv/ 
```

- Flight CSV: `matlab/Palletrone_Decryptor.m`
- CHR CSV: `matlab/CHR_Decryptor.m`


## Tree
```bash
├── chr_controller
│   ├── CMakeLists.txt
│   ├── config
│   │   └── chr_controller.yaml
│   ├── include
│   │   └── chr_controller
│   ├── launch
│   │   └── temporary_experiment.launch.py
│   ├── package.xml
│   └── src
│       ├── chr_controller_node.cpp
│       ├── px4_frame_adapter.cpp
│       └── whole_body_kinematics.cpp
├── data_logging
│   ├── bags
│   ├── matlab
│   │   ├── CHR_Decryptor.m
│   │   └── Palletrone_Decryptor.m
│   ├── __pycache__
│   │   └── rosbag_to_csv.cpython-310.pyc
│   ├── README.md
│   └── rosbag_to_csv.py
├── dynamixel_arm_driver
│   ├── CMakeLists.txt
│   ├── config
│   │   └── arm_hardware.yaml
│   ├── launch
│   │   └── arm_driver.launch.py
│   ├── package.xml
│   └── src
│       └── dynamixel_driver_node.cpp
├── DynamixelSDK
├── EXPERIMENT_MANUAL_KO.md
├── keyboard_teleop_chr
│   ├── CMakeLists.txt
│   ├── config
│   │   └── teleop.yaml
│   ├── launch
│   │   └── keyboard_teleop.launch.py
│   ├── package.xml
│   └── src
│       └── keyboard_teleop.cpp
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
│   │   ├── conversion.py
│   │   ├── __init__.py
│   │   ├── mocap_bridge_node.py
│   │   └── transforms.py
│   ├── README.md
│   ├── resource
│   │   └── px4_mocap_bridge
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_integrated_launch.py
│       ├── test_message_conversion.py
│       └── test_transforms.py
├── px4_msgs
│   ├── CMakeLists.txt
│   ├── CONTRIBUTING.md
│   ├── LICENSE
│   ├── msg
│   ├── package.xml
│   ├── README.md
│   └── srv
│       └── VehicleCommand.srv
├── px4-offboard
├── px4_ros_com
│   ├── CMakeLists.txt
│   ├── include
│   │   └── px4_ros_com
│   ├── launch
│   │   ├── offboard_control_launch.yaml
│   │   ├── sensor_combined_listener.launch.py
│   │   └── v3_flight_launcher.launch.py
│   ├── LICENSE
│   ├── package.xml
│   ├── px4_ros_com
│   │   ├── __init__.py
│   │   └── module_to_import.py
│   ├── README.md
│   ├── scripts
│   │   ├── build_all.bash
│   │   ├── build_ros2_workspace.bash
│   │   ├── __init__.py
│   │   └── setup_system.bash
│   ├── src
│   │   ├── examples
│   │   └── lib
│   └── test
│       ├── __init__.py
│       ├── pipeline_io_test.py
│       ├── test_input.py
│       └── test_output.py
├── quadrotor_controller
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       ├── dynamixel_fix.cpp
│       └── dynamixel_reboot.cpp
└── README.md
```