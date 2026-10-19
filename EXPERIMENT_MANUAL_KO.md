# CHR 임시 비행·arm/base trajectory 실험 운용 매뉴얼

이 매뉴얼은 LattePanda onboard workspace와 GSC의 `chr_visualization`을 사용해
Palletrone을 비행시킨 뒤 수동 EE 명령 또는 predefined arm/base trajectory를 시험하는 순서만 다룬다.

## 핵심 구조

| 터미널 | 실행 위치 | 담당 기능 |
|---|---|---|
| `#1` | LattePanda (SSH) | Micro XRCE-DDS Agent |
| `#2` | LattePanda (SSH) | OptiTrack 수신 및 PX4 mocap bridge |
| `#3` | Ground Station Computer | `chr_visualization`/RViz 감시 |
| `#4` | LattePanda (SSH) | Palletrone tilt Dynamixel |
| `#5` | LattePanda (SSH) | Palletrone flight controller, flight rosbag |
| `#6` | LattePanda (SSH) | Arm Dynamixel, 공중 zero 캡처, torque-on |
| `#7` | LattePanda (SSH) | `chr_controller`, CHR/arm rosbag |
| `#8` | LattePanda (SSH) | `keyboard_teleop_chr` |

1. `#1` Agent와 `#2` OptiTrack을 실행한다.
2. `#3` GSC RViz로 OptiTrack과 로봇 상태를 감시한다.
3. `#4` tilt Dynamixel과 `#5` flight controller를 차례로 실행한다.
4. `#3` RViz에서 상태를 다시 확인한 뒤 Palletrone을 arm/이륙시킨다.
5. 안정 hover가 만들어진 뒤에만 `#6`, `#7`, `#8`을 차례로 실행한다.

## Terminal setting 및 운용 순서

### #1 — Micro XRCE-DDS Agent

```bash
alias sshpal='ssh palletrone@192.168.10.152' 
alias cw='source ~/Desktop/chr_onboard_ws/install/setup.bash'
alias agent='sudo chmod 666 /dev/tty* && MicroXRCEAgent serial --dev /dev/ttyPX4 -b 921600'
```

### #2 — OptiTrack 수신 + PX4 mocap bridge

```bash
sshpal
cw
alias chr_opti='ros2 launch px4_mocap_bridge optitrack_px4.launch.py'
```

이 터미널은 `motion_capture_tracking_node`와 `px4_mocap_bridge`만 실행한다.
Motive의 rigid body pose를 수신하고 PX4용 mocap 메시지로 변환한다. 

필요하면 별도 SSH 터미널에서 수신 주기를 확인한다.

```bash
cw
ros2 topic hz /poses
```

### #3 — GSC의 chr_visualization

이 터미널은 Onboard SSH가 아니라 GSC NUC에서 실행한다.

```bash
alias cgw='source ~/Desktop/chr_gsc_ws/install/setup.bash'
alias chr_vis='ros2 launch chr_visualization chr_visualization.launch.py'
```

### #4 — Palletrone tilt Dynamixel

```bash
sshpal
cw
alias chr_fix='ros2 run quadrotor_controller dynamixel_fix'
```

### #5 — Palletrone flight controller + flight bag

```bash
sshpal
cw
alias chr_pt='ros2 launch px4_mocap_bridge v3_optitrack_flight_launcher.launch.py launch_optitrack:=false'
```

`launch_optitrack:=false`가 중요하다. `#2`에서 OptiTrack 노드를 이미 따로
실행했으므로, 이 launch에서는 OptiTrack을 중복 실행하지 않는다. (추후 코드 수정 예정)

이 launch가 실행하는 주요 항목은 다음과 같다.

- `px4_ros_com/listen_and_speak_ros`
- Palletrone flight rosbag

Flight bag은 3초 뒤 자동으로 시작되며 다음 위치에 저장된다.

```text
~/Desktop/chr_onboard_ws/src/data_logging/bags/
  palletrone_rosbag2_YYYY_MM_DD-HH_MM_SS/
```

### Palletrone arm 및 hover

1. 기존에 검증한 Palletrone 절차로 arm한다.
2. 안정적으로 hover할 때까지 기다린다.

### #6 — Arm Dynamixel zero 캡처 + torque-on

```bash
sshpal 
cw
alias chr_dyn='ros2 launch dynamixel_arm_driver arm_driver.launch.py'
```

실행 순간 driver가 각 모터의 현재 위치를 읽어 그 늘어진 자세를 `q=[0,0,0,0]`으로 저장하며 `/cone/torque_enabled`가 `true`이다. 

필요하면 별도 SSH 터미널에서 arm feedback/command가 갱신되는지 확인한다.

```bash
cw
ros2 topic echo /cone/joint_states
ros2 topic echo /cone/joint_commands
```

### #7 — CHR controller + arm/CHR bag

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

### #8 — Keyboard teleop

```bash
sshpal
cw
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
| Predefined arm-init/base-diagonal trajectory | `G` | ON / 현재 명령에서 HOLD |
| Palletrone DOB | `B` | ON/OFF |
| Arm/motion 비상 억제/재허가 | `M` | motion 및 trajectory 중지/재허가, torque에는 영향 없음 |


`G ON` 동작 순서는 다음과 같으며, predefined trajectory의 기본값과 튜닝 위치는
`chr_controller/config/chr_controller.yaml`이다.

1. arm을 config의 `trajectory_init_pose_deg`로 이동
2. init pose 도달 및 settle 확인
3. DLS-IK를 사용하지 않고 base X/Z predefined 왕복운동 시작
4. `G OFF` 시 해당 순간의 base/arm command를 hold

수동 EE 위치·자세 키는 기존처럼 DLS-IK 입력에 사용된다. 

## 실험 종료 및 Bag 변환

Flight bag과 CHR bag은 별도 파일이다. 각각 변환한다.

```bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/palletrone_rosbag2_2026_10_19-19_51_15/
python3 rosbag_to_csv.py bags/chr_rosbag2_2026_10_19-19_51_18/
```

파일 가져오기 
```bash
scp palletrone@192.168.10.152:~/Desktop/chr_onboard_ws/src/data_logging/bags/chr_rosbag2_2026_10_19-19_51_18/chr_rosbag2_2026_10_19-19_51_18.csv ~/Desktop/chr_onboard_bags/
scp palletrone@192.168.10.152:~/Desktop/chr_onboard_ws/src/data_logging/bags/palletrone_rosbag2_2026_10_19-19_51_15/palletrone_rosbag2_2026_10_19-19_51_15.csv ~/Desktop/chr_onboard_bags/
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
│   │       ├── px4_frame_adapter.hpp
│   │       └── whole_body_kinematics.hpp
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
│   │       └── frame_transforms.h
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
│   │   │   ├── advertisers
│   │   │   ├── listen_and_speak_ros
│   │   │   ├── listeners
│   │   │   ├── offboard
│   │   │   └── offboard_py
│   │   └── lib
│   │       └── frame_transforms.cpp
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
