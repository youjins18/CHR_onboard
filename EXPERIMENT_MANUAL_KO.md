# CHR 임시 비행·EE tracking 실험 운용 매뉴얼

이 문서는 Palletrone에 CHR arm을 장착한 임시 DLS-IK 실험의 현장용
체크리스트다. 위에서 아래 순서대로 진행하고, 각 확인 항목을 건너뛰지
않는다.

## 절대 원칙

- 비행 중 `dynamixel_arm_driver` 또는 전체 launch를 종료하지 않는다.
- `M OFF`는 비행 중 DLS를 즉시 정지하지만, arm torque는 기체가 disarm될
  때까지 유지된다.
- 전체 launch의 `Ctrl-C` 종료는 착륙, disarm, arm torque-off 확인 뒤에만
  수행한다.
- `/chr/controller_state`가 `FAULT`이거나 feedback/통신 오류가 반복되면
  trajectory를 시작하지 않는다.
- 기존 `custom_trajectory_generation`과 `px4-offboard`는 실행하지 않는다.
  `chr_controller`만 PX4 custom command를 발행해야 한다.
- 실제 비행 전 첫 확인은 반드시 프로펠러를 제거하거나 기체를 안전하게
  구속한 지상 상태에서 수행한다.

## 1. 실험 전 지상 점검

### 기구·전원·통신

- [ ] Arm 체결부, EE, 각 링크와 케이블에 풀림이나 간섭이 없다.
- [ ] Arm이 torque-off 상태에서 수직으로 자연스럽게 매달리며 시작 자세가
      의도한 `0000` 자세와 일치한다.
- [ ] Arm Dynamixel USB와 Palletrone tilt-servo USB를 혼동하지 않았다.
- [ ] `arm_hardware.yaml`의 `usb_port`, baudrate, ID가 실제 하드웨어와 맞다.
- [ ] 배터리, flight controller, onboard computer 전원 상태가 정상이다.
- [ ] `data_logging/bags`가 있는 디스크에 실험 전체를 기록할 여유 공간이
      있다.

디스크 확인:

```bash
df -h ~/Desktop/chr_onboard_ws/src/data_logging/bags
```

### 지상 arm 확인 — 프로펠러 제거 상태

- [ ] Driver 시작 시 arm torque가 OFF다.
- [ ] Dynamixel ID 1~4 feedback이 모두 들어온다.
- [ ] Torque-on 때 현재 매달린 자세가 zero로 캡처된다.
- [ ] J1은 `-90~+90 deg`, J2는 `0~30 deg`, J3는 `-90~+90 deg` 범위를
      넘지 않는다.
- [ ] J1/J2/J3 명령 방향과 실제 회전 방향이 예상과 일치한다.
- [ ] 통신 timeout, sync-read, sync-write 오류가 반복되지 않는다.

하나라도 만족하지 않으면 비행 실험으로 넘어가지 않는다.

## 2. 터미널 준비

실험 중에는 최소 세 개의 터미널을 유지한다.

### 터미널 1 — 전체 stack과 자동 rosbag

```bash
cd ~/Desktop/chr_onboard_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch chr_controller temporary_experiment.launch.py
```

launch 직후 확인:

- [ ] rosbag이 `src/data_logging/bags/rosbag2_YYYY_MM_DD-HH_MM_SS`에 기록을
      시작했다.
- [ ] `listen_and_speak_ros`, tilt controller, arm driver, `chr_controller`가
      오류 없이 시작했다.
- [ ] Arm torque는 OFF다.
- [ ] Dynamixel 통신 오류가 반복되지 않는다.
- [ ] `/chr/controller_state`가 처음에는 `WAITING_FOR_FEEDBACK` 또는
      `WAITING_FOR_HOVER`로 표시된다.

실제 생성된 bag 폴더 이름을 실험 노트의 run 이름으로 기록한다.

### 터미널 2 — Keyboard

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
ros2 run keyboard_teleop_chr keyboard_teleop \
  --ros-args --params-file \
  ~/Desktop/chr_onboard_ws/src/keyboard_teleop_chr/config/teleop.yaml
```

시작 직후 아무 키도 누르지 않는다. EE 측정 pose가 들어와 target이
초기화됐다는 로그를 확인한다.

### 터미널 3 — 상태 감시

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
ros2 topic echo /chr/controller_state
```

필요하면 별도 터미널에서 torque 상태도 확인한다.

```bash
ros2 topic echo /cone/torque_enabled
```

## 3. Controller 상태 의미

| 상태 | 의미 | 조치 |
|---|---|---|
| `WAITING_FOR_FEEDBACK` | PX4 status/odometry 또는 arm feedback이 없거나 오래됨 | 이륙/trajectory 진행 금지, 통신 확인 |
| `WAITING_FOR_HOVER` | 기체가 armed 상태가 아니거나 hover 안정 조건을 아직 만족하지 못함 | 조종 입력을 줄이고 안정화 대기 |
| `WAITING_FOR_TORQUE` | Torque 요청은 들어왔지만 driver torque-on 완료 전 | 추가 키 입력 없이 대기 |
| `HOLD` | 첫 hover gate 통과, torque 요청 전 | 이 상태에서만 첫 `M` 입력 |
| `ARM_READY` | Torque-on 후 두 번째 hover gate 통과 | Keyboard가 custom mode와 DLS를 자동 활성화하며 매우 짧게 보일 수 있음 |
| `TRACKING` | Whole-body DLS가 EE target을 추종 중 | 수동 EE 또는 `G` trajectory 가능 |
| `FAULT` | PX4 failsafe 또는 controller 안전 조건 위반 | `G`/`M`을 끄고 정상 착륙, 원인 확인 |

Hover gate 기준은 선속도 `0.15 m/s` 이하, 각속도 `0.20 rad/s` 이하가
3초 동안 유지되는 것이다. Torque-on 뒤에는 동일한 3초 gate를 다시
통과해야 한다.

## 4. Keyboard 조작

| 기능 | 키 | 한 번 누를 때 변화 |
|---|---|---|
| EE +X / -X | `W` / `S` | `0.01 m` |
| EE +Y / -Y | `A` / `D` | `0.01 m` |
| EE +Z / -Z | `R` / `F` | `0.01 m` |
| EE +roll / -roll | `I` / `K` | `3 deg` |
| EE +pitch / -pitch | `J` / `L` | `3 deg` |
| 현재 측정 EE pose로 target 복귀 | `Space` | trajectory도 함께 정지 |
| 현재 target 출력 | `P` | 동작 변화 없음 |
| 단일 xyz+roll/pitch Lissajous ON/OFF | `G` | 현재 target을 중심으로 시작/정지 |
| Palletrone DOB ON/OFF | `B` | 실험 계획에 포함된 경우에만 사용 |
| Arm torque/DLS 요청 ON/OFF | `M` | Torque 요청과 DLS 자동 절차 시작/정지 |

`C`, `V`, `T`, `0`, `?`, `Q`, `Esc`는 사용하지 않으며 기능도 없다.
Keyboard 프로세스 종료는 실험 종료 절차를 마친 뒤 `Ctrl-C`로 한다.

## 5. 실제 비행 실험 순서

### A. 이륙과 첫 hover gate

1. [ ] `M`, `G`, `B`가 모두 OFF인지 확인한다.
2. [ ] 기존에 검증된 Palletrone 절차로 arm하고 이륙한다.
3. [ ] 목표 고도에서 조종 입력을 줄이고 안정 hover를 유지한다.
4. [ ] `/chr/controller_state`가 `WAITING_FOR_HOVER`에서 `HOLD`로 바뀔
       때까지 기다린다.
5. [ ] `HOLD` 전에는 `M`을 누르지 않는다.

### B. Arm torque-on과 DLS 진입

1. [ ] 안정 hover와 `HOLD`를 확인한 뒤 `M`을 한 번 누른다.
2. [ ] Driver가 현재 arm 자세를 zero로 캡처하고 torque를 켰다는 로그를
       확인한다.
3. [ ] `/cone/torque_enabled`가 `true`인지 확인한다.
4. [ ] Arm이 갑자기 큰 각도로 움직이지 않고 현재 자세를 유지하는지 본다.
5. [ ] 다시 `WAITING_FOR_HOVER` 상태에서 최소 3초 동안 안정 hover한다.
6. [ ] `ARM_READY`를 거쳐 `TRACKING`으로 바뀌는지 확인한다.
7. [ ] `TRACKING` 전에는 수동 EE 키와 `G`를 누르지 않는다.

`ARM_READY` 메시지는 keyboard가 받자마자 custom mode를 자동으로 켜므로
매우 짧게 보이고 바로 `TRACKING`으로 넘어갈 수 있다.

### C. 작은 수동 EE 확인

1. [ ] `P`로 현재 target xyz/roll/pitch를 출력한다.
2. [ ] 한 축만 선택해 한 번만 누른다. 첫 입력은 `W/S`, `A/D`, `R/F`
       중 계획된 안전 방향을 사용한다.
3. [ ] Palletrone과 arm이 예상 방향으로 작게 움직이는지 확인한다.
4. [ ] 위치 오차, 진동, arm limit 접근, DLS reject 로그가 없는지 확인한다.
5. [ ] 필요하면 `Space`를 눌러 현재 측정 EE pose를 새 target으로 만든다.
6. [ ] 위치 확인이 끝난 뒤에만 roll과 pitch를 축별로 한 번씩 확인한다.

한 축이라도 방향이 반대거나 큰 진동이 나타나면 `M`을 눌러 DLS를 끄고
실험을 중단한다.

### D. DOB와 사전 정의 trajectory

1. [ ] 실험 계획에 DOB가 포함됐을 때만 `B`를 한 번 눌러 ON 로그를
       확인한다.
2. [ ] 안정 hover와 `TRACKING`을 다시 확인한다.
3. [ ] `G`를 한 번 눌러 trajectory를 시작한다.
4. [ ] 처음 3초 동안 quintic ramp-in이 부드러운지 집중해서 확인한다.
5. [ ] 실험 중 기체 위치, arm 자세, EE 방향, 통신 오류, controller state를
       계속 감시한다.
6. [ ] 계획된 구간이 끝나면 `G`를 다시 눌러 trajectory를 정지한다.

Trajectory는 하나뿐이다. 현재 target을 중심으로 주파수 비 `1:2:3`의
닫힌 Lissajous 곡선을 사용하며 X/Y `±4 cm`, Z `±2 cm`, roll `±8 deg`,
pitch `±6 deg`를 동시에 명령한다. 기본 주기는 20초이고 처음 3초는
quintic ramp-in이다. 수동 EE 키나 `Space`를 누르면 trajectory는 자동으로
정지한다.

EE가 뻗는 local z축 주위의 yaw/twist는 keyboard에서 명령하지 않으며
DLS 오차와 수렴 판정에서도 제외한다. Palletrone base yaw는 position과
tool-z tilt를 만드는 데 사용할 수 있는 출력 자유도로 유지한다.

## 6. 정상 종료 순서

1. [ ] `G`를 눌러 trajectory를 OFF로 만들고 target hold를 확인한다.
2. [ ] DOB를 사용했다면 `B`를 눌러 OFF로 만든다.
3. [ ] 안정 hover 상태에서 `M`을 눌러 OFF로 만든다.
4. [ ] DLS 정지 로그를 확인하고 최소 3초 기다린다. PX4 custom offset은
       이 동안 0으로 완만하게 복귀한다.
5. [ ] 비행 중이므로 arm torque가 아직 ON인 것은 정상이다.
6. [ ] 기존 Palletrone 절차로 정상 착륙하고 disarm한다.
7. [ ] Disarm 뒤 `/cone/torque_enabled`가 `false`로 바뀌었는지 확인한다.
8. [ ] Torque-off 확인 뒤 keyboard 터미널을 `Ctrl-C`로 종료한다.
9. [ ] 마지막으로 전체 launch 터미널을 `Ctrl-C`로 종료한다.
10. [ ] rosbag이 종료되고 `metadata.yaml` 작성이 끝날 때까지 기다린다.

Torque-off 확인 전에 전체 launch를 종료하지 않는다.

## 7. 이상 상황 대응

| 상황 | 즉시 조치 | 이후 조치 |
|---|---|---|
| Trajectory 방향/형상이 이상함 | `G` OFF | `Space`로 현재 pose hold, 상태 확인 |
| EE 또는 base 움직임이 예상과 다름 | `M` OFF | DLS 정지와 offset 복귀 확인 후 착륙 |
| 큰 진동 또는 joint limit 접근 | `M` OFF | 안정 hover 후 즉시 착륙, 파라미터/방향 재점검 |
| `FAULT` 또는 feedback timeout | `G` OFF, `M` OFF | 정상 조종권 유지, 착륙 후 로그 분석 |
| Dynamixel 통신 오류 반복 | `M` OFF | 비행 중 driver를 끄지 말고 착륙 |
| Keyboard가 멈추거나 종료됨 | 조종과 착륙을 우선 | Target timeout으로 DLS는 정지하지만 torque request가 남을 수 있으므로 아래 복구 수행 |
| Rosbag 오류/디스크 부족 | 비행 조작을 우선 | 해당 run을 불완전 로그로 표시하고 정상 종료 |

Keyboard가 비정상 종료된 경우에는 target timeout으로 DLS가 멈추지만,
마지막 torque request가 controller에 남아 있을 수 있다. 정상 착륙과 disarm
뒤에만 다음 명령으로 arm torque를 끈다.

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
ros2 service call /cone/set_torque std_srvs/srv/SetBool "{data: false}"
```

비행 중 기계적 위험이 발생해 즉시 torque 차단이 필요한 상황은 소프트웨어
절차보다 현장 안전책임자의 비상 절차를 우선한다. Torque를 즉시 끄면 arm이
떨어지면서 비행 안정성을 크게 해칠 수 있다는 점을 함께 고려한다.

## 8. Bag 확인과 실험 후 정리

가장 최근 bag 확인:

```bash
cd ~/Desktop/chr_onboard_ws/src
LATEST_BAG=$(ls -dt data_logging/bags/rosbag2_* | head -n 1)
echo "${LATEST_BAG}"
ros2 bag info "${LATEST_BAG}"
```

다음을 확인한다.

- [ ] `metadata.yaml`이 존재한다.
- [ ] Duration이 실제 실험 시간과 비슷하다.
- [ ] `/data_logging_msg`와 `/chr/data_logging_msg`의 message count가 0이 아니다.
- [ ] PX4 odometry/status, joint state/command, TCP target/state가 기록됐다.
- [ ] Run 이름과 실제 수행 조건(DOB 여부, 수동/trajectory, 특이사항)을
      실험 노트에 적었다.

CSV 변환은 simulator와 동일하게 bag 경로 하나만 넘긴다. CSV는 해당 bag
폴더 안에 같은 이름으로 생성된다.

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/chr_onboard_ws/install/setup.bash
cd ~/Desktop/chr_onboard_ws/src/data_logging
python3 rosbag_to_csv.py bags/rosbag2_YYYY_MM_DD-HH_MM_SS/
```

MATLAB에서는 생성된 CSV 하나를 선택해 `Palletrone_Decryptor`와
`CHR_Decryptor`를 각각 실행한다.

## 9. Run별 최소 기록 항목

각 run이 끝날 때 다음 내용을 실험 노트에 남긴다.

- Run 이름과 시각
- 배터리 상태와 이륙/hover 고도
- DOB ON/OFF
- 수행한 수동 EE 축 또는 trajectory 종류
- `M ON`, `TRACKING`, `G ON/OFF`, `M OFF`, disarm 시각
- 최대 진동 또는 비정상 움직임
- DLS reject, feedback timeout, Dynamixel 오류 여부
- 정상 종료인지 중단 run인지
