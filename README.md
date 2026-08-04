# 3차원 부표 위치·타격 패키지

이 패키지는 YOLO와 RealSense Depth로 여러 부표의 위치를 계산하고,
외부 GUIDED 제어 패키지에 목표 좌표를 전달한 뒤 비전 기반 RC Override로
부표 타격 임무를 수행합니다.

이 패키지에는 자동 잠수와 지그재그 탐색 제어가 없습니다. 비전 좌표 계산은
항상 백엔드에서 동작하고, 제어 상태는 시작 플래그를 받은 뒤에만 시작합니다.

수영장 표시 좌표계는 `/guided/start_frame`을 기준으로 만듭니다. 이 pose의
위치가 `arena` 원점이고, pose의 yaw 방향이 `arena +X`이므로 기준 로봇의
yaw는 `arena`에서 0도로 보입니다.

부표 관측, 평균 트랙, 로봇 위치, 접근 목표와 `/waypoint` 명령은 변환해서
저장하지 않고 처음부터 끝까지 `odom` 좌표로 유지합니다. `arena` 좌표는
수영장 크기 입력, 영역 안/밖 검사와 RViz 표시에만 사용합니다.

## 수영장 영역과 기준 프레임

공통 파라미터는 다음과 같습니다.

- `arena_length_m`: 수영장 로컬 `+X` 방향 길이
- `arena_width_m`: 수영장 안쪽 Y 방향 너비
- `arena_depth_m`: 수면부터 바닥까지 깊이
- `arena_start_frame_topic`: 기준 pose 토픽, 기본 `/guided/start_frame`
- `arena_frame`: 표시용 TF 자식 프레임, 기본 `arena`
- `arena_start_corner`: `bottom_left` 또는 `bottom_right`
- `arena_safety_margin_m`: 이동 목표가 벽에서 떨어져야 하는 거리

수영장 좌표 범위는 항상 다음과 같습니다.

```text
arena x=[0, length]
arena z=[-depth, 0]
bottom_left : arena y=[-width, 0]
bottom_right: arena y=[0, width]
```

`guided_navigation.cpp`와 같은 start-frame 회전식을 사용합니다.

```text
odom_x = origin_x + cos(start_yaw) * arena_x - sin(start_yaw) * arena_y
odom_y = origin_y + sin(start_yaw) * arena_x + cos(start_yaw) * arena_y
odom_z = origin_z + arena_z
```

영역 검사 때만 `odom` 점을 위 식의 역변환으로 `arena`에 잠시 옮깁니다.
저장된 부표나 웨이포인트 값 자체는 바꾸지 않습니다.

```text
arena_start_corner:=bottom_left
arena_length_m:=5.49
arena_width_m:=2.74
arena_depth_m:=2.0
arena_start_frame_topic:=/guided/start_frame
```

`buoy_mission_manager_node`가 `/guided/start_frame`을 받아 `odom -> arena`
TF를 발행합니다. RViz의 Fixed Frame을 `arena`로 설정하면 화면에서만 기준
yaw가 0도로 정렬됩니다. 영역 밖에서 계산된 odom 부표 점은 좌표 지도에
저장하지 않으며 접근 좌표도 안전 여유를 포함한 영역 안으로 제한합니다.

## 음향 제어에서 비전 제어로 인계

기본값 `acoustic_handoff_enabled: true`에서는 다음 순서로 제어권을 받습니다.

```text
음향 제어기
  -> /homing/vision_search_active=true
  -> 이 패키지가 YOLO 부표와 유효한 Depth를 연속 4프레임 확인
  -> /vision/target_confirmed=true
  -> 음향 제어기가 RC 중립을 전송하고 음향 RC를 종료
  -> /homing/vision_control_granted=true
  -> 미션 상태 VISUAL_SERVO
  -> RC 타격 상태 ACQUIRE_BUOY
  -> 정렬, 삽입, 타격, 후진, 분리 확인
```

`/homing/vision_search_active=true`는 카메라 확인 요청일 뿐 제어권 승인이
아닙니다. 실제 이동 명령은 `/homing/vision_control_granted=true`를 받은
뒤에만 나갑니다. 승인 전에는 `/waypoint`, 모드 변경 요청,
`/mavros/rc/override`가 모두 잠깁니다. 음향 인계 경로에서는 승인 후 바로
RC 타격만 수행합니다.

확인에 사용하는 기본 조건은 다음과 같습니다.

- 부표 클래스가 `buoy_class_id`와 같아야 합니다.
- 바운딩 박스 안에서 유효한 Depth가 나와야 합니다.
- 서로 가까운 3차원 측정이 `handoff_confirm_hits`만큼 연속으로 들어와야 합니다.
- 측정 간격은 `handoff_detection_timeout_sec` 이하여야 합니다.
- 카메라 기준 위치 변화는 `handoff_detection_consistency_m` 이하여야 합니다.

타격 성공 시 `COMPLETE`, 실패 시 `FAILSAFE`로 끝납니다.

음향 인계를 사용하지 않고 단독 운용하려면
`acoustic_handoff_enabled:=false`로 실행하고 `/mission/start=true`를
보냅니다.

## 전체 동작

아래 순서는 음향 인계를 끈 단독 임무의 동작입니다.

1. 제어 시작 전에도 YOLO, Depth 거리 측정, `odom` 부표 좌표 저장을 계속합니다.
2. `/mission/start=true`를 기다립니다.
3. 플래그를 받으면 가장 가까운 확정 부표를 선택합니다.
4. `/waypoint`로 부표 앞 정지 좌표를 보냅니다.
5. 외부 GUIDED 제어기가 그 좌표까지 이동합니다.
6. `/odometry/filtered`로 도착과 정지를 확인합니다.
7. STABILIZE와 비전 제어로 전환합니다.
8. 스틱 정렬, 포크 삽입, 강한 전진 타격, 후진, 분리 확인을 수행합니다.
9. 성공하면 `COMPLETE`, 실패하면 `FAILSAFE`로 끝납니다.

`buoy_class_id`로 지정된 부표만 3차원 지도에 저장합니다.
`stick_class_id`는 부표 가까이에서 포크를 정렬할 때만 사용합니다.

## 백엔드 부표 좌표 계산

부표 좌표 계산은 `buoy_coordinate_mapper_node`가 전담합니다. 이 노드는
YOLO·Depth 처리 런치인 `laptop_yolo_detection.launch.py`에 포함되어 있으며
제어 노드와 독립적으로 계속 동작합니다.

따라서 음향 제어 중이거나 RC 제어권을 아직 받지 못한 상태에서도 YOLO,
Depth 거리 계산, 오돔 기준 좌표 변환, 이상치 제거, 평균 좌표 저장이 계속
동작합니다. 제어 상태는 이 계산을 켜거나 끄지 않습니다.

```text
YOLO 부표 바운딩 박스
  -> 바운딩 박스 내부 Depth 계산
  -> 카메라 기준 3차원 위치 계산
  -> /odometry/filtered와 TF를 이용해 odom 좌표로 변환
  -> 기존 부표 측정점과 연결
  -> 튀는 측정값 제거
  -> 부표 평균 위치 갱신
```

바운딩 박스가 잡혔다고 무조건 지도에 저장되는 것은 아닙니다. 다음 조건을
모두 만족해야 합니다.

- 인식된 객체가 `buoy_class_id`여야 합니다.
- 바운딩 박스 내부에서 유효한 Depth가 측정되어야 합니다.
- 영상 시각과 가까운 `/odometry/filtered` 데이터가 있어야 합니다.
- 카메라 프레임과 `base_link` 사이 TF가 있어야 합니다.
- 계산된 `odom` 좌표가 설정한 3차원 수영장 영역 안에 있어야 합니다.

계산된 좌표와 평균 부표 목록은 다음 토픽으로 나옵니다.

- `/mission/buoy_observation`: 매 프레임 계산된 원시 `odom` 좌표
- `/mission/buoy_tracks`: 이상치를 제거하고 평균 낸 부표 목록

`buoy_mission_manager_node`는 더 이상 카메라 좌표 변환이나 평균 계산을 하지
않습니다. 완성된 `/mission/buoy_tracks`만 구독하여 제어 판단에 사용합니다.

## 노드 역할 분리

```text
비전 데이터 처리
  yolo_buoy_detector
  depth_range_node
  buoy_coordinate_mapper_node
  yolo_range_overlay_node

제어
  buoy_mission_manager_node
  buoy_visual_strike_node
```

`buoy_coordinate_mapper_node`는 `/vision/target_confirmed` 판단도 담당합니다.
제어 노드는 이 결과와 음향 제어기의 최종 승인만 받아 RC 타격을 시작합니다.

비전 데이터 처리는 다음 런치로 실행합니다.

```bash
ros2 launch auv_vision_nav_control laptop_yolo_detection.launch.py \
  model_path:=/path/to/model.pt \
  arena_start_corner:=bottom_left \
  arena_length_m:=5.49 \
  arena_width_m:=2.74 \
  arena_start_frame_topic:=/guided/start_frame \
  show_preview:=false \
  show_output_window:=false
```

이 런치를 실행하면 YOLO, Depth 거리 계산, 오돔 좌표 저장 노드가 함께
실행됩니다. 제어 런치는 별도로 실행합니다.

### 카메라 장착 TF

로스백에는 `base_link -> fcu_link`와 RealSense 내부 TF는 있지만
`fcu_link -> camera_link`가 없습니다. 비전 처리 런치는 기본적으로 카메라가
픽소호크와 같은 위치이고 같은 방향이라고 가정하여 다음 TF를 발행합니다.

```text
fcu_link -> camera_link
translation = (0, 0, 0)
rotation    = (roll=0, pitch=0, yaw=0)
```

나중에 카메라 장착 위치를 측정하면 다음 인자로 수정할 수 있습니다.

```bash
ros2 launch auv_vision_nav_control \
  laptop_yolo_detection.launch.py \
  model_path:=/path/to/model.pt \
  camera_mount_parent_frame:=fcu_link \
  camera_mount_x_m:=0.0 \
  camera_mount_y_m:=0.0 \
  camera_mount_z_m:=0.0 \
  camera_mount_roll_rad:=0.0 \
  camera_mount_pitch_rad:=0.0 \
  camera_mount_yaw_rad:=0.0
```

다른 패키지가 이미 같은 TF를 발행한다면 중복 발행을 막아야 합니다.

```bash
publish_camera_mount_tf:=false
```

실행 후 다음 명령이 끊기지 않고 값을 출력해야 부표의 `odom` 좌표를 계산할
수 있습니다.

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

```bash
ros2 launch auv_vision_nav_control buoy_mission.launch.py \
  arena_start_corner:=bottom_left \
  arena_length_m:=5.49 \
  arena_width_m:=2.74
```

## 기본 운용 설정

현재 기본값은 실제 이동과 실제 타격이 켜져 있습니다.

- 기본 음향 인계 경로에서는 승인 직후 실제 RC Override 타격을 시작합니다.
- 이 경로에서는 GUIDED 목표 좌표를 보내지 않습니다.
- 외부 `GUIDED` 제어기로 보내는 좌표는 음향 인계를 끈 플래그 시작 임무에서 사용합니다.

자동 시작 기능은 없으며 반드시 시작 또는 인계 플래그가 필요합니다.
기본 음향 인계 모드에서는 카메라로 부표를 확인하고 최종 승인을 받은 뒤
바로 RC 타격을 시작합니다.

로봇 위치가 설정한 활동 범위를 벗어나거나 `/odometry/filtered`가 끊기면
비전 제어를 해제하고 `FAILSAFE` 상태로 들어갑니다.

## GUIDED 이동 좌표

부표 지도와 선택된 부표 앞 목표점을 `odom` 기준의 절대좌표로 사용합니다.
미션 노드는 픽소호크로 직접 보내지 않고 외부 GUIDED 제어 패키지에만
목표를 전달합니다.

```text
odom에 고정된 최종 목표
  -> 목표의 odom x, y, z를 그대로 사용
  -> /waypoint로 전송
  -> 외부 GUIDED 제어 패키지가 수신하여 픽소호크 제어
```

따라서 로봇이 이동하거나 회전해도 보내는 목표 좌표는 변하지 않습니다.
`base_link` 기준 상대좌표로 바꾸는 계산은 하지 않습니다.
`yaw`와 `yaw_rate`는 모두 무시하도록 설정하므로 GUIDED 위치 명령은 기체의
방향을 제어하지 않습니다.

ROS의 `odom` 좌표 방향은 다음과 같습니다.

```text
+x, +y: 시작할 때 정해진 수평 방향
+z: 로봇 위쪽
```

메시지는 `mavros_msgs/PositionTarget`이며 좌표 형식은
`FRAME_LOCAL_NED`입니다. 이 노드는 ROS의 `odom` 좌표를 넣어
`/waypoint`로만 발행합니다. MAVROS 토픽 발행과 GUIDED 모드
전환은 외부 제어 패키지의 책임입니다.

외부 GUIDED 제어 담당 코드가 맞춰야 하는 `/waypoint` 규격은 다음과
같습니다.

| 항목 | 값 |
|---|---|
| 메시지 | `mavros_msgs/msg/PositionTarget` |
| `header.frame_id` | `/odometry/filtered`의 프레임, 일반적으로 `odom` |
| `position.x/y/z` | ROS `odom` 기준 절대좌표 |
| 속도·가속도 | 사용하지 않음 |
| `yaw`, `yaw_rate` | 사용하지 않음 |
| 발행 주기 | 접근 목표가 활성화된 동안 약 10 Hz |

`coordinate_frame` 필드가 `FRAME_LOCAL_NED`이더라도 `position`에는
`/odometry/filtered`에서 사용하는 ROS `odom` 좌표값이 그대로 들어갑니다.
외부 GUIDED 노드는 같은 목표가 반복 수신된다는 이유로 이동 상태를 매번
초기화하면 안 됩니다.

## 주요 입력 토픽

- `/odometry/filtered` (`nav_msgs/Odometry`)
- `/vision/buoy_detection_2d`
- `/vision/buoy_detection_3d`
- `/homing/vision_search_active` (`std_msgs/Bool`)
- `/homing/vision_control_granted` (`std_msgs/Bool`)
- `/mission/start` (`std_msgs/Bool`): 단독 임무 시작·해제 플래그
- `/mission/target_complete` (`std_msgs/Bool`)

## 주요 출력 토픽

- `/mission/state`
- `/vision/target_confirmed`
- `/mission/buoy_tracks`
- `/mission/buoy_observation`
- `/mission/active_waypoint`
- `/mission/control_enable`
- `/vision/buoy_bbox`: 선택된 부표와 그 부표에 연결된 스틱
- `/mission/visual_state`
- `/mission/target_complete`
- `/mission/target_failed`
- `/mission/rc_command`: dry-run에서도 확인할 수 있는 RC 명령
- `/waypoint`: `dry_run=false`이고 제어권이 있을 때 발행하는 절대 `odom` 목표
- `odom -> arena` TF: `/guided/start_frame`을 받은 뒤 계속 발행

시각화 노드에서 추가로 발행하는 토픽:

- `/mission/visualization_markers`
- `/mission/actual_path`

## 패키지 빌드

현재 워크스페이스 안에는 같은 ROS 패키지 이름을 사용하는 다른 패키지가
있습니다. 기본 `build/`와 `install/`을 사용하면 서로 충돌할 수 있으므로
이 패키지 전용 폴더로 빌드합니다.

```bash
cd /home/pc/Downloads/auv_buoy_ws

colcon --log-base log_yolo_range_ver1 build \
  --base-paths src/yolㅐ_range_ver1/auv_vision_nav_control \
  --build-base build_yolo_range_ver1 \
  --install-base install_yolo_range_ver1

source install_yolo_range_ver1/setup.bash
```

## 실행 전 카메라 확인

RealSense ROS 패키지는 최소한 다음 기능을 켜야 합니다.

```text
enable_color:=true
enable_depth:=true
align_depth.enable:=true
enable_sync:=true
```

이 패키지를 실행하기 전에 아래 세 토픽이 실제로 들어오는지 확인합니다.

```bash
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo --once /camera/camera/aligned_depth_to_color/camera_info
```

`/odometry/filtered`도 끊기지 않고 들어와야 부표를 `odom` 좌표로 저장할 수
있습니다.

```bash
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 전체 실행 순서

각 명령은 별도 터미널에서 실행합니다. 모든 터미널에서 같은 ROS 환경과 이
패키지의 설치 환경을 먼저 불러와야 합니다.

첫 번째 터미널에서는 비전, 거리 측정, 부표 좌표 저장을 실행합니다.

```bash
source /home/pc/Downloads/auv_buoy_ws/install_yolo_range_ver1/setup.bash

ros2 launch auv_vision_nav_control \
  laptop_yolo_detection.launch.py \
  model_path:=/home/pc/Downloads/best.pt \
  device:=auto \
  show_preview:=false \
  show_output_window:=false
```

두 번째 터미널에서는 미션과 타격 제어를 실행합니다.

```bash
source /home/pc/Downloads/auv_buoy_ws/install_yolo_range_ver1/setup.bash

ros2 launch auv_vision_nav_control buoy_mission.launch.py
```

세 번째 터미널은 선택 사항입니다. RViz가 설치된 모니터링 노트북에서만
실행합니다.

```bash
source /노트북의/워크스페이스/install_yolo_range_ver1/setup.bash

ros2 launch auv_vision_nav_control \
  buoy_mission_visualization.launch.py
```

수영장 크기와 오프셋을 바꿀 때는 비전 런치, 미션 런치, 시각화 런치에
동일한 `arena_*` 값을 전달해야 합니다.

## 최종 실행 점검 결과

2026년 7월 23일 기준으로 패키지 전용 설치 폴더에 다시 빌드하고 세 런치를
각각 실행해 다음 항목을 확인했습니다.

| 런치 | 확인 결과 |
|---|---|
| `laptop_yolo_detection.launch.py` | 5개 프로세스 시작, `best.pt`의 `buoy/stick` 클래스 로딩, `fcu_link -> camera_link` TF 발행 확인 |
| `buoy_mission.launch.py` | 미션 관리자와 비전 타격 노드 시작, `dry_run`에서 외부 좌표와 RC 출력이 잠기는 것 확인 |
| `buoy_mission_visualization.launch.py` | 시각화 노드 시작과 `odom` 기준 3차원 수영장 영역 생성 확인 |

Python 런치 파일 문법 검사와 `colcon build`도 통과했습니다. 자동 단위
테스트는 아직 등록되어 있지 않으므로 실제 카메라 영상, Depth, 오돔,
음향 인계 플래그를 함께 넣는 수조 통합 시험은 별도로 필요합니다.

## 움직이지 않는 시험

프로펠러를 제거하거나 기체를 고정한 상태에서 다음처럼 실행하면 목표 좌표와
RC Override를 실제로 보내지 않습니다.

```bash
ros2 launch auv_vision_nav_control buoy_mission.launch.py \
  dry_run:=true \
  visual_dry_run:=true \
  request_vision_mode:=false
```

음향 인계 없이 패키지만 시험할 때는 인계를 끄고 임무 시작 신호를 보냅니다.

```bash
ros2 launch auv_vision_nav_control buoy_mission.launch.py \
  acoustic_handoff_enabled:=false \
  dry_run:=true \
  visual_dry_run:=true

ros2 topic pub --once --qos-durability transient_local \
  /mission/start std_msgs/msg/Bool "{data: true}"
```

## Jetson Orin NX와 모니터링 노트북 구성

권장 구성은 다음과 같습니다.

```text
Jetson Orin NX                    모니터링 노트북
----------------------------     -----------------------------
RealSense                        buoy_mission_visualization_node
YOLO와 거리 계산            ->   RViz2
부표 좌표 지도
임무 상태 관리
픽소호크 제어
```

Jetson에서는 RViz2와 `buoy_mission_visualization_node`를 실행하지 않는 것을
권장합니다. 일반 임무 실행 파일인 `buoy_mission.launch.py`에는 RViz와
시각화 노드가 포함되어 있지 않으므로 Jetson은 임무와 제어만 수행합니다.

`buoy_mission_visualization.launch.py use_rviz:=false`를 실행하면 RViz 창만
꺼지고 시각화 노드는 계속 실행됩니다. 따라서 이 방법도 Jetson용 실행
방법으로 권장하지 않습니다.

### 두 장비의 ROS 통신 설정

Jetson과 노트북을 같은 네트워크에 연결하고 양쪽에서 같은 값을 사용합니다.

```bash
export ROS_DOMAIN_ID=26
export ROS_LOCALHOST_ONLY=0
```

### Jetson에서 실행

Jetson에서는 RViz를 제외하고 비전 런치와 임무 런치를 각각 실행합니다.

```bash
source /home/pc/Downloads/auv_buoy_ws/install_yolo_range_ver1/setup.bash

ros2 launch auv_vision_nav_control \
  laptop_yolo_detection.launch.py \
  model_path:=/home/pc/Downloads/best.pt \
  device:=auto \
  show_preview:=false \
  show_output_window:=false
```

다른 터미널에서 실행합니다.

```bash
source /home/pc/Downloads/auv_buoy_ws/install_yolo_range_ver1/setup.bash
ros2 launch auv_vision_nav_control buoy_mission.launch.py
```

위 명령은 실제 이동과 실제 타격이 켜진 기본 설정을 사용합니다. 노드를
실행하는 것만으로 바로 움직이지 않습니다. 음향 인계 모드에서는 음향
제어기의 요청과 최종 승인이 필요하고, 단독 모드에서는 `/mission/start`
플래그가 필요합니다.

```bash
ros2 topic echo /homing/vision_search_active
ros2 topic echo /vision/target_confirmed
ros2 topic echo /homing/vision_control_granted
```

### 노트북에서 RViz 실행

필요하면 모니터링 노트북에 RViz2를 설치합니다.

```bash
sudo apt install ros-humble-rviz2
```

Jetson의 `install` 폴더를 x86 노트북으로 복사해서 사용하면 안 됩니다.
Jetson에서 만든 실행 파일은 ARM용이므로 같은 소스 패키지를 노트북에서
별도로 빌드해야 합니다.

노트북에서 해당 빌드 환경을 불러온 뒤 실행합니다.

```bash
source /노트북의/워크스페이스/install_yolo_range_ver1/setup.bash

ros2 launch auv_vision_nav_control \
  buoy_mission_visualization.launch.py
```

GUI를 실행하기 전에 노트북에서 Jetson 토픽이 보이는지 확인합니다.

```bash
ros2 topic echo --once /odometry/filtered
ros2 topic echo --once /mission/buoy_tracks
```

RViz에는 다음 항목이 표시됩니다.

- 설정된 3차원 탐색 공간
- 실제 로봇 이동 경로
- 로봇의 현재 위치와 방향
- 현재 목표 좌표
- 부표를 측정할 때마다 생성된 점
- 여러 측정값으로 계산한 부표 평균 위치와 번호
- 현재 임무 상태

부표 상태 색상은 다음과 같습니다.

- 노란색: 아직 확정되지 않은 부표 후보
- 빨간색: 위치가 확정된 부표
- 파란색: 현재 접근 중인 부표
- 초록색: 타격 완료된 부표
- 자홍색: 놓친 부표
- 회색: 오래되었거나 제외된 부표

RViz의 기준 좌표는 `arena`입니다. 부표, 로봇 경로와 웨이포인트 메시지는
여전히 `odom`이지만 RViz가 `odom -> arena` TF로 화면에서만 변환합니다.

임무 탐색 범위를 변경했다면 시각화에도 같은 값을 전달해야 합니다.

```bash
ros2 launch auv_vision_nav_control \
  buoy_mission_visualization.launch.py \
  arena_start_corner:=bottom_left \
  arena_length_m:=5.49 \
  arena_width_m:=2.74 \
  arena_depth_m:=11.0
```

## YOLO 설정 주의사항

YOLO는 부표와 스틱을 모두 발행해야 합니다. YOLO 실행 파일에서 다음 값을
유지합니다.

```text
target_class_id:=-1
target_class_name:=""
publish_all_targets:=true
```

## RC Override 충돌 주의

기존 `auv_buoy_vision_control/mission_state_machine_node` 또는
`bbox_controller_node`를 `buoy_visual_strike_node`와 동시에 실행하면 안
됩니다. 이 노드들은 모두 같은 `/mavros/rc/override` 토픽에 명령을 보내므로
동시에 실행하면 서로 다른 제어 명령이 충돌합니다.

## 기본 타격 순서

```text
ALIGN_STICK
  -> INSERT_FORK    1560 PWM, 0.8초
  -> STRIKE         1620 PWM, 0.3초
  -> RETRACT        1420 PWM, 0.5초
  -> VERIFY_RELEASE
```

이 PWM 값과 시간은 실제 기체에서 검증된 최종값이 아니라 초기 설정값입니다.
프로펠러를 제거하거나 기체를 단단히 고정한 상태에서 값을 조정해야 합니다.
시험할 때는 `/mission/rc_command`, `/mission/visual_state`, 카메라 영상을
함께 기록하는 것을 권장합니다.
