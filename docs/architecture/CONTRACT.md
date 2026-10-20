# 시스템 계약서 (Interface Contract)

이 문서는 워크스페이스 전체가 공유하는 **단일 기준**이다. 패키지 이름, 토픽·액션 이름과 타입, 좌표계, 관절 이름, 로봇·창고 치수, 설정 파일 위치를 여기서 정한다. 코드를 바꾸다가 계약과 달라져야 한다면 **이 문서를 먼저 고친다**.

아키텍처는 4개 층으로 나뉜다 (`docs/architecture.md`의 그림 참고).

| 층 | 역할 | 패키지 |
|---|---|---|
| 지령 층 | 무엇을 · 어떤 순서로 | `warehouse_tasks` (LLM, Task Manager, 콘솔), `warehouse_dashboard` |
| 실행 층 | 어디로 · 어떻게 움직일지 | Nav2 (`dualarm_navigation`), MoveIt 2 (`dualarm_moveit_config`), `warehouse_skills` (PickPlace 스킬 서버) |
| 브리지 층 | ROS 2 인터페이스 | `dualarm_mujoco` (MuJoCo ↔ ROS 2 브리지 + 카메라 핸들러) |
| 물리 층 | MuJoCo | `dualarm_mujoco` (MJCF 빌더, 창고 월드), `dualarm_description` (URDF) |

공통: `warehouse_interfaces` (msg/srv/action), `dualarm_bringup` (통합 launch).

---

## 0. 공통 규칙

- 대상 환경: **ROS 2 Jazzy, Python 3.12, MuJoCo ≥ 3.3**. Ubuntu 24.04(apt 또는 RoboStack)와 macOS Apple Silicon(RoboStack) 모두 지원. **CUDA·GPU 불필요** — 모든 계산은 CPU.
- `warehouse_interfaces`만 `ament_cmake`, 나머지는 모두 `ament_python`.
- ROS 노드 코드와 순수 로직을 분리한다. 순수 로직(기하 계산, 제어 법칙, 스키마, 계획 검증, 보간 등)은 rclpy 없이 import·테스트 가능해야 한다.
- 모든 노드는 `use_sim_time:=true`로 실행된다 (`/clock`은 브리지가 발행). 시간 판단은 시뮬레이션 시간 기준. wall-clock 고정 sleep으로 완료를 추정하지 않는다 — 상태·액션 결과를 확인한다.
- 주석·docstring·로그·문서는 **한국어**로 간결하게. 식별자는 영어.
- 실행 스크립트는 기본 `ROS_DOMAIN_ID=47`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- 각도는 rad, 길이는 m, 질량은 kg.

---

## 1. 좌표계 (TF)

```
map ──(AMCL 또는 slam_toolbox)──▶ odom ──(브리지: 바퀴 엔코더 오도메트리)──▶ base_footprint
base_footprint ──(robot_state_publisher, URDF)──▶ base_link ─┬─ left_wheel_link / right_wheel_link
                                                             ├─ laser_link
                                                             ├─ base_camera_link ─ base_camera_optical_frame
                                                             └─ torso_link ─┬─ head_camera_link ─ head_camera_optical_frame
                                                                            ├─ left_arm_base_link ─ left_link1 … left_link7 ─ left_hand_link ─┬─ left_tcp
                                                                            │                                                              ├─ left_finger_link1 / left_finger_link2
                                                                            │                                                              └─ left_wrist_camera_link ─ left_wrist_camera_optical_frame
                                                                            └─ right_arm_base_link ─ (left와 대칭, 접두사 right_)
```

- **MuJoCo world 좌표계 = `map` 좌표계**. 창고 바닥 중심이 원점, z 위쪽. 로봇 초기 위치에서 `odom`은 `map`과 일치(초기 AMCL pose = `locations.yaml`의 `home`).
- 광학 프레임(`*_optical_frame`)은 ROS 규약: z 전방, x 오른쪽, y 아래.
- `base_footprint`는 바닥(z=0), 바퀴 축 바로 아래, 로봇 회전 중심.

## 2. 로봇 모델 `mobile_dualarm`

차동 구동 이동 베이스 + 7축 양팔 + 평행 그리퍼. 메시 없이 기본 도형(box/cylinder/capsule/sphere)만으로 만든 **원본 모델**이다.

### 2.1 베이스

| 항목 | 값 |
|---|---|
| 바퀴 반지름 | 0.0825 |
| 바퀴 간격 (좌우 바퀴 중심 사이) | 0.43 |
| `base_footprint → base_link` | z = 0.0825 (바퀴 축 높이) |
| 차체 외형 (footprint) | x: -0.32 ~ +0.30, y: -0.24 ~ +0.24 (바퀴 포함) |
| 캐스터 | 앞·뒤 각 1개, 구, 저마찰 |
| 최대 속도 (브리지 명령 제한) | 0.5 m/s, 1.2 rad/s |
| cmd_vel watchdog | 0.5 s 동안 명령이 없으면 감속 정지 |
| `laser_link` | 차체 위 앞쪽, 바닥에서 약 0.30, 360° 시야 (로봇 자신의 형상은 레이 캐스트에서 제외) |
| 베이스 질량 | 약 30 (팔 동작으로 넘어지지 않을 것) |

### 2.2 몸통·팔·그리퍼

- `torso_link`: 베이스 위 고정 기둥. 어깨 높이(양팔 `*_arm_base_link`) 바닥에서 약 1.0.
- 팔: 각 7 회전 관절 (S-R-S 구조, 영점 자세 = 팔을 아래로 늘어뜨림).
  - `{side}_joint1` 어깨 pitch, `{side}_joint2` 어깨 roll, `{side}_joint3` 상완 yaw, `{side}_joint4` 팔꿈치 pitch, `{side}_joint5` 전완 yaw, `{side}_joint6` 손목 pitch, `{side}_joint7` 손목 roll. (`side` ∈ {`left`, `right`}, 왼팔은 +y 쪽)
  - 관절 한계·속도 한계는 URDF에 명시하고 SRDF·joint_limits.yaml과 일치시킨다.
- 그리퍼: `{side}_finger_joint1`(능동, prismatic), `{side}_finger_joint2`(mimic, 같은 변위로 반대 방향). 한쪽 finger 변위 0(닫힘) ~ 0.045(열림). 최대 벌림 0.09 ≥ 상자 폭.
- TCP: `{side}_tcp` = 두 finger 패드 사이 중심. TCP의 +z가 접근 방향(손가락이 뻗은 방향), finger는 TCP의 ±y로 벌어진다.
- **도달성 요구사항** (테스트로 검증): 도킹 후(로봇 앞 범퍼와 작업대 앞 모서리 간격 0.05~0.10), 작업대 상판 위 상자 중심이 앞 모서리에서 0.08~0.20 안쪽, 좌우 ±0.25 범위에 있을 때 **위에서 아래로 내려 잡는 자세**(TCP z축이 아래)의 IK 해가 각 팔에 대해 존재해야 한다. pre-grasp(상자 위 0.12)와 lift(상자 위 0.15)도 포함.

### 2.3 관절 상태 `/joint_states` (20개, 이 순서)

```
left_wheel_joint, right_wheel_joint,
left_joint1 … left_joint7, left_finger_joint1, left_finger_joint2,
right_joint1 … right_joint7, right_finger_joint1, right_finger_joint2
```

### 2.4 이름 있는 팔 자세 (SRDF group_state)

| 이름 | 용도 |
|---|---|
| `home` | 모든 관절 0 (늘어뜨림). 주행 자세 아님 |
| `ready` | 작업대 위 조작 준비 자세 (손이 앞쪽 위) |
| `carry` | 주행용 접은 자세. 손과 들고 있는 상자가 footprint 안에 있어야 함. **주행 전 기본 자세** |

### 2.5 카메라 4대

| 이름 | 장착 | 용도 |
|---|---|---|
| `base_camera` | 차체 앞면, 높이 약 0.25, 수평 전방 | 작업대 앞면 ArUco 마커 도킹 |
| `head_camera` | 몸통 위, 전방 아래로 약 40° | 작업대 위 상자 검출 |
| `left_wrist_camera` / `right_wrist_camera` | 손목, TCP 방향 | 근접 확인 |

기본 해상도 320×240, 수평 FOV 약 70°. MuJoCo 카메라와 URDF 광학 프레임의 외부 파라미터가 일치해야 한다 (테스트로 검증).

## 3. 창고 월드 (단일 원본: `dualarm_mujoco/worlds/warehouse.yaml`)

MuJoCo 모델, 정적 지도(PGM), MoveIt planning scene, `locations.yaml`의 근거가 모두 이 YAML 하나다.

| 항목 | 값 |
|---|---|
| 내부 바닥 | 15 × 11 (x: -7.5 ~ 7.5, y: -5.5 ~ 5.5), 둘레 벽 높이 2.5 |
| 랙 | 6개, 고정 장애물 (약 2.4 × 0.6 × 2.0), 통로 폭 ≥ 1.4 |
| 작업대 | `pickup_station`, `drop_station` 2개. 상판 높이 0.75, 크기 약 1.2 × 0.6. 앞면에 ArUco 마커 |
| 상자 | `parcel_red`, `parcel_blue`, `parcel_yellow` — 0.05 × 0.05 × 0.06, 0.08 kg, free body. 처음엔 모두 `pickup_station` 위 (앞 모서리에서 0.12 안쪽, 좌우 간격 0.2) |
| ArUco | `DICT_4X4_50`, 검은 사각형 한 변 0.10. `pickup_station`=id 10, `drop_station`=id 20. 작업대 앞면 중앙, 중심 높이 약 0.35 (base_camera 시야 안) |
| 배우(actor) | 사람 1, 지게차 1. mocap(키네마틱) 몸체로 웨이포인트를 왕복. 라이다·카메라에 보인다. 작업대 도킹 구역은 지나가지 않는다. 런치 인자 `actors:=true|false` |
| 로봇 초기 위치 | `home` (map 좌표, YAML에 명시) |

## 4. 장소 `dualarm_navigation/config/locations.yaml`

```yaml
locations:
  home:            {x: ..., y: ..., yaw: ..., description: "출발·대기 위치"}
  pickup_station:  {x: ..., y: ..., yaw: ..., description: "픽업 작업대 앞 사전 도킹 위치", station: true, marker_id: 10}
  drop_station:    {x: ..., y: ..., yaw: ..., description: "적재 작업대 앞 사전 도킹 위치", station: true, marker_id: 20}
  # 순찰·이동용 추가 지점 (aisle_*, inspection_point 등)
```

- 작업대 항목의 pose는 **작업대 앞면에서 약 0.8 떨어진 사전 도킹 위치**이며 로봇이 작업대를 정면으로 바라보는 yaw.
- 한국어 별칭(예: "픽업 작업대", "적재 작업대", "출발 위치")은 `warehouse_tasks`의 설정에서 정의한다.

## 5. ROS 인터페이스

### 5.1 브리지 (`dualarm_mujoco`, 노드 `mujoco_bridge`)

| 종류 | 이름 | 타입 | 비고 |
|---|---|---|---|
| pub | `/clock` | rosgraph_msgs/Clock | 100 Hz (시뮬레이션 시간) |
| pub | `/joint_states` | sensor_msgs/JointState | 50 Hz, 2.3의 20개 관절 |
| pub | `/odom` | nav_msgs/Odometry | 50 Hz, `odom` → `base_footprint`, 바퀴 엔코더 적분 |
| tf | `odom` → `base_footprint` | | 50 Hz |
| pub | `/scan` | sensor_msgs/LaserScan | 10 Hz, `laser_link`, 360 beams, 0.12 ~ 12.0 |
| pub | `/ground_truth` | nav_msgs/Odometry | 실제 pose (`map` 기준). 검증·평가용. Nav2/SLAM 입력 금지 |
| pub | `/warehouse/object_poses` | geometry_msgs/PoseArray | `map` 기준 상자 실제 pose, YAML의 parcels 순서 |
| sub | `/cmd_vel` | geometry_msgs/Twist | 차동 구동 (watchdog 0.5 s) |
| action | `/left_arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | 왼팔 7관절 |
| action | `/right_arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | 오른팔 7관절 |
| action | `/left_gripper_controller/gripper_cmd` | control_msgs/GripperCommand | position = 한쪽 finger 변위 |
| action | `/right_gripper_controller/gripper_cmd` | control_msgs/GripperCommand | |
| srv | `/sim/reset` | std_srvs/Trigger | 로봇·상자·배우 초기화 |
| srv | `/sim/set_actors_enabled` | std_srvs/SetBool | 배우 움직임 켜기/끄기 |
| pub | `/vision/detections` | warehouse_interfaces/DetectionArray | 카메라 핸들러, `base_footprint` 기준 3D 위치 |
| pub | `/vision/markers` | warehouse_interfaces/ArucoMarkerArray | 카메라 핸들러, `base_footprint` 기준 |

- **영상 토픽(Image/CompressedImage/CameraInfo)은 발행하지 않는다.** 카메라 핸들러는 같은 프로세스 안에서 MuJoCo 렌더 프레임을 NumPy로 받아 검출 결과만 발행한다.
- 카메라 핸들러 백엔드: `render`(오프스크린 렌더 → HSV 색 분할 + 깊이 → 3D, `cv2.aruco` + solvePnP) / `truth`(실제 pose를 카메라에 투영 + 시야·가림 검사). 렌더 초기화에 실패하면 `truth`로 자동 전환하고 경고한다.
- 안전 인터록: 팔/그리퍼 궤적 실행 중에는 `/cmd_vel`을 무시, 베이스가 움직이는 중(|v| > 0.02)에는 새 팔 궤적을 거절. (파라미터로 끌 수 있음)
- 궤적 액션: 관절 이름·개수·유한값·한계·시간 단조 증가 검사, 동시 중복 목표 거절, 취소 시 현재 자세 유지, goal tolerance 기본 0.02 rad, 실제 관절이 목표에 도달했을 때 SUCCEEDED.
- 파지: 마찰 기반 물리 파지. `grasp_assist:=true`(기본)이면 두 finger가 같은 상자에 닿은 채 닫힘 명령이 유지될 때 손–상자 weld 구속을 켜고, 열림 명령에 끈다(시뮬레이션 보조임을 문서에 명시).

### 5.2 실행 층

| 종류 | 이름 | 타입 | 제공자 |
|---|---|---|---|
| action | `/navigate_to_pose` | nav2_msgs/NavigateToPose | Nav2 |
| action | `/follow_waypoints` | nav2_msgs/FollowWaypoints | Nav2 (순찰) |
| action | `/move_action` | moveit_msgs/MoveGroup | MoveIt move_group |
| action | `/execute_trajectory` | moveit_msgs/ExecuteTrajectory | MoveIt move_group |
| srv | `/compute_ik` | moveit_msgs/GetPositionIK | MoveIt |
| srv | `/compute_cartesian_path` | moveit_msgs/GetCartesianPath | MoveIt |
| action | `/pick_place` | warehouse_interfaces/PickPlace | `warehouse_skills` (`pick_place_server`) |

- MoveIt planning group: `left_arm`, `right_arm`, `both_arms`, `left_gripper`, `right_gripper`. end effector: `left_hand`, `right_hand`. planning frame: `base_footprint`.
- Nav2 footprint: `carry` 자세 기준 다각형. 컨트롤러는 4코어 CPU에서 20 Hz로 돌 수 있어야 한다.

### 5.3 지령 층

| 종류 | 이름 | 타입 | 제공자 |
|---|---|---|---|
| sub/pub | `/warehouse/utterance` | warehouse_interfaces/Utterance | 콘솔·대시보드 → Task Manager |
| pub | `/warehouse/narration` | warehouse_interfaces/Utterance | Task Manager → 사용자 |
| pub | `/warehouse/task_status` | warehouse_interfaces/TaskStatus | Task Manager (변화 시 + 1 Hz) |
| action | `/execute_plan` | warehouse_interfaces/ExecutePlan | Task Manager |
| srv | `/warehouse/get_locations` | warehouse_interfaces/GetLocations | Task Manager |

### 5.4 PickPlace 스킬 동작 (`/pick_place`)

- `phase=pick`: `approach`(Nav2로 station 사전 도킹 위치) → `dock`(`/vision/markers`의 해당 marker로 정밀 접근: 마커 법선과 정렬, 범퍼–작업대 간격 목표값까지 `/cmd_vel` 폐루프) → `locate`(`/vision/detections`에서 object 색의 3D 위치) → `pre_grasp` → `grasp`(그리퍼 닫기) → `lift` → `carry`(carry 자세). 상자를 든 채 성공으로 끝난다.
- `phase=place`: `approach` → `dock` → `pre_place` → `release` → `retreat`(후진 0.3) → `carry`.
- 팔 선택: `arm`이 비면 상자의 y가 양수면 left, 음수면 right.
- 실패 시 그리퍼를 열지 않고(들고 있으면 유지) 팔을 carry로 복귀 시도 후 실패 결과와 `phase_reached`를 반환.

### 5.5 LLM 스키마 (OpenAI Structured Outputs, pydantic)

- `RobotCommand` (모듈 ⑪): `intent` ∈ {`move_object`, `go_to`, `pick`, `place`, `arm_pose`, `gripper`, `stop`, `status`, `unknown`}, `object` (red|blue|yellow|""), `source`, `destination` (locations 키 또는 ""), `arm` (left|right|auto), `pose_name`, `reply`.
- `RobotPlan` (모듈 ⑫): `summary`, `feasible`, `reason`, `steps: list[PlanStep]` — `PlanStep`은 `warehouse_interfaces/PlanStep`과 같은 필드.
- 백엔드: `openai`(기본 모델은 설정 파일에서, API 키는 `.env`의 `OPENAI_API_KEY`) / `offline`(규칙 기반, API 키 불필요, 테스트·오프라인 실습용). 런치 인자 `llm:=openai|offline`, 키가 없으면 offline으로 전환하고 알린다.
- 계획 검증: 알려진 장소·물체만 허용, `place` 앞에 `pick` 필요, 들고 있지 않은데 `place` 금지 등. 검증 실패 시 실행하지 않고 이유를 narration으로 알린다.

## 6. 설정 파일 위치

| 파일 | 내용 |
|---|---|
| `dualarm_mujoco/worlds/warehouse.yaml` | 창고 월드 단일 원본 |
| `dualarm_mujoco/config/bridge.yaml` | 브리지 주기·제한·인터록·카메라 설정 |
| `dualarm_navigation/config/locations.yaml` | 이름 있는 장소 |
| `dualarm_navigation/config/nav2_params.yaml`, `slam_params.yaml` | Nav2 / SLAM |
| `dualarm_navigation/maps/warehouse.yaml/.pgm` | 월드에서 생성한 정적 지도 |
| `dualarm_moveit_config/config/*` | SRDF, kinematics, joint_limits, controllers, OMPL |
| `warehouse_skills/config/skills.yaml` | 도킹·파지 오프셋, 허용 오차, 타임아웃 |
| `warehouse_tasks/config/tasks.yaml` | LLM 모델·백엔드, 한국어 별칭, 프롬프트 설정 |

## 7. 실행 명령 (래퍼 `scripts/camp`)

| 명령 | 내용 |
|---|---|
| `camp build` | colcon build |
| `camp minimal` | 모듈 ② 최소 브리지 예제 |
| `camp display` | URDF + RViz (모듈 ③) |
| `camp drive` | MuJoCo 브리지 + robot_state_publisher (+RViz) |
| `camp teleop` | 키보드 주행 |
| `camp slam` | SLAM Toolbox 지도 작성 |
| `camp nav [map:=...]` | 정적 지도 + AMCL + Nav2 |
| `camp moveit` | MuJoCo + MoveIt |
| `camp start` | 전체 (MuJoCo + Nav2 + MoveIt + 스킬 + Task Manager + 대시보드) |
| `camp goal X Y YAW` / `camp go NAME` | Nav2 목표 |
| `camp arm left|right|both POSE` | 이름 자세 |
| `camp pick COLOR [STATION]` / `camp place STATION` | PickPlace 스킬 |
| `camp say "빨간 상자를 적재 작업대로 옮겨"` | 자연어 지령 |
| `camp console` | 텍스트 지령 콘솔 |
| `camp test` | 단위 테스트 |

공통 런치 인자: `viewer:=true|false`(MuJoCo 뷰어), `rviz:=true|false`, `actors:=true|false`, `llm:=openai|offline`, `camera_backend:=render|truth`.
