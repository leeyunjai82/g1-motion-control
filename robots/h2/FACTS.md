# H2 확인 사실표

H2 지원(4단계)을 위해 **출처로 확인한 값**과 **확인이 필요한 값**을 나눠 기록합니다.
추측한 값은 넣지 않습니다. 실기에서 확인하면 이 표를 갱신한 뒤 config(`robot.yaml`)로 옮깁니다.

> 현재 `robots/h2/robot.yaml` 은 `enabled: false` — 모든 스크립트/서버가 **실행 거부** (명령 경로 닫힘).
> 아래 "확인 필요" 항목이 모두 정리되기 전에는 열지 않습니다.
>
> **오프라인 검증 (2026-10):** `robot.yaml` 값으로 공통 `robot_arm.py` 를 가짜 DDS 로 초기화한 LowCmd 35 슬롯
> (mode/q/dq/tau/kp/kd, weight 31, head 이득, 초기화 슬롯 0–30) 이 **h2-motion-control.red `robot_arm.py`(실기 동작)와 완전히 동일**.

## 작업 범위 (2026-10 결정)

- **제자리에 서서** 박스를 잡아 **옆으로 옮겨 내려놓기**만 합니다.
- 보행 중 잡기, 들고 걷기, 마커 추종은 하지 않습니다 (`/follow`, `/loco` 비활성 예정).
- 시퀀스: G1 박스 잡기 흐름 그대로 — 인식 → **허리 yaw 로 박스 쪽 정렬** → 접근 → 잡기 → 들기 → **건네기 (정면/좌/우 yaw)** (G1 box 모드: 2초 대기 후 손 벌림) → 복귀.
- **허리 pitch/roll 0, yaw 만 사용** (앞으로 숙이지 않아 넘어짐 방지) — `robot.yaml grab.waist_base_pitch_deg: 0`, `waist_locked: false`. H2 waist_yaw 한계 ±1.7453 rad (±100°, URDF).
- ⚠️ yaw 를 실제로 움직이므로 **허리 슬롯 순서(12 = yaw)가 틀리면 숙이거나 기울어집니다.** 첫 실행은 매달기/지지 상태에서 작은 각도(5° 정도)로 12번이 몸통을 수직축으로 돌리는지 확인.
- **손바닥 마주보기**로 양손 파지 (G1 과 같은 단위 회전 — 아래 "손 자세" 참고).
- **테이블을 높여서** 맞춤 (아래 "시퀀스 IK 가능 범위" 참고).

## 출처

| 기호 | 출처 |
| --- | --- |
| SDK | `unitreerobotics/unitree_sdk2_python` master `814556d` (2026-09-21) |
| SDK-arm | 위 SDK `example/h2/high_level/h2_arm_sdk_dds_example.py` (최종 수정 `e6cd8af`, 2026-08-20) |
| SDK-low | 위 SDK `example/h2/low_level/h2_ankle_swing_example.py` (관절 순서 수정 `65691c8`, 2026-07-21) |
| XR | `unitreerobotics/xr_teleoperate` master `817fb00` (2026-09-07) `teleop/robot_control/robot_arm.py` |
| ROS | `unitreerobotics/unitree_ros` `5994d4f` (2026-09-30) `robots/h2_description/H2.urdf` |
| H2R | `leeyunjai82/h2-motion-control.red` `8e7996d` — **실기에서 simulator 동작 확인** (사용자 보고, 2026-10) |

## 확인된 값

| 항목 | 값 | 근거 |
| --- | --- | --- |
| DDS 메시지 | `unitree_hg` `LowCmd_` / `LowState_` (motor 35 슬롯 고정 배열) | SDK, XR, H2R(실기) |
| 팔 제어 토픽 | `rt/arm_sdk` | SDK-arm, XR, H2R(실기) |
| arm_sdk weight 슬롯 | `motor_cmd[31].q` | SDK-arm, XR, H2R(실기) |
| 어깨·팔꿈치 | 왼 15 ShoulderPitch, 16 Roll, 17 Yaw, 18 Elbow / 오른 22–25 같은 순서 | SDK-arm, SDK-low, XR, H2R 모두 일치 |
| 헤드 | 29 HeadPitch, 30 HeadYaw | SDK-arm(`e6cd8af` 에서 수정), SDK-low, XR, H2R |
| LocoClient | `unitree_sdk2py.h2.loco.h2_loco_client.LocoClient` | SDK. **현재 requirements 고정 커밋 `f559291` 에는 없음 → SDK 업그레이드 필요** |
| FSM ID | 0 ZeroTorque, 1 Damp, 2 Squat, 3 Sit, 4 StandUp, **601 Start** | SDK `h2_loco_client.py` (`Start()` 가 500 → 601 로 바뀐 커밋 `6dec8b2`, 2026-07-02) |
| 기립 순서 | 1 → (5초) → 4 → (10초) → **703** (이 repo, 2026-10-07 변경). H2R 는 → 601 | 601 에서는 arm_sdk 가 안 먹음 (실기) |
| ~~FSM 601 에서 arm_sdk 동작 (EnableArmSDK 없이)~~ | **실기 2026-10-07 재확인: 동작 안 함** (h2-motion-control.red simulator.py 그대로 실행해도 안 움직임) | 사용자 실기 |
| FSM 조회 API | `GetFsmId` 7001, `GetAvailableFsmIds` 7008 (ID+이름), `GetArmSdkStatus` 7007 | SDK `h2_loco_api.py`. 실기 응답 여부는 확인 필요 |
| URDF | `robots/h2/H2.urdf` = ROS `H2.urdf` (H2R 의 URDF 와 바이트 동일) | ROS, H2R |
| 메시 | ROS `5994d4f` (H2R 대비 `head_pitch_link.stl`, `head_yaw_link.stl` 만 갱신 — 시각용) | ROS |
| **`LowState_.mode_machine`** | **1** (`mode_pr` 0, `version` [0, 0]) | 실측 2026-10-07 `utils/check_robot_id.py` 부팅 직후, 20회 관측 모두 1 → `robot.yaml identity.mode_machine` |
| 살아 있는 모터 슬롯 | 0–30 (31개). 31–34 온도·전압 0 (weight 슬롯 31 에 모터 없음) | 실측 2026-10-07 |
| 부팅 직후 다리 0–3, 6–9 | `mode 10`, `vol 0.00` (온도는 정상 출력). 발목 4/5/10/11·상체는 `mode 1`, 73.5–74.0 V | 실측 2026-10-07. mode 10 의미는 확인 필요 (이번 범위에서 다리 미사용) |
| 기본 팔 자세 (FSM 601, arm_sdk 전) | 슬롯 15–28 [deg] = [6.2, 23.7, -25.8, 63.8, 0.6, 0.3, 0.1, 3.2, -19.9, 18.4, 62.6, -0.3, 0.7, 0.1] | 실측 2026-10-07 거치대, 1회 → `robot.yaml default_arm_deg`. 좌우 어깨 roll(16 +, 23 −)·yaw(17 −, 24 +) 부호가 대칭 → 15–18/22–25 배치와 일치. 손목 6개는 모두 ≈0 이라 순서 판정 불가 (jog 로 확인) |
| 601 기립 후 다리 0–3, 6–9 | 여전히 `mode 10`, `vol 0.00` → 부팅 직후와 같음 (기립·밸런스 중에도 이 값) | 실측 2026-10-07 |
| 실기 FSM 목록 (`GetAvailableFsmIds` 7008) | 0 Invalid, 1 Passive, 2 Protection, 3 Sit, **4 FixStand**, 5 HybridPassive, 502 HumanMimic, 503 HumanMimic2, 100 BeyondMimic, **601 HybridWalk**, 701 WalkNew, **703 PhaseWalk** (+ 100xxx/502xxx/503xxx 모션 ID 다수) | 실측 2026-10-07 `utils/robot_state.py` (code 0 응답) |
| arm_sdk 지원 FSM | SDK master `814556d` 예제 `ARM_SDK_SUPPORTED_FSM_IDS = {4, 703}` + `EnableArmSDK()` (601 은 목록에 없음) | SDK 예제. 601 에서 jog 했을 때 팔이 안 움직였다는 사용자 보고 있음 (2026-10-07) |
| FSM 4 상태 | `GetFsmId` (0, 4), `GetFsmMode` (0, 0), **`GetArmSdkStatus` (0, True)** — EnableArmSDK 호출 없이 True. rt/arm_sdk 240 Hz 수신, weight 1.0, 팔 명령 vs 실측 차이 ≤ 1.3° | 실측 2026-10-07 `start_fsm.sh no-bal` 후. jog 추종 여부는 확인 중 |
| LowCmd 헤더 `mode_machine` | xr_teleoperate `817fb00` `H2_ArmController`: `msg.mode_pr = 0`, `msg.mode_machine = lowstate.mode_machine`. 이 repo·H2R 는 0 을 보냄 → `robot.yaml lowcmd.mode_machine: lowstate` 로 변경 (2026-10-07). 실기 효과는 `utils/arm_sdk_test.py` 로 확인 중 | XR, H2R |
| 허리 슬롯 (충돌 추가) | xr_teleoperate `817fb00` "[fix] H2 waist's joint index": **12 WaistRoll, 13 WaistPitch, 14 WaistYaw** — 현재 yaml(12 yaw / 13 roll / 14 pitch, H2R 기준)과 다름 → `/check` jog 로 실기 판정 | XR |
| **EnableArmSDK 필요** | 호출 없이: FSM 4·601, LowCmd mode_machine 0·1, 메시지 구성(이 repo / 공식 예제) 모두 팔 0.0° (`GetArmSdkStatus` 는 True 로 나옴). 공식 예제(SDK `814556d`, `EnableArmSDK` 결과 0) 에서는 팔이 움직임 — Stage 1(0자세로) 중 팔이 부딪혀 충돌음, 1.1 s 에 Ctrl+C | 실기 2026-10-07 → `robot.yaml sdk.enable_arm_sdk: true` (robot_arm.py 송신 전 호출, arm_server 종료 시 weight 0 후 Disable) |
| **허리 12–14 는 arm_sdk 로 안 움직임** | FSM 4 + EnableArmSDK 상태에서 팔은 움직이는데 허리 3축 모두 반응 없음 (simulator 에서 명령). SDK `814556d` 공식 예제의 `upper_body_joints` 도 팔 14 + 머리 2 뿐 (허리 없음) | 실기 2026-10-07 사용자 → `grab.waist_locked: true` (정면 건네기만) |
| 팔 추종 | `arm_sdk_test --enable` 슬롯 15 +5° 명령 → +2.7° (kp 80 / kd 3, robot.yaml gains 의 kp_low). xr_teleoperate H2 는 팔 kp 140 / kd 3, 손목 50 / 2 | 실기 2026-10-07 |
| **팔 슬롯 15–28 매핑 ✓** | 15 L ShoulderPitch, 16 Roll, 17 Yaw, 18 Elbow, 19 WristRoll, 20 WristPitch, 21 WristYaw / 22–28 오른쪽 같은 순서 — `robot.yaml joints.map` 그대로 맞음. 23(오른 roll) +8° 는 몸통 쪽이라 3.1° 에서 막힘, −8° 는 −5.4° | 실기 2026-10-07 `arm_sdk_test.py --enable --slot N --deg 8`, 사용자 육안 확인 |
| FSM 703 (PhaseWalk) | `start_fsm.sh 703` (4 → 703) Result 0, `GetFsmId` (0, 703). 다리는 601 과 같은 밸런싱 서기 (걷지 않음, 사용자 확인). EnableArmSDK 후 팔 15 +8° → +6.8° ✓ (게인 140/3). **허리 12/13/14 +5° → 0.0° (안 움직임)** | 실기 2026-10-07 |
| 허리 결론 | FSM 4·703 모두 arm_sdk 로 허리 안 움직임 → H2 는 이 방식으로 허리 제어 불가, `waist_locked: true` 유지 | 실기 2026-10-07 |
| 기본 팔 자세 (FSM 703) | [8.2, 18.2, -12.1, 73.7, 0.8, 0.9, 0.1, 8.1, -18.2, 12.4, 73.4, -0.3, 1.2, -0.5] | 실측 2026-10-07 → `robot.yaml default_arm_deg` (601 값 대체) |
| 온도·전압 추이 | 어깨 pitch 15/22: 52 (부팅) → 53 (601) → 59 → **61/60 °C** (703 진입 직후). 허리 55–56 °C. 전압 74.0 → 71.0–71.5 V | 실측 2026-10-07. 한계값 자료 없음 (확인 필요) |
| D435i 숙임각 | 39.1° (IMU, 중력 기준), 좌우 +2.2°. 로봇 IMU pitch −0.3° / roll +0.2° (703) → `camera.pitch_deg 39.1` | 실측 2026-10-07 `utils/cam_tilt.py` |
| D435i 마커 보정 | depth 기준 2점(0.45/0.55, 테이블 렌즈 아래 0.51 m): 축척 0.101/0.100, 숙임 35.2°, camera.x 0.075 (두 점 오차 0). IMU 는 같은 때 36.3° (앞서 39.1° — 로봇 자세 차이 추정). PnP(마커 크기) 거리는 이 조건에서 10 cm 이동을 5 cm 로 냄 → 사용 안 함 | 실측 2026-10-07 `utils/cam_marker_check.py` |
| IK 축소 모델 | 다리 12 + 허리 3 + head 2 잠금, 팔 14, `L_ee`/`R_ee` = `*_wrist_yaw_joint` + x 0.05 m | XR `robot_arm_ik.py`, H2R. G1 과 같은 구조 |

## 공식 자료끼리 충돌 → h2-motion-control.red 실기 기준으로 결정 (2026-10)

| 항목 | SDK-arm | SDK-low | XR | H2R |
| --- | --- | --- | --- | --- |
| 손목 19/20/21 (오른 26/27/28) | Roll/Pitch/Yaw | **Yaw/Pitch/Roll** | Roll/Pitch/Yaw | Roll/Pitch/Yaw ← 채택 (사용자 실기 확인) |
| 허리 12/13/14 | Yaw/Roll/Pitch | Roll/Pitch/Yaw | Roll/Pitch/Yaw | **Yaw/Roll/Pitch** ← 채택 (사용자 실기 확인) |
| 발목 4/5 (우리 기능 무관) | Pitch/Roll | Roll/Pitch | Roll/Pitch | Roll/Pitch |
| arm_sdk 허용 FSM | {4, 703} + `EnableArmSDK()` 필요 | — | (검사 없음) | 601 에서 동작 (실기) |

**실기 확인 방법** (H2R simulator, 관절 모드): 관절 하나만 조금 움직이고 실제로 어느 축이 도는지 확인.
- 허리 12 → 몸통이 좌우로 도는가(yaw) / 옆으로 기우는가(roll) / 앞뒤로 숙는가(pitch)
- 손목 19 → 손등 축 비틀림(roll) / 손바닥 면 꺾임(pitch) / 좌우 꺾임(yaw)
- 주의: 관절 모드는 순서가 틀려도 "어느 모터인가는" 움직여 정상처럼 보입니다. **IK 는 URDF 체인 순서(roll→pitch→yaw)로 19/20/21 에 넣으므로 순서가 다르면 손 자세가 틀어집니다.**

## 확인 필요 (자료 없음)

| 항목 | 비고 |
| --- | --- |
| 모터 온도 한계 | 부팅 직후(서 있지 않음) 허리 12–14·어깨 pitch 15/22 가 50–52 °C, 손목 20/21/27/28 이 41–48 °C (실측 2026-10-07). 경고/보호 온도 자료 없음 |
| 펌웨어 버전 | 7109(EnableArmSDK) 필요 여부가 펌웨어에 따라 달라질 수 있음 |
| 손 종류 / SDK | H2R 는 G1 과 같은 `mandro3.py` 포함 — 실제 장착 손 확인 |
| D435i 장착 위치·외부 파라미터 | **외부 D435i 를 mini PC USB 에 연결** (G1 과 같은 방식, 헤드 내장 카메라 미사용). `robot.yaml camera` 는 **임시값**(가슴 고정 가정: x 0.10, z 0.35, 아래 50°) — 실장착 후 4개 값만 수정. 목 관절이 있으므로 torso 고정 권장 |
| H2 기본(헤드) 카메라 | 2차 자료상 140° 광각 **바이노큘러 RGB** (unitree.com 직접 확인 못 함). 깊이 출력·공장 intrinsics API 근거 없음. teleimager(`f883d6a`)는 컬러 JPEG 만 전송 (depth/intrinsics 는 RealSense 백엔드에서만 읽고 전송 안 함). 일부 판매처 상품명의 "depth camera" 는 실체 미확인 → H2 PC 에서 `lsusb` / `rs-enumerate-devices` 로 RealSense 존재 여부 확인 필요 |
| 테이블 높이 | 아래 시퀀스 IK 가능 범위 + 실측 pelvis 높이로 결정 |
| 서 있을 때 pelvis 높이 | H2R `GROUND_TO_PELVIS = 0.782` 는 G1 값 복사. URDF 다리 편 자세 ankle_roll_link = pelvis −0.985 m |
| IMU | URDF 에 `imu_in_torso`, `imu_in_pelvis` 링크. `rt/lowstate.imu_state` 가 어느 쪽인지 확인 필요 |

## IK 도달 범위 (오프라인 근사, 2026-10)

조건: 양손 y = ±0.17 m, 손 자세 = 단위 회전(G1 `wrist_params` 기본값), 허리·헤드 고정, pelvis 기준.
pinocchio DLS 로 계산 (서버의 casadi 풀이와 동일하지 않음 — 경향 확인용).

| | G1 | H2 |
| --- | --- | --- |
| 1 cm / 5° 이내로 풀리는 z (x 0.2–0.4 m) | z ≥ 0.0 ~ +0.10 | **z ≥ +0.20** |
| 위치만 (자세 무시) | z ≥ −0.10 | z ≥ 0.0 |

→ G1 과 같은 손 자세면 H2 는 pelvis 위 0.2 m 이상에서만 잡을 수 있습니다. 테이블 높이 / 허리 pitch / 손목 자세 중 무엇으로 맞출지 결정 필요.

## 손 자세 (단위 회전 = 손바닥 마주보기)

zero 자세에서 손목 프레임 회전 = 단위 행렬 (G1·H2 모두 URDF FK 로 확인).
손 메시의 가장 얇은 주축(= 손바닥 법선) 이 손목 프레임 기준 거의 y 축:
- H2 `left_hand_link.stl`: 크기 x 0.164 / y 0.086 / z 0.114 m, 얇은 축 (0.22, 0.98, −0.04)
- G1 `left_rubber_hand.STL`: 크기 x 0.132 / y 0.067 / z 0.106 m, 얇은 축 (0.31, 0.95, 0.03)

→ 단위 회전이면 양손 손바닥 면이 ±y (서로 마주봄). G1 이 실기에서 쓰는 자세와 같은 조건. (손바닥/손등 방향 구분은 3D 뷰어·실기로 최종 확인)

## 시퀀스 IK 가능 범위 (오프라인 근사, 허리 0, 손 마주보기)

박스 W 0.28 × H 0.12 m (예시 — 실제 박스 치수로 다시 계산), G1 `robot_server` 오프셋 그대로
(GRIP_EXTRA −0.05, APPROACH_EXTRA 0.10, GRAB_Z_OFFSET 0.08, 들기 = 윗면 + 0.15).
접근 → 하강 → 잡기 → 들기 → 옆 이동 → 내리기 → 손 벌림 7점을 이어 풀어 전부 1 cm / 5° 이내면 가능.

| 옆 이동 dy | 가능한 박스 윗면 z (pelvis 기준) × 박스 중심 x |
| --- | --- |
| 0 | top +0.15: x 0.25–0.40 / top +0.20: x 0.25–0.45 |
| 0.10 m | top +0.15: x 0.25–0.35 / top +0.20: x 0.25–0.40 |
| 0.15 m | top +0.15: x 0.25–0.30 / top +0.20: x 0.25–0.40 |
| 0.20 m | top +0.20: x 0.25–0.30 |
| 0.25 m | 거의 불가 |

권장 시작점: **박스 윗면 = pelvis + 0.20 m, 박스 중심 x 0.30–0.35 m, 옆 이동 ≤ 0.15 m**.
테이블 높이 = (서 있을 때 pelvis 높이, 실측) + 0.20 − 박스 높이.
(근사 계산이라 경계 칸은 들쭉날쭉함 — 실제 서버 IK(casadi)로 같은 점을 다시 확인할 것)

## 건네기 IK 가능 범위 (오프라인 근사, 허리 0, 정면 건네기 HANDOVER_X 0.30)

접근 → 하강 → 잡기 → 들기 → 건네기(x 0.30, 들기 높이) → 손 벌림 6점, 박스 W 0.28 × H 0.12 (예시).

| 박스 중심 y | top +0.10 | top +0.15 | top +0.20 | top +0.25 |
| --- | --- | --- | --- | --- |
| 0.00 | x 0.25 | x 0.25–0.40 | x 0.25–0.40 | x 0.30 |
| ±0.05 | x 0.25 | x 0.25–0.35 | x 0.25–0.40 | 불가 |
| ±0.10 | 불가 | x 0.25–0.35 | x 0.25–0.40 | 불가 |

→ 허리 yaw 정렬 없이도 박스가 정면 ±10 cm 안이면 가능. **yaw 정렬을 쓰므로** 실제로는 박스가 옆에 있어도 몸통(torso) 기준 정면으로 돌려서 잡는다 (카메라·팔이 모두 torso 에 붙어 있어 IK 는 torso 기준 그대로). 권장: 박스 윗면 pelvis + 0.20 m, x 0.30–0.35 m.
건네는 높이 = 박스 윗면 + 0.15 = **pelvis + 약 0.35 m** (바닥 기준은 서 있을 때 pelvis 실측 후 확정).

⚠️ box 모드 건네기는 G1 과 같이 **받았는지 확인하지 않고 2초 뒤 손을 벌립니다** (`HANDOVER_HOLD_SEC`).
아무도 안 받으면 그 높이에서 박스가 떨어집니다. H2 는 G1 보다 높은 위치라 받는 사람이 있을 때만 실행할 것.

## 시뮬레이터 검증 (start_sim.sh, 2026-10) — 실제 서버 코드 그대로

`sim/fake_robot.py` (기구학 가짜 로봇) + `sim/sim_server.py` (가짜 detect_box) 로 robot_server / arm_server 를 수정 없이 실행.
위 "IK 가능 범위" 표들은 손 목표를 박스 중심 x 에 둔 근사이고, 실제 box 모드는 손 목표 x = 박스 x − 0.15 (`GRAB_X_OFFSET`) —
**아래 시뮬 결과를 우선**한다.

| 시나리오 (박스 W 0.28 · D 0.20 · H 0.12, 윗면 pelvis+0.20) | 결과 |
| --- | --- |
| 정면 x 0.45, 정면 건네기 | 10단계 완료. 단계 끝 손 목표 오차 ≤ 1.3 cm |
| 왼쪽 y 0.10, 좌측 건네기 | 허리 yaw 12.5° 정렬 → 재검출 시 torso y 0.000 → 잡기 오차 0.1–0.3 cm → yaw 30° 건네기 → 복귀 |

### 발견 1 — 카메라→IK 좌표 차이 (수정함)

잡기 코드는 카메라에서 바꾼 torso_link 좌표를 그대로 IK(pelvis 기준) 목표로 쓴다. 두 좌표 차이(z)는 G1 4.4 cm, **H2 12.3 cm**.
G1 은 `grab.z_offset 0.08` 이 실험으로 흡수 (robot_server 주석). H2 에 같은 방식을 쓰면:

| (윗면 pelvis+0.20, H 0.12) | G1 방식 그대로 | `exact_ik_frame: true` + `z_offset 0.036` |
| --- | --- | --- |
| 위쪽 접근 손 높이 − 박스 윗면 | **−2.3 cm (박스와 충돌)** | +9.7 cm |
| 잡기 손 높이 − 박스 옆면 중간 | −4.3 cm | +3.7 cm (G1 실효 +3.6 cm 와 같음) |
| 들기 손 높이 − 박스 윗면 | +2.7 cm | +14 cm |

→ H2 는 `frames.exact_ik_frame: true` (torso + pelvis_to_torso 로 정확 변환), `grab.z_offset 0.036` (= G1 실효값) 로 시작. 실기 실험으로 재조정.
G1 은 `exact_ik_frame: false` 로 기존 동작 그대로.

### 발견 2 — 카메라 장착 높이와 시야

detect_box 의 K(640×480)로 계산. 박스 중심 x 0.45 m 일 때 화면에 박스 윗면이 다 들어오는 좌우 범위:

| 장착 (torso 기준 x, z, 아래각) | 좌우 범위 |
| --- | --- |
| 0.10, 0.35, 50° (첫 임시값) | ±0.05 m |
| 0.06, 0.50, 52° (**현재 임시값**) | ±0.12 m |
| 0.05, 0.60, 58° | ±0.18 m |

카메라가 박스에 너무 가까우면 박스가 화면 대부분을 차지해 조금만 옆에 있어도 잘린다 — 장착 시 참고.

## 실기 모터 번호 확인 절차 (최종 조정 전)

`ROBOT_CHECK=1 ROBOT=h2 ./start_simulator.sh real` → `http://<pc>:8000/check` (README "모터 번호 확인" 참고).

| 확인 대상 | 방법 | 이번에 특히 볼 것 |
| --- | --- | --- |
| 허리 12/13/14 | 명령 ±5° | 12 = yaw (몸통 수직축 회전) 인지 — 공식 자료끼리 다름 |
| 손목 19–21, 26–28 | 명령 ±5° | roll/pitch/yaw 순서 — 공식 자료끼리 다름 |
| 어깨·팔꿈치 15–18, 22–25 | 명령 ±5° | (자료 일치, 재확인) |
| 헤드 29/30, 다리 0–11 | 제어권 반납 후 손으로 → Δ | 발목 4/5 순서 |
| `mode_machine` | 화면 상단 | `robot.yaml identity.mode_machine` 에 기입 |
| 기본 팔 자세 | FSM 601 기립, arm_server release 상태에서 각도 | `default_arm_deg` 14개 |

결과 파일(`robots/h2/joint_check_*.json`)로 `robot.yaml` 을 고친 뒤 `enabled: true` 로 바꾼다.
