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
- 시퀀스: G1 박스 잡기 흐름 그대로 — 인식 → 접근 → 잡기 → 들기 → **정면 건네기** (G1 box 모드: 2초 대기 후 손 벌림) → 복귀.
- **허리 0 고정** (yaw/roll/pitch 모두, 넘어짐 방지) — `robot.yaml grab.waist_locked: true`, `waist_base_pitch_deg: 0`. 박스 쪽 yaw 정렬·좌우 건네기 없음.
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
| 기립 순서 | 1 → (5초) → 4 → (10초) → 601 | H2R `utils/init_fsm.py` `stand` — 실기 사용 |
| **FSM 601 에서 arm_sdk 동작** | 동작함, `EnableArmSDK()` 호출 없이 | H2R(실기, `start_fsm.sh stand` 후 simulator). 펌웨어 버전 확인 필요 |
| FSM 조회 API | `GetFsmId` 7001, `GetAvailableFsmIds` 7008 (ID+이름), `GetArmSdkStatus` 7007 | SDK `h2_loco_api.py`. 실기 응답 여부는 확인 필요 |
| URDF | `robots/h2/H2.urdf` = ROS `H2.urdf` (H2R 의 URDF 와 바이트 동일) | ROS, H2R |
| 메시 | ROS `5994d4f` (H2R 대비 `head_pitch_link.stl`, `head_yaw_link.stl` 만 갱신 — 시각용) | ROS |
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
| `LowState_.mode_machine` 값 | 로봇 식별 안전장치 기준값. `python utils/check_robot_id.py` 로 측정 (SDK-low 가 이 값을 "H2 type" 으로 출력) |
| 펌웨어 버전 | 7109(EnableArmSDK) 필요 여부가 펌웨어에 따라 달라질 수 있음 |
| 손 종류 / SDK | H2R 는 G1 과 같은 `mandro3.py` 포함 — 실제 장착 손 확인 |
| D435i 장착 위치·외부 파라미터 | **외부 D435i 를 mini PC USB 에 연결** (G1 과 같은 방식, 헤드 내장 카메라 미사용). `robot.yaml camera` 는 **임시값**(가슴 고정 가정: x 0.10, z 0.35, 아래 50°) — 실장착 후 4개 값만 수정. 목 관절이 있으므로 torso 고정 권장 |
| H2 기본(헤드) 카메라 | 2차 자료상 140° 광각 **바이노큘러 RGB** (unitree.com 직접 확인 못 함). 깊이 출력·공장 intrinsics API 근거 없음. teleimager(`f883d6a`)는 컬러 JPEG 만 전송 (depth/intrinsics 는 RealSense 백엔드에서만 읽고 전송 안 함). 일부 판매처 상품명의 "depth camera" 는 실체 미확인 → H2 PC 에서 `lsusb` / `rs-enumerate-devices` 로 RealSense 존재 여부 확인 필요 |
| 테이블 높이 | 아래 시퀀스 IK 가능 범위 + 실측 pelvis 높이로 결정 |
| 서 있을 때 pelvis 높이 | H2R `GROUND_TO_PELVIS = 0.782` 는 G1 값 복사. URDF 다리 편 자세 ankle_roll_link = pelvis −0.985 m |
| 기본 팔 자세 (`DEFAULT_ARM_DEG`) | 601 기립 상태에서 실측 |
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

→ 허리 yaw 정렬 없이도 **박스가 정면 ±10 cm 안이면** 잡기·건네기 가능. 권장: 박스 윗면 pelvis + 0.20 m, x 0.30–0.35 m.
건네는 높이 = 박스 윗면 + 0.15 = **pelvis + 약 0.35 m** (바닥 기준은 서 있을 때 pelvis 실측 후 확정).

⚠️ box 모드 건네기는 G1 과 같이 **받았는지 확인하지 않고 2초 뒤 손을 벌립니다** (`HANDOVER_HOLD_SEC`).
아무도 안 받으면 그 높이에서 박스가 떨어집니다. H2 는 G1 보다 높은 위치라 받는 사람이 있을 때만 실행할 것.
