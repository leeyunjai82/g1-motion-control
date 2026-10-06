# Unitree Motion Control (G1 / H2)

**Ubuntu 24.04 + external Intel mini PC + Intel RealSense D435i** 환경에서 Unitree 휴머노이드를 제어하는 통합 패키지입니다.
마커 기반 접근, 박스 파지, 들고 걷기, 웹 모션 에디터를 기능별 작은 HTTP 서버로 나눠 구성합니다.

- 공통 코드는 `common/`, 로봇마다 다른 파일은 `robots/<ROBOT>/` 에 둡니다.
- 실행할 때 **`ROBOT=g1`** 처럼 로봇을 지정합니다. **지정하지 않으면 실행을 거부합니다** (기본값 없음).
- 현재 지원: **`g1`** (G1 29 DOF). H2 는 실기 확인 전까지 명령 경로를 열지 않습니다.
- 기준 소스: `leeyunjai82/g1-motion-control.red` main `731075b` (RedHat 판) — Ubuntu 용으로 경로/설치만 정리, 제어 값은 동일.

- 설치 : [**INSTALL.md**](./INSTALL.md)
- 내부 구조 (관절 맵, 모션 JSON, IK, DDS, FSM) : [**TECH.md**](./TECH.md)

---

## ⚠️ 안전 수칙 (먼저 읽기)

1. **스탠드 고정** — 지지 스탠드를 로봇 어깨에 단단히 묶어 흔들리지 않게 합니다.
2. **`ROBOT=g1 ./start_fsm.sh stand`** 는 즉시 모터에 힘이 들어갑니다. **사람이 로봇을 붙잡고** 실행하세요.
3. **`ROBOT=g1 ./start_fsm.sh sit`** 는 천천히 주저앉습니다. 팔을 먼저 몸 옆으로 내리고 **내려가는 동안 계속 지지**하세요.
4. 작업 공간에 사람·장애물이 없게 하고, **비상정지(E-STOP)** 위치를 미리 확인하세요.
5. **연결된 로봇과 `ROBOT` 값이 같은지** 실행 전에 확인하세요. 로봇은 한 번에 한 대만 연결합니다.

---

## 구성

자원(하드웨어)마다 소유 프로세스는 하나. 나머지는 HTTP 로 통신합니다.

```
rs_stream      :50001   RealSense 카메라 (단독 점유) → MJPEG / depth API
detect_marker  :50011   ArUco 마커 자세 (rs_stream 사용)
detect_box     :50010   박스 인식, 파지점 (rs_stream 사용)
arm_server     :50022   rt/arm_sdk 단독 점유 — 팔/허리, IK, hold/release
robot_server   :50000   오케스트레이터 — 잡기 시퀀스, 마커 추종, 웹 UI
dashboard      :50003   3D URDF 뷰어 / 관절 상태 (rt/lowstate 읽기 전용)
simulator      :8000    모션 에디터 (Joint + IK), arm_server 경유
launcher       :80      FSM 버튼 + start_robot.sh 실행 웹
```

규칙:
- **팔/허리**: 모든 프로세스는 `arm_server` 를 거칩니다 (`rt/arm_sdk` 는 publisher 하나만 허용).
- **보행**: `LocoClient` 는 다중 클라이언트 RPC — 직접 써도 되지만 이동 명령은 한 곳에서만 보냅니다.

## 빠른 시작

```bash
export ROBOT=g1             # 모든 스크립트가 이 값을 요구합니다

# 1. 자세 (로봇을 붙잡고!)
./start_fsm.sh stand        # 또는: sit / bal / no-bal / damp

# 2. 전체 스택 (서버 6개, 의존 순서대로)
./start_robot.sh
#   → 제어 UI : http://<pc-ip>:50000/
#   → 3D 뷰어 : http://<pc-ip>:50003/dashboard

# 3. 모션 에디터 — 모드 지정 필수 (virtual | real)
./start_simulator.sh real          # 실기 (start_robot.sh 와 함께 써도 됨 — arm_server 재사용)
#   → 에디터  : http://<pc-ip>:8000/        모터 번호 확인 : http://<pc-ip>:8000/check

# (선택) 웹 런처 — FSM 버튼 + start_robot.sh 실행 (포트 80, sudo)
./launcher.sh
```

## Motion Editor 모드 (`start_simulator.sh`)

| 명령 | 모드 | 띄우는 것 |
| --- | --- | --- |
| `ROBOT=h2 ./start_simulator.sh virtual` | **가상** — URDF/메시 3D + fake_robot, 로봇 없이 (DDS 도메인 1) | fake_robot, arm_server, dashboard, simulator |
| `ROBOT=g1 ./start_simulator.sh real` | **실기** — 실제 로봇이 움직임 | arm_server·dashboard(떠 있으면 재사용), simulator |
| `ROBOT_CHECK=1 ROBOT=h2 ./start_simulator.sh real` | **실기 · 모터 번호 확인 전용** — `enabled: false` 로봇 | 위와 같음. 잡기·보행 서버는 실행 거부, 시작 시 `yes` 확인 |

### 모터 번호 확인 (`http://<pc-ip>:8000/check`)

1. 로봇을 매달거나 지지하고 주변을 비운다. E-STOP 을 손에 둔다.
2. `ROBOT_CHECK=1 ROBOT=h2 ./start_simulator.sh real` (H2) — 상단에 `mode_machine` 이 표시된다 (robot.yaml `identity` 기준값).
3. **팔·허리** (명령 이동): [기준 잡기] → 슬롯의 [+]/[−] (한 번 5°, 기준 대비 최대 ±15°, 서버에서 제한).
   실제 로봇에서 움직인 관절과 오른쪽 3D(robot.yaml 이름으로 그림)에서 움직인 관절이 같으면 ✓, 다르면 ✗ + 실제 관절을 메모.
4. **다리·헤드** (읽기 전용): [제어권 반납] (또는 FSM Damp) → [기준 잡기] → 관절을 손으로 움직이면 변한 슬롯이 노랗게 표시.
5. [결과 저장] → `robots/<robot>/joint_check_<날짜시각>.json` → 이 결과로 robot.yaml 관절 맵을 최종 조정.

가상 모드에서 같은 화면으로 절차를 미리 연습할 수 있다 (가상은 robot.yaml 대로 움직이므로 항상 ✓).

## 시뮬레이터 (로봇 없이 시험)

```bash
ROBOT=h2 ./start_sim.sh            # 가상 카메라 + 가상 박스  → http://<pc-ip>:50010/
ROBOT=h2 ./start_sim.sh real-cam   # 실물 D435i + 실제 박스 인식(YOLO), 로봇만 가상 → http://<pc-ip>:50012/
```

- `real-cam`: rs_stream + detect_box 를 실물로 띄우고, 인식 결과로 가상 로봇이 잡기 시퀀스를 돈다.
  인식 좌표 → 로봇 좌표 변환은 robot.yaml `camera` 장착값을 쓰므로, 카메라를 그 높이·각도로 들고(고정해) 시험해야 거리가 맞는다.

- `sim/fake_robot.py` 가 로봇 역할: `rt/arm_sdk` 를 받아 관절을 움직이고 `rt/lowstate` 를 낸다 (기구학만, 물리·균형 없음).
- `sim/sim_server.py` 가 detect_box(50010) 자리를 대신: 가상 박스를 robot.yaml `camera` 장착값으로 카메라 좌표로 바꿔
  `/pose` 로 준다 → **robot_server / arm_server 는 실기와 같은 코드 그대로** 잡기 시퀀스를 돈다 (허리 yaw 정렬·재검출 포함).
- 화면에 표시: 잡기 단계, 손 목표 오차(IK 도달), 손 높이 − 박스 옆면/윗면, 허리 각, 카메라 시야 안/밖.
- **안전**: `ROBOT_SIM=1` → DDS 도메인 1. 실기(도메인 0)와 섞이지 않고, fake_robot/sim_server 는 시뮬 모드가 아니면 실행을 거부.
  실기 스택이 떠 있으면(포트 사용 중) 아무것도 죽이지 않고 시작을 거부한다.
- `enabled: false` 로봇(H2)도 시뮬에서는 실행된다. 실기 스크립트(`start_robot.sh` 등)는 계속 거부.

로그는 `logs/<name>_<date>.log` 에 타임스탬프와 함께 기록됩니다. 종료는 `Ctrl+C`.

## 일반 시나리오

1. (외부 SLAM, 선택) 테이블 근처까지 이동 후 **이동 명령을 멈추고** 넘겨받습니다.
2. `robot_server` 가 바닥 ArUco 마커를 추종 — 마커 법선 위 경유점을 거쳐 항상 **마커 정면**으로 도착 (0.25 m 정지, 좌우 4 cm 이내).
3. 박스 잡기 (Box 모드), **hold** (weight = 1, 허리 pitch −3° 보정), 들고 걷기.
4. **release** 하면 팔/허리를 보행 제어기로 돌려 자연스러운 팔 흔들기 보행.

## 디렉터리

```
g1-motion-control/
├── robot_env.sh            # ROBOT 검사 + conda/tv python 경로 (스크립트 공용)
├── activate_tv.sh          # tv conda 환경 활성화 (계정명 하드코딩 없음)
├── start_fsm.sh            # 자세 전환 (stand / sit / bal / no-bal / damp)
├── start_robot.sh          # 전체 스택 (camera, detect, arm, robot, dashboard)
├── start_simulator.sh      # 모션 에디터 (+ arm_server 없으면 같이 기동)
├── start_mission.sh        # 미션 스택 (marker_nav → mission_server). robot_server 와 동시 사용 금지
├── start_sim.sh            # 시뮬레이터 (fake_robot + 서버 + 가상 박스, DDS 도메인 1)
├── sim/                    # fake_robot.py (가짜 로봇), sim_server.py (가짜 detect_box + 시뮬 화면)
├── launcher.sh / run_launcher.py   # 웹 런처 (:80)
├── common/                 # 로봇 공통 코드
│   ├── robot_env.py        # ROBOT 선택, robots/<ROBOT>/ 경로
│   ├── robot_server.py     # 오케스트레이터 + 웹 UI (robot_web.html)
│   ├── arm_server.py       # 팔/허리 HTTP 서버 (arm_sdk 소유)
│   ├── simulator.py        # 모션 에디터 백엔드 (simulator.html)
│   ├── dashboard.py        # 3D URDF 뷰어
│   ├── rs_stream.py        # 카메라 서버
│   ├── ctrl/               # 래퍼, IK, detect_marker/box, 손, TTS
│   ├── models/             # 박스 인식 모델 (OpenVINO / PyTorch)
│   └── assets/vendor/      # three.min.js
├── robots/
│   └── g1/                 # G1 전용: URDF, meshes/, motions/, IK 모델 캐시
├── utils/                  # init_fsm, IMU/식별 확인 도구
└── low/                    # G1 저수준 테스트 (rt/lowcmd 송신 — 일반 운용에서 사용 금지)
```

## 로봇별 설정 — `robots/<ROBOT>/robot.yaml`

로봇마다 다른 값은 모두 여기서 읽습니다 (`common/robot_env.py` → `CFG`).

| 항목 | 키 |
| --- | --- |
| 관절 맵(URDF 이름→슬롯), 팔/허리/헤드 슬롯, weight 슬롯, 이득 | `joints`, `gains` |
| FSM ID / 전이 규칙 / launcher 문구 | `fsm` |
| IK 잠금 관절, 손끝(L_ee/R_ee) | `ik` |
| 기본 팔 자세 | `default_arm_deg` |
| pelvis→torso, 잡기 z 오프셋 | `frames`, `grab` |
| **D435i 장착 위치** | `camera` (x, y, z [m, torso_link 기준], pitch_deg [아래로 숙인 각]) |
| 연결 로봇 확인 | `identity.mode_machine` (`utils/check_robot_id.py` 로 측정) |
| 실행 허용 | `enabled` (false 면 모든 스크립트/서버가 거부 — 시뮬 제외) |
| 카메라→IK 좌표 변환 | `frames.exact_ik_frame` (G1 false: 기존대로 / H2 true: pelvis 기준 정확 변환) |
| 허리 사용 / 보행 | `grab.waist_base_pitch_deg`, `grab.waist_locked`, `features.locomotion` |

- **G1**: `.red` 731075b 상수 그대로 — `python utils/check_g1_equiv.py <red> .` 로 원본과 동일함을 확인 (상수, LowCmd 35 슬롯, launcher HTML, FSM 규칙).
- **H2**: `enabled: false`. 근거·확인 필요 항목은 [`robots/h2/FACTS.md`](./robots/h2/FACTS.md).

### D435i 를 실제로 장착한 뒤

`robots/<ROBOT>/robot.yaml` 의 `camera:` **4개 값만** 실측값으로 고칩니다. 코드 수정 없음.

```yaml
camera:
  x: 0.10          # torso_link 원점 → 카메라 (앞 +) [m]
  y: 0.0           # (왼쪽 +) [m]
  z: 0.35          # (위 +) [m]
  pitch_deg: 50.0  # 아래로 숙인 각 [deg] — 박스 윗면 기울기 보정(중력 방향)에도 같이 쓰임
```
