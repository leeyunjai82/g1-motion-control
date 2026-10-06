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

# 3. 모션 에디터 (단독, 또는 start_robot.sh 와 함께)
./start_simulator.sh
#   → 에디터  : http://<pc-ip>:8000/

# (선택) 웹 런처 — FSM 버튼 + start_robot.sh 실행 (포트 80, sudo)
./launcher.sh
```

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

## G1 전용 값 (3단계에서 `robots/g1/robot.yaml` 로 분리 예정)

아직 코드에 상수로 남아 있습니다. **값은 기존과 동일**합니다.

| 항목 | 위치 |
| --- | --- |
| 관절 수/순서, weight 슬롯(29), `unitree_hg` | `common/ctrl/robot_arm.py`, `common/ctrl/arm_controller_wrapper.py` |
| FSM ID / 전이 규칙 (1, 4, 501, 3) | `run_launcher.py` (`FSM_NAME`, `allowed()`), `utils/init_fsm.py` |
| `PELVIS_TO_TORSO`, `GRAB_Z_OFFSET`, `DEFAULT_ARM_DEG` | `common/robot_server.py` (`DEFAULT_ARM_DEG` 는 `arm_server.py` 에도 사본) |
| 카메라 장착 (`CAMERA_X/Y/Z`, `CAM_TILT_DEG = 47.6`) | `robot_server.py`, `mission_server.py`, `ctrl/detect_box.py`, `ctrl/detect_box_conv.py`, `ctrl/detect_marker.py` |
| IK 체인 (잠금 관절, `L_ee`/`R_ee`) | `common/ctrl/robot_arm_ik.py` |
