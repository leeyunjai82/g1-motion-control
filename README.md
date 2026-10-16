# G1 Motion Control

**Ubuntu 24.04 + 외장 Intel mini PC + Intel RealSense D435i** 환경에서 Unitree G1(29 DOF)을 제어하는 통합 패키지입니다.
마커 기반 접근, 박스 파지, 들고 걷기, 웹 모션 에디터를 기능별 작은 HTTP 서버로 나눠 구성합니다.

- 설치 : [**INSTALL.md**](./INSTALL.md) · 가상환경 다시 만들기 / 스냅샷 : [**REBUILD_ENV.md**](./REBUILD_ENV.md)
- 내부 구조 (관절 맵, 모션 JSON, IK, DDS, FSM) : [**TECH.md**](./TECH.md)
- 이 저장소는 **G1 전용**입니다. `ROBOT` 은 안 줘도 `g1` 이고, 다른 값은 거부합니다. H2 는 `h2-motion-control`.

> **실기 확인 필요** — 이 저장소의 G1 코드는 Ubuntu 24.04 이전 뒤 아직 G1 실기에서 돌려 보지 않았습니다.
> 처음 실행할 때는 아래 안전 수칙대로 로봇을 지지한 상태에서 `start_fsm.sh` → `start_robot.sh` 순으로 하나씩 확인하세요.

## 출처

| 부분 | 출처 |
| --- | --- |
| G1 코드 · 설정 · 모션 | `h2-motion-control` `00844a1` (G1 + Ubuntu 24.04 정리가 같이 있는 마지막 커밋) — G1 은 `g1-motion-control.red` `731075b` 기준 |
| `.red` 2026-10-12 변경 | `0239bdd` 박스 모델 box11s-seg2 · seg 마스크 수정, `0d3c343` 카메라 848×480 · K 를 카메라에서, `9d008de` 잡기 접근 폭 0.15 · 박스 잡기 2 cm 몸쪽 · 웹 색상, `eeff3c8`·`11476fc` 웹 이름 Pion — `.red` 마지막 `11476fc` 와 같은 동작 |
| 가상환경 스냅샷 · 재구성 문서 | `h2-motion-control` `592d235` (`utils/env_snapshot.sh`, `REBUILD_ENV.md`) |
| 웹 UI `Cache-Control: no-cache` | `h2-motion-control` `95bc553` (+ 다른 웹 서버에도 같이 적용) |
| sudo 로 만든 파일의 소유자 유지 | `h2-motion-control` `7b4e607` 의 chown 부분 |
| unitree_sdk2py | `requirements.txt` 고정 커밋 `f559291` 그대로 (third_party SDK 없음) |

빠진 것: `robots/h2`, H2 머리 카메라(`head_track` · GStreamer), H2 시험 도구, `physical_ai/` · `trainer/`.

---

## ⚠️ 안전 수칙 (먼저 읽기)

1. **스탠드 고정** — 지지 스탠드를 로봇 어깨에 단단히 묶어 흔들리지 않게 합니다.
2. **`./start_fsm.sh stand`** 는 즉시 모터에 힘이 들어갑니다. **사람이 로봇을 붙잡고** 실행하세요.
3. **`./start_fsm.sh sit`** 는 천천히 주저앉습니다. 팔을 먼저 몸 옆으로 내리고 **내려가는 동안 계속 지지**하세요.
4. 작업 공간에 사람·장애물이 없게 하고, **비상정지(E-STOP)** 위치를 미리 확인하세요.
5. 웹 버튼(launcher 포함)은 비상정지가 아닙니다. 물리 E-STOP / 리모컨을 항상 손에 두세요.

---

## 구성

자원(하드웨어)마다 소유 프로세스는 하나. 나머지는 HTTP 로 통신합니다.

```
rs_stream      :50001   RealSense 카메라 (단독 점유, 기본 848×480) → MJPEG / depth API / GET /intrinsics (카메라 K)
detect_marker  :50011   ArUco 마커 자세 (rs_stream 사용, 시작 시 K 를 rs_stream 에서 읽음)
detect_box     :50010   박스 인식(box11s-seg2), 파지점 (rs_stream 사용, OpenVINO intel:cpu, 시작 시 K 를 rs_stream 에서 읽음)
arm_server     :50022   rt/arm_sdk 단독 점유 — 팔/허리, IK, hold/release
robot_server   :50000   오케스트레이터 — 잡기 시퀀스, 마커 추종, 웹 UI
dashboard      :50003   3D URDF 뷰어 / 관절 상태 (rt/lowstate 읽기 전용)
simulator      :8000    모션 에디터 (Joint + IK), arm_server 경유
launcher       :80      FSM 버튼 + start_robot.sh 실행 웹 (root)
```

- 웹 이름: Pion·manipulation (:50000) / Pion·launcher (:80) / Pion motion editor (:8000).
- 카메라 해상도는 `RS_W` / `RS_H` 환경변수로 바꿀 수 있습니다 (기본 848×480). 인식 서버는 시작할 때 읽은 K 의 해상도와 다른 프레임은 쓰지 않으므로,
  해상도를 바꾸면 rs_stream 과 인식 서버를 같이 다시 시작하세요 (`start_robot.sh` 를 다시 실행).

규칙:
- **팔/허리**: 모든 프로세스는 `arm_server` 를 거칩니다 (`rt/arm_sdk` 는 publisher 하나만 허용).
- **보행**: `LocoClient` 는 다중 클라이언트 RPC — 직접 써도 되지만 이동 명령은 한 곳에서만 보냅니다.

## 빠른 시작

```bash
# 1. 자세 (로봇을 붙잡고!) — stand = 1(Damping) → 5초 → 4(Lock Standing) → 10초 → 501(Walk 3DoF waist)
./start_fsm.sh stand        # 또는: sit / bal / no-bal / damp

# 2. 전체 스택 (서버 6개, 의존 순서대로)
./start_robot.sh
#   → 제어 UI : http://<pc-ip>:50000/
#   → 3D 뷰어 : http://<pc-ip>:50003/dashboard

# 3. 모션 에디터 — 모드 지정 필수 (virtual | real)
./start_simulator.sh real          # 실기 (start_robot.sh 와 함께 써도 됨 — arm_server 재사용)
#   → 에디터  : http://<pc-ip>:8000/        모터 번호 확인 : http://<pc-ip>:8000/check

# (선택) 웹 런처 — FSM 버튼 + start_robot.sh 실행 (포트 80, 시작할 때 sudo 비밀번호 한 번)
./launcher.sh
```

Wi-Fi 와 유선이 같이 켜진 PC 에서 로봇 데이터(rt/lowstate)가 안 들어오면 DDS 인터페이스를 지정합니다:

```bash
ip -br link                                 # 로봇 쪽 유선 인터페이스 이름 확인
DDS_IFACE=enp3s0 ./start_fsm.sh stand       # 모든 start_*.sh / launcher.sh 가 같은 변수 사용
export DDS_IFACE=enp3s0                     # 매번 쓰기 싫으면 ~/.profile 에
```

- 비우면 자동 선택 (기존과 같음). 없는 이름이면 실행을 거부합니다.
- sudo 로 도는 `init_fsm.py` / `run_launcher.py` 에는 스크립트가 인자(`iface=<이름>`)로 넘깁니다 (sudo 가 환경변수를 지우므로).

## 실행 스크립트

| 스크립트 | 하는 일 |
| --- | --- |
| `start_fsm.sh <stand\|sit\|bal\|no-bal\|damp>` | 자세 전환 (`sudo` 로 tv python → `utils/init_fsm.py`) |
| `start_robot.sh` | rs_stream → arm_server → robot_server → dashboard → detect_marker → detect_box |
| `start_simulator.sh <virtual\|real>` | 모션 에디터 (+ arm_server · dashboard 없으면 같이) |
| `start_mission.sh` | 미션 스택 (rs_stream · detect_box · arm_server · mission_server · marker_nav). `robot_server` 와 동시 사용 금지 |
| `start_sim.sh [real-cam]` | 로봇 없이 잡기 시퀀스 시험 (fake_robot, DDS 도메인 1) |
| `launcher.sh` | 웹 런처 (:80) — FSM 단계별 버튼 + `start_robot.sh` 시작/정지 |

공통 동작:
- 시작할 때 이전에 남은 서버를 **실행 형태 패턴**(`^[^ ]*python[^ ]* (-u )?[^ ]*arm_server\.py` 등)으로 찾아
  **TERM → 대기 → KILL** 순서로 정리합니다 (바로 KILL 하면 arm_server 가 팔 제어권(weight)을 반납하지 못함).
- 서버는 `setpriv --pdeathsig KILL` 로 띄웁니다 — 스크립트가 강제 종료돼도 서버가 고아로 남지 않습니다.
  정상 종료(`Ctrl+C`)는 TERM → 대기 → KILL 순서 그대로입니다.
- 로그: `logs/<이름>_<날짜>.log` (줄마다 타임스탬프, mawk 는 `-W interactive` 로 줄 단위 즉시 기록).

## Motion Editor 모드 (`start_simulator.sh`)

| 명령 | 모드 | 띄우는 것 |
| --- | --- | --- |
| `./start_simulator.sh virtual` | **가상** — URDF/메시 3D + fake_robot, 로봇 없이 (DDS 도메인 1) | fake_robot, arm_server, dashboard, simulator |
| `./start_simulator.sh real` | **실기** — 실제 로봇이 움직임 | arm_server·dashboard(떠 있으면 재사용), simulator |

### 모터 번호 확인 (`http://<pc-ip>:8000/check`)

1. 로봇을 거치대에 걸고 주변을 비운다. E-STOP 을 손에 둔다.
2. `./start_simulator.sh real` — 상단에 `mode_machine` 이 표시된다 (robot.yaml `identity` 기준값).
3. **팔·허리** (명령 이동): [기준 잡기] → 슬롯의 [+]/[−] (한 번 5°, 기준 대비 최대 ±15°, 서버에서 제한).
   실제 로봇에서 움직인 관절과 오른쪽 3D 에서 움직인 관절이 같으면 ✓, 다르면 ✗ + 실제 관절을 메모.
4. **다리** (읽기 전용): [제어권 반납] (또는 FSM Damp) → [기준 잡기] → 관절을 손으로 움직이면 변한 슬롯이 노랗게 표시.
5. **IK**: [제어권 잡기] → [IK 기준 잡기] → 위/아래·앞/뒤·좌/우·벌림/좁힘 (3 cm 씩, 기준 대비 최대 10 cm, 손목 자세 유지).
6. [결과 저장] → `robots/g1/joint_check_<날짜시각>.json` (sudo 로 띄웠어도 소유자는 원래 사용자).
7. [제어권 반납] → simulator Ctrl+C.

가상 모드에서 같은 화면으로 절차를 미리 연습할 수 있습니다 (가상은 robot.yaml 대로 움직이므로 항상 ✓).

## 시뮬레이터 (로봇 없이 시험)

```bash
./start_sim.sh            # 가상 카메라 + 가상 박스  → http://<pc-ip>:50010/
./start_sim.sh real-cam   # 실물 D435i + 실제 박스 인식(YOLO), 로봇만 가상 → http://<pc-ip>:50012/
```

- `sim/fake_robot.py` 가 로봇 역할: `rt/arm_sdk` 를 받아 관절을 움직이고 `rt/lowstate` 를 냅니다 (기구학만, 물리·균형 없음).
- `sim/sim_server.py` 가 detect_box(50010) 자리를 대신 — **robot_server / arm_server 는 실기와 같은 코드 그대로** 잡기 시퀀스를 돕니다.
- **안전**: `ROBOT_SIM=1` → DDS 도메인 1. 실기(도메인 0)와 섞이지 않습니다. `DDS_IFACE` 는 시뮬에 적용하지 않습니다.
  실기 스택이 떠 있으면(포트 사용 중) 아무것도 죽이지 않고 시작을 거부합니다.

## 일반 시나리오

1. (외부 SLAM, 선택) 테이블 근처까지 이동 후 **이동 명령을 멈추고** 넘겨받습니다.
2. `robot_server` 가 바닥 ArUco 마커를 추종 — 마커 법선 위 경유점을 거쳐 항상 **마커 정면**으로 도착 (0.25 m 정지, 좌우 4 cm 이내).
3. 박스 잡기 (Box 모드), **hold** (weight = 1, 허리 pitch −3° 보정), 들고 걷기.
4. **release** 하면 팔/허리를 보행 제어기로 돌려 자연스러운 팔 흔들기 보행.

## 디렉터리

```
g1-motion-control/
├── robot_env.sh            # ROBOT(g1) 검사, conda/tv python 경로, stamp, DDS_IFACE 검사, with_pdeath (스크립트 공용)
├── activate_tv.sh          # tv conda 환경 활성화 (계정명 하드코딩 없음)
├── start_fsm.sh            # 자세 전환 (stand / sit / bal / no-bal / damp)
├── start_robot.sh          # 전체 스택 (camera, detect, arm, robot, dashboard)
├── start_simulator.sh      # 모션 에디터 (+ arm_server 없으면 같이 기동)
├── start_mission.sh        # 미션 스택 (marker_nav → mission_server). robot_server 와 동시 사용 금지
├── start_sim.sh            # 시뮬레이터 (fake_robot + 서버 + 가상 박스, DDS 도메인 1)
├── launcher.sh / run_launcher.py   # 웹 런처 (:80)
├── sim/                    # fake_robot.py (가짜 로봇), sim_server.py (가짜 detect_box + 시뮬 화면)
├── common/
│   ├── robot_env.py        # robots/g1/robot.yaml 읽기, dds_init (DDS_IFACE), own_by_sudo_user
│   ├── robot_server.py     # 오케스트레이터 + 웹 UI (robot_web.html)
│   ├── arm_server.py       # 팔/허리 HTTP 서버 (arm_sdk 소유)
│   ├── simulator.py        # 모션 에디터 백엔드 (simulator.html, joint_check.py)
│   ├── dashboard.py        # 3D URDF 뷰어
│   ├── rs_stream.py        # 카메라 서버
│   ├── marker_nav.py · mission_server.py · pilot_server.py   # 미션 스택
│   ├── ctrl/               # 래퍼, IK, detect_marker/box, 손, TTS, hw_usage(CPU/GPU/NPU 사용률)
│   ├── models/             # 박스 인식 모델 (OpenVINO / PyTorch)
│   └── assets/vendor/      # three.min.js
├── robots/g1/              # robot.yaml, URDF, meshes/, motions/, IK 모델 캐시(*.pkl, git 제외)
├── utils/                  # init_fsm, IMU/식별/카메라 확인, get_dev, env_snapshot.sh
└── low/                    # G1 저수준 테스트 (rt/lowcmd 송신 — 일반 운용에서 사용 금지)
```

## 로봇 설정 — `robots/g1/robot.yaml`

G1 값은 모두 여기서 읽습니다 (`common/robot_env.py` → `CFG`). `.red` 11476fc 와 같은 값입니다
(`python utils/check_g1_equiv.py <red 경로> .` 로 원본과 같은지 확인 — 상수, LowCmd 35 슬롯, launcher HTML, FSM 규칙,
박스 모델 바이트, 848×480·K 로드, 잡기 접근 폭·박스 2 cm. 사용법은 스크립트 머리말).

| 항목 | 키 |
| --- | --- |
| 관절 맵(URDF 이름→슬롯), 팔/허리 슬롯, weight 슬롯, 이득 | `joints`, `gains` |
| FSM ID / 전이 규칙 / launcher 문구 | `fsm` |
| IK 잠금 관절, 손끝(L_ee/R_ee) | `ik` |
| 기본 팔 자세 | `default_arm_deg` |
| pelvis→torso, 잡기 z 오프셋, 접근 폭(`approach_extra` 0.15), 박스 잡기 x 보정(`box_grab_x_extra` −0.02) | `frames`, `grab` |
| **D435i 장착 위치** | `camera` (x, y, z [m, torso_link 기준], pitch_deg [아래로 숙인 각]) |
| 카메라 K | 실기는 rs_stream `/intrinsics` (카메라에서 읽음). `camera.intrinsics` (640×480) 는 시뮬 가상 카메라 전용 |
| 연결 로봇 확인 | `identity.mode_machine` (`utils/check_robot_id.py` 로 측정) |
| 실행 허용 | `enabled` (false 면 모든 스크립트/서버가 거부 — 시뮬 제외) |
| 보행 | `features.locomotion` (G1 true) |

### D435i 를 다시 장착한 뒤

`robots/g1/robot.yaml` 의 `camera:` **4개 값만** 실측값으로 고칩니다. 코드 수정 없음.
숙임각은 `python utils/check_rsimu.py --sec 5` (D435i IMU − 로봇 IMU) 로 잴 수 있습니다 (IMU 권한 오류면 sudo 또는 RealSense udev 규칙).
