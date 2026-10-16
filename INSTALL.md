# 설치 가이드 — G1 Motion Control · Ubuntu 24.04 LTS

외장 Intel mini PC(x86_64)에 **Ubuntu 24.04 LTS** 를 기준으로 설치하는 절차입니다. 순서대로 진행하세요.

- 사용법 : [`README.md`](./README.md)
- 가상환경(tv) 다시 만들기 / 스냅샷 : [`REBUILD_ENV.md`](./REBUILD_ENV.md)
- 내부 구조 : [`TECH.md`](./TECH.md)

> **확인 필요** 표시는 이 문서 작성 시점에 G1 운영 PC / G1 실기에서 검증하지 못한 항목입니다.
> 절차는 같은 requirements 로 Ubuntu 24.04 운영 PC 에 설치해 쓰고 있는 `h2-motion-control` 실측을 따릅니다.
> 설치하면서 결과를 기록하고, 잘 되면 `./utils/env_snapshot.sh` 결과를 커밋해 주세요 (16단계).

---

## 1. 시스템 요구사항

| 항목 | 권장 |
| --- | --- |
| OS | **Ubuntu 24.04 LTS** (x86_64) |
| Python | 3.10 (Miniconda `tv` 환경) — G1 운영 PC 의 기존 tv 버전은 확인 필요 (`pip freeze` 로 맞춤) |
| 메모리 / 디스크 | 8 GB 이상 / 10 GB 이상 여유 |
| 네트워크 | 로봇과 같은 LAN `192.168.123.0/24` (유선 권장) |
| 카메라 | Intel RealSense D435i (USB 3.0 포트) |
| 권한 | sudo 가능한 계정 |

설치 경로 예: `$HOME/project/g1-motion-control/`.
스크립트는 계정명·절대경로를 하드코딩하지 않습니다 (`$HOME`, 스크립트 위치 기준).

---

## 2. OS 기본 패키지 (apt — 최소)

Python 쪽은 전부 `tv` 가상환경에 넣고, 시스템에는 꼭 필요한 것만 설치합니다.

```bash
sudo apt update
sudo apt install -y wget curl
```

- 빌드 도구(build-essential, libssl-dev, libsuitesparse-dev 등)는 **필요 없습니다** — cyclonedds 는 pip 바이너리 휠,
  scikit-sparse 는 conda-forge 패키지로 설치합니다.
- `curl` : `start_sim.sh` / `start_simulator.sh` 가 서버 응답 확인에 사용 (가상환경 활성화 전에 호출).
- 서버판 Ubuntu 에서 OpenCV 가 `libGL.so.1` 을 찾지 못하면: `sudo apt install -y libgl1 libglib2.0-0`
- 로그 타임스탬프(`awk strftime`): 24.04 기본 `mawk 1.3.4 20240123` 는 파이프 입력을 블록 단위로 모아 읽어
  로그가 늦게 써집니다 → `robot_env.sh` 의 `stamp()` 가 mawk 면 `awk -W interactive` 로 줄 단위 처리 (gawk 불필요).
- 서버 자식 프로세스는 `setpriv --pdeathsig KILL` 로 띄웁니다 — `setpriv` 는 util-linux (24.04 기본 2.39) 에 포함.

---

## 3. Intel RealSense (D435i) — pip `pyrealsense2` 만

코드는 Python `pyrealsense2` 만 씁니다. `requirements.txt` 의 `pyrealsense2==2.55.1.6486` 이 7단계에서 **`tv` 가상환경 안에 설치**되며
(PyPI 바이너리 휠, librealsense 라이브러리 포함), **apt 로 RealSense SDK 를 깔 필요는 없습니다.**

카메라는 **허브 없이 USB 3.0(SS, 파란) 포트**에 연결하고, 7단계 후 확인:

```bash
source $HOME/miniconda3/bin/activate tv
python -c "import pyrealsense2 as rs; d=rs.context().devices; print([x.get_info(rs.camera_info.name)+' '+x.get_info(rs.camera_info.usb_type_descriptor) for x in d])"
# ['Intel RealSense D435I 3.2'] 처럼 나오면 정상 (3.x = USB3)
conda deactivate
```

**권한 오류일 때만** — 같은 버전(v2.55.1)의 udev 규칙 파일을 시스템에 넣습니다:

```bash
sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.55.1/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# 카메라 USB 를 뺐다 다시 꽂기
```

- **`librealsense2-dkms` 는 설치하지 마세요** — 공식 지원 커널 목록에 24.04 커널(6.8 이상)이 없습니다.
- `2.1` (USB2) 로 잡히면 **케이블 교체** (USB2 전용 케이블 — H2 운영 PC 에서 케이블 교체로 3.2 확인), 커넥터 끝까지, 허브 없이 SS 포트.
- D435i **IMU(HID/iio)** 는 udev 규칙이 없으면 일반 계정으로 못 읽습니다 → `utils/check_rsimu.py` 등은 `sudo` 로 실행하거나 위 규칙 설치.

---

## 4. Miniconda — `.bashrc` 를 건드리지 않고 설치

로봇을 돌리는 계정 하나에만 설치합니다. **`conda init` 을 하지 않으므로** 다른 계정과 이 계정의 평소 셸에는 영향이 없습니다.

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3     # -b: 묻지 않음, .bashrc 수정 없음
rm Miniconda3-latest-Linux-x86_64.sh
$HOME/miniconda3/bin/conda --version
```

- 스크립트는 conda 를 `$CONDA_BASE` → `~/miniconda3` → `~/anaconda3` → `~/miniforge3` → `/opt/conda` 순으로 찾습니다.
  다른 위치에 설치했다면 `export CONDA_BASE=<경로>`.
- `tv` 활성화는 항상 **환경 이름을 명시**합니다 (`source $CONDA_BASE/bin/activate tv`). 이름을 빼면 호출한 스크립트의 `$1`
  (예: `real-cam`)이 환경 이름으로 넘어가 `EnvironmentNameNotFound: real-cam` 이 납니다 (`activate_tv.sh` 가 처리).

---

## 5. `tv` 가상환경 생성 (conda-forge 만 사용)

```bash
source $HOME/miniconda3/bin/activate          # 이 터미널에서만 conda 사용
conda create -y -n tv --override-channels -c conda-forge \
    python=3.10 pinocchio=3.1.0 numpy=1.24.4 scikit-sparse=0.4.16 git
conda activate tv
```

- conda 26.x 는 기본 채널을 쓰면 `CondaToSNonInteractiveError` (약관 승인) 로 멈춥니다 → 생성·삭제 모두 `--override-channels -c conda-forge`.
  지울 때도 `conda env remove` 는 같은 오류 → `conda remove -y --all -n tv --override-channels -c conda-forge` ([REBUILD_ENV.md](./REBUILD_ENV.md)).
- `pinocchio=3.1.0` : **pip 의 `pin` 패키지는 `pinocchio.casadi` 가 없어 쓰면 안 됩니다.**
- `numpy=1.24.4` : requirements.txt 와 같은 값. **numpy 2.x 금지** (OpenCV 4.10).
- `scikit-sparse` : conda-forge 바이너리 → 시스템 `libsuitesparse-dev` 불필요.
- `git` : 가상환경 안의 git (unitree_sdk2py 를 GitHub 에서 받을 때 사용).
- 환경 이름은 `tv` 고정 — `robot_env.sh` 의 `TV_PY` 와 sudoers 경로가 `envs/tv` 기준입니다.

---

## 6. 프로젝트 받기

```bash
mkdir -p $HOME/project && cd $HOME/project
git clone https://github.com/leeyunjai82/g1-motion-control.git      # tv 활성화 상태 → 가상환경의 git
cd g1-motion-control
```

---

## 7. Python 패키지 (전부 `tv` 안, **한 줄로**)

```bash
# (tv 활성화 상태, 프로젝트 폴더에서)
pip install --upgrade pip
pip install -r requirements.txt \
    torch==2.4.1+cpu torchvision==0.19.1+cpu \
    meshcat==0.3.2 logging-mp openvino \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    --src "$CONDA_PREFIX/src"
```

- **torch 와 나머지를 한 번에** 설치합니다. torch 만 먼저 깔면 torchvision 이 numpy 2.2.6 을 끌어옵니다 (H2 실측).
- PyTorch 는 CPU 판(`+cpu`). 버전 없이 / CUDA 판으로 깔면 nvidia-* 패키지 수 GB 가 같이 들어옵니다.
- `cyclonedds==0.10.2` 는 pip 바이너리 휠 — **Python 3.10 이 아니면** 소스 빌드로 넘어가 실패합니다 (`python -V` 확인).
- `unitree_sdk2py` 는 requirements 의 고정 커밋 `f559291` 을 가상환경 안(`$CONDA_PREFIX/src`)에 받아 editable 설치.
- `openvino`, `logging-mp` 는 버전을 고정하지 않았습니다 — 기존 G1 운영 PC 의 `pip freeze` 값과 맞추는 것을 권장 (확인 필요).
- 이후 **`pip install -U <패키지>` 금지** — 의존성이 같이 올라가 numpy 2.x 가 들어올 수 있습니다.
- 확인 필요: 위 한 줄 명령은 G1 PC 에서 아직 실행해 보지 않았습니다 (H2 의 REBUILD_ENV 3-A 와 같은 방식).

확인:
```bash
python -c "import numpy, torch, cv2, pinocchio, sksparse, unitree_sdk2py, openvino; from pinocchio import casadi; print('numpy', numpy.__version__, '| torch', torch.__version__)"
# numpy 1.24.4 | torch 2.4.1+cpu 가 나와야 정상
```

### 평소 사용 (conda 는 필요할 때만)

- `start_*.sh` / `launcher.sh` 는 스스로 `tv` 를 찾아 씁니다 → **그냥 실행하면 됩니다.**
- 터미널에서 직접 python 을 쓸 때만:
  ```bash
  source activate_tv.sh          # 프로젝트 폴더에서 (또는 source $HOME/miniconda3/bin/activate tv)
  ...
  conda deactivate
  ```

---

## 8. 성능: BLAS 스레드 1개 고정 (중요)

양팔 IK(pinocchio + CasADi)는 작은 선형계를 풉니다. 멀티스레드 OpenBLAS 에서는 동기화 오버헤드로 IK 1회가 ~1.5 ms → ~40 ms 로 느려집니다.

```bash
mkdir -p "$CONDA_PREFIX/etc/conda/activate.d"
cat > "$CONDA_PREFIX/etc/conda/activate.d/threads.sh" << 'EOF'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
EOF
conda deactivate && conda activate tv && echo $OMP_NUM_THREADS   # 1
```

> start 스크립트는 `activate_tv.sh` 로 tv 를 활성화하므로 이 훅이 적용됩니다.

---

## 9. sudo 설정 (`start_fsm.sh`)

`start_fsm.sh` 는 tv 환경 python 을 `sudo` 로 실행합니다. 비밀번호 없이 쓰려면 NOPASSWD 규칙을 추가합니다.

```bash
sudo visudo -f /etc/sudoers.d/g1-motion
```
```
<사용자> ALL=(root) NOPASSWD: /home/<사용자>/miniconda3/envs/tv/bin/python
```

- 경로는 스크립트가 실제 실행하는 python 절대경로와 **정확히 같아야** 합니다. 확인:
  ```bash
  source robot_env.sh && echo "$TV_PY"
  sudo -n "$TV_PY" -c "print('ok')"     # 비밀번호를 묻지 않아야 정상
  ```
- sudo 는 환경변수를 지웁니다. `ROBOT`, `DDS_IFACE` 는 sudo 뒤에서 **명령 인자**로 넘깁니다
  (`init_fsm.py <mode> g1 [iface=<이름>]`, `run_launcher.py g1 [iface=<이름>]`). **sudoers 에 `VAR=` · `SETENV` 를 쓰지 않습니다.**
- `launcher.sh` (포트 80 = root 필요) 는 시작할 때 `sudo -v` 로 비밀번호를 한 번 묻습니다 (NOPASSWD 불필요).
  시작 전에 `ss -ltn` 으로 80 포트 점유를 확인하고, 다른 프로그램(nginx/apache 등)이 쓰면 그 프로세스를 보여주고 멈춥니다.
- root 로 실행되는 동안 만든 파일은 원래 사용자 소유로 돌립니다: launcher 로그(`SUDO_USER`), IK 캐시·모터 번호 확인 결과(`SUDO_UID/SUDO_GID`).
  launcher 는 `start_robot.sh` 를 `SUDO_USER` 로 내려서 실행합니다 (logs/ 가 root 소유가 되면 수동 실행 때 쓰기 실패).
- launcher 를 끌 때 TERM 은 python 에만 보냅니다 (sudo 부모까지 받으면 uvicorn 이 신호를 두 번 받아 정리를 건너뜀) — `launcher.sh` 가 처리.

---

## 10. 네트워크

로봇 대역 `192.168.123.0/24`. PC 유선 LAN 을 같은 대역의 고정 IP 로 설정합니다.

```bash
nmcli connection show                     # 유선 연결 이름 확인
sudo nmcli connection modify "<연결이름>" ipv4.addresses 192.168.123.222/24 ipv4.method manual
sudo nmcli connection up "<연결이름>"
ping 192.168.123.161
```

- `192.168.123.222` 는 예시입니다 — 로봇·다른 장비와 겹치지 않는 주소로.
- G1 쪽 주소: `.red` 문서는 `192.168.123.161` 기준입니다. **확인 필요** — G1 내부 PC 주소를 Unitree 문서/실기로 확인해 기록해 주세요.
- 방화벽: firewalld 대신 Ubuntu 기본 **`ufw`** (설치 직후 비활성, `sudo ufw status`). 켜져 있으면 로봇 대역과 웹 포트를 허용:
  ```bash
  sudo ufw allow from 192.168.123.0/24     # 로봇 LAN (DDS)
  sudo ufw allow 80/tcp                    # launcher (다른 PC 에서 접속할 때)
  sudo ufw allow 50000:50050/tcp           # 웹 UI / 서버
  sudo ufw allow 8000/tcp                  # simulator
  ```
- **DDS 인터페이스**: 서버들은 기본으로 인터페이스를 자동 선택합니다(`ChannelFactoryInitialize(0)`). Wi-Fi 와 유선이 같이 켜진 PC 에서
  로봇 데이터가 안 들어오면 `DDS_IFACE=<유선 인터페이스>` 로 지정하세요 (README "빠른 시작"). 없는 이름이면 실행을 거부합니다.

---

## 11. 손 컨트롤러 시리얼 (`/dev/ttyACM*`, 손을 달았을 때만)

```bash
sudo usermod -aG dialout $USER     # 다시 로그인 후 적용
ls -l /dev/ttyACM0
```

- Ubuntu 의 `ModemManager` 가 새로 꽂힌 ttyACM 장치를 모뎀으로 검사하며 잠시 점유할 수 있습니다.
  손 연결이 간헐적으로 실패하면: `sudo systemctl disable --now ModemManager` (모뎀을 안 쓰는 PC 에서만).

---

## 12. Intel GPU / NPU (OpenVINO)

- 드라이버 패키지 이름·버전은 문서로 정리된 것이 없습니다 — **확인 필요**. 기존 운영 PC 가 있으면 그 PC 의 `env_snapshot/<호스트>/system.txt`
  (커널, `intel-*` / `libze*` apt 목록, NPU 펌웨어)와 맞추고, 아래로 장치가 보이는지 확인합니다:
  ```bash
  source activate_tv.sh
  python utils/get_dev.py | head -3          # Available devices: ['CPU', 'GPU', 'NPU'] 등
  python common/ctrl/hw_usage.py --sec 3     # CPU / GPU / NPU 사용률 (못 읽으면 이유)
  ```
- 사용률은 root 없이 sysfs 로 읽습니다 (`common/ctrl/hw_usage.py`, 제어 화면 위 칩):
  GPU = xe `idle_residency_ms` / i915 `rc6_residency_ms`, NPU = intel_vpu `npu_busy_time_us`.
- G1 박스 인식(`detect_box`)은 `intel:cpu` 입니다 (robot.yaml `vision.box_device` 없음 → 기본값). GPU/NPU 로 바꾸려면
  `vision: {box_device: "intel:gpu"}` 처럼 지정 — **NPU 는 첫 컴파일이 느려 OpenVINO `CACHE_DIR` 이 필요**한데,
  ultralytics 경로에는 아직 CACHE_DIR 을 넘기지 않습니다 (H2 머리 카메라 `ov_detect.py` 에만 있었음). 캐시 경로는 `~/.cache/g1-motion-control/openvino` 로 통일.

---

## 13. 실행 권한

```bash
chmod +x robot_env.sh activate_tv.sh start_fsm.sh start_robot.sh start_simulator.sh start_mission.sh start_sim.sh launcher.sh utils/env_snapshot.sh
```

---

## 14. 캐시 (버전이 바뀔 때만 지움)

| 캐시 | 언제 지움 |
| --- | --- |
| `robots/g1/g1_29_model_cache.pkl` (IK 모델, robot.yaml `ik_cache`, git 제외) | **pinocchio 버전이 바뀔 때** — `pickle.load` 오류(`class version …`)가 나면 반드시. 다음 실행 때 URDF 에서 다시 만듦 |
| `~/.cache/g1-motion-control/openvino` (OpenVINO 컴파일 결과) | OpenVINO 버전이 바뀔 때 |

---

## 15. 설치 검증

```bash
source activate_tv.sh

# (1) 설정 읽기 (ROBOT 은 안 줘도 g1)
python -c "import sys; sys.path.insert(0,'common'); import robot_env as r; print(r.ROBOT, r.URDF_PATH, r.DDS_IFACE or '(자동)')"

# (2) 다른 ROBOT → 거부되어야 정상
ROBOT=h2 python -c "import sys; sys.path.insert(0,'common'); import robot_env"     # ❌ G1 전용 메시지 + 종료코드 2

# (3) 핵심 패키지
python -c "import numpy, cv2, torch, pyrealsense2, pinocchio, casadi, unitree_sdk2py; print('OK')"

# (4) NumPy 1.x + OpenCV 인코딩
python -c "
import numpy as np, cv2
assert np.__version__ < '2', np.__version__
print('imencode', cv2.imencode('.jpg', np.zeros((480,640,3), np.uint8))[0])"

# (5) RealSense
python -c "import pyrealsense2 as rs; print(rs.context().devices[0].get_info(rs.camera_info.name))"

# (6) IK 속도 — 스레드 고정 시 수 ms
cd common && python -c "
import time, numpy as np, pinocchio as pin
from ctrl.robot_arm_ik import G1_29_ArmIK
ik = G1_29_ArmIK(); q=np.zeros(14); dq=np.zeros(14)
L=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3, 0.2,0.1])).homogeneous
R=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3,-0.2,0.1])).homogeneous
ik.solve_ik(L,R,q,dq); t=time.time()
for _ in range(50): ik.solve_ik(L,R,q,dq)
print(f'IK avg: {(time.time()-t)/50*1000:.1f} ms')"; cd ..

# (7) 로봇 없이 잡기 시퀀스 — http://<pc-ip>:50010/ 에서 Grab 이 끝까지 가면 정상
./start_sim.sh

# (8) 로봇 연결 후 (읽기 전용) — 식별값
python utils/check_robot_id.py           # 인터페이스 지정: python utils/check_robot_id.py enp3s0
```

---

## 16. 스냅샷 (잘 될 때)

```bash
./utils/env_snapshot.sh                 # → env_snapshot/<호스트명>/  (root 불필요, 읽기만 함)
git add env_snapshot/ && git commit -m "가상환경 스냅샷 $(hostname) $(date +%F)"
```

conda explicit · pip(torch/일반/git) · activate.d 훅 · `system.txt`(커널·GPU/NPU 드라이버·apt·OpenVINO 장치·RealSense)가 남습니다.
복원은 [REBUILD_ENV.md](./REBUILD_ENV.md) 3-A.

---

## 문제 해결

| 증상 | 원인 / 해결 |
| --- | --- |
| `ROBOT='…' — 이 저장소는 G1 전용` | `unset ROBOT` (안 주면 g1) |
| `conda 를 찾지 못했습니다` | `export CONDA_BASE=<conda 설치 경로>` |
| `EnvironmentNameNotFound: real-cam` 등 | activate 에 환경 이름을 안 준 것 — `source activate_tv.sh` 사용 (4단계) |
| `DDS_IFACE='…' — 이 PC 에 없는 네트워크 인터페이스` | `ip -br link` 로 이름 확인, 비우면 자동 선택 |
| 로봇 데이터(rt/lowstate) 안 들어옴 | 유선 IP 대역(10단계), Wi-Fi 가 같이 켜져 있으면 `DDS_IFACE` 지정, ufw |
| sudo 가 계속 비밀번호 요구 (FSM) | sudoers 경로 ≠ `$TV_PY`. 9단계 확인 |
| launcher: `포트 80 사용 중` | 표시된 프로세스(nginx/apache 등) 중지 |
| `CondaToSNonInteractiveError` | 기본 채널을 쓴 경우 — 5단계처럼 `--override-channels -c conda-forge` |
| `cyclonedds` 빌드 실패 | Python 3.10 휠이 안 받아진 경우 — `python -V` 가 3.10 인지(tv 활성화) 확인 |
| `scikit-sparse`: `cholmod.h` 없음 | pip 로 빌드하려 한 것 — 5단계 conda-forge `scikit-sparse` 로 설치 |
| OpenCV `img is not a numpy array` | NumPy 2.x. `pip install numpy==1.24.4` |
| PyTorch 설치가 수 GB / nvidia-* | CUDA 판 — 7단계처럼 `+cpu` 와 `--extra-index-url …/whl/cpu` |
| IK 가 수십 ms | BLAS 스레드 고정 (8단계) |
| RealSense 인식 안 됨 / 권한 오류 | USB 3.0 포트, 3단계 udev 규칙 파일 설치 후 재연결 |
| RealSense 가 `2.1` (USB2) 로 잡힘 | 케이블 교체, 허브 없이 SS 포트 |
| D435i IMU 권한 오류 | sudo 로 실행 또는 3단계 udev 규칙 |
| detect_box / detect_marker: `카메라 K 를 읽지 못함` | rs_stream(50001) 이 60 초 안에 안 뜬 것 — 카메라 연결·rs_stream 로그 확인 (고정 K 로 대체하지 않고 멈춤) |
| `프레임 WxH ≠ K 해상도 … 프레임 무시` | rs_stream 해상도(`RS_W`/`RS_H`)가 인식 서버 시작 뒤 바뀜 — `start_robot.sh` 다시 실행 |
| 손 `/dev/ttyACM0` 열기 실패 | `dialout` 그룹, ModemManager (11단계) |
| `pickle.load` 시 `class version ...` | 다른 pinocchio 버전의 캐시 → `robots/g1/g1_29_model_cache.pkl` 삭제 후 재생성 (14단계) |
| 로그가 늦게 써짐 | `stamp()` 를 거치지 않은 실행 — `start_*.sh` 로 실행 (mawk `-W interactive`) |
| 서버가 남아 있음 | `./start_robot.sh` 가 시작 시 TERM → 대기 → KILL 로 정리. 수동: `pgrep -af "python.*(rs_stream\|arm_server\|robot_server)"` 확인 후 `kill` (TERM 먼저) |
