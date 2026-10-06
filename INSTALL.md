# 설치 가이드 — Ubuntu 24.04 LTS

외장 Intel mini PC(x86_64)에 **Ubuntu 24.04 LTS** 를 기준으로 설치하는 절차입니다. 순서대로 진행하세요.

- 사용법 : [`README.md`](./README.md)
- 내부 구조 : [`TECH.md`](./TECH.md)

> "확인 필요" 표시는 이 문서 작성 시점에 실기/운영 PC 에서 검증하지 못한 항목입니다. 설치하면서 결과를 기록해 주세요.

---

## 1. 시스템 요구사항

| 항목 | 권장 |
| --- | --- |
| OS | **Ubuntu 24.04 LTS** (x86_64) |
| Python | 3.10 (Miniconda `tv` 환경) |
| 메모리 / 디스크 | 8 GB 이상 / 10 GB 이상 여유 |
| 네트워크 | 로봇과 같은 LAN `192.168.123.0/24` (유선 권장) |
| 카메라 | Intel RealSense D435i (USB 3.0 포트) |
| 권한 | sudo 가능한 계정 |

설치 경로 예: `$HOME/project/g1-motion-control/`.
스크립트는 계정명을 하드코딩하지 않습니다 (`$HOME`, 스크립트 위치 기준).

---

## 2. OS 기본 패키지

```bash
sudo apt update
sudo apt install -y \
    build-essential git wget curl \
    cmake pkg-config \
    libusb-1.0-0-dev \
    libgl1-mesa-dev libglu1-mesa-dev libglvnd-dev \
    libglfw3 libglfw3-dev \
    python3-dev \
    iproute2
```

- `curl` : `start_simulator.sh` 가 arm_server 상태 확인에 사용.
- `iproute2`(`ss`) : `launcher.sh` 의 포트 80 점유 확인에 사용.
- 로그 타임스탬프(`awk strftime`) : 24.04 기본 `mawk 1.3.4 20240123` 에서 동작 확인.
  단 mawk 는 파이프 입력을 블록 단위로 모아 읽어 **로그가 몇 KB 씩 늦게 기록**된다 → `robot_env.sh` 의 `stamp()` 가 mawk 면 `-W interactive` 로 줄 단위 처리 (확인함). gawk 가 있으면 그대로 사용.

---

## 3. Intel RealSense (D435i)

Python 은 pip 의 `pyrealsense2` (requirements.txt, `2.55.1.6486`) 를 씁니다.
시스템 쪽에는 **장치 권한(udev 규칙)** 과 확인용 도구만 설치합니다.

> ⚠️ **24.04 에서는 `librealsense2-dkms` 를 설치하지 마세요.** 공식 문서상 DKMS 패키지는 HWE 커널 5.15 / 5.19 / 6.5 만 지원하고, 24.04 커널(6.8 이상)은 대상이 아닙니다.
> 출처: <https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md>

공식 저장소 등록 (2026 기준 주소가 `librealsense.realsenseai.com` 으로 바뀌었습니다):

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
    gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y librealsense2-utils     # librealsense2-udev-rules 같이 설치됨
```

카메라를 USB 3.0 포트에 연결하고 확인:

```bash
rs-enumerate-devices | head -20     # "Usb Type Descriptor : 3.2" 확인
realsense-viewer                    # 컬러/깊이 영상 확인
```

- 확인 필요: 24.04 기본 커널 드라이버(uvcvideo)만으로 D435i 의 color/depth 는 동작하지만, **IMU(HID) 와 프레임 메타데이터** 는 커널 패치 없이 나오지 않을 수 있습니다. 현재 코드는 IMU 를 쓰지 않습니다(`CAM_TILT_DEG` 고정값). D435i IMU 로 중력 보정을 넣을 때 다시 확인합니다.
- apt 저장소 접근이 막힌 환경이면 udev 규칙만 수동 설치해도 됩니다: librealsense 소스의 `config/99-realsense-libusb.rules` 를 `/etc/udev/rules.d/` 에 복사 → `sudo udevadm control --reload-rules && sudo udevadm trigger`.

---

## 4. Miniconda

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
source $HOME/miniconda3/bin/activate
conda init bash
```

새 터미널에서 `conda --version` 확인.

> 다른 위치(`~/anaconda3`, `~/miniforge3`, `/opt/conda`)에 설치해도 스크립트가 자동으로 찾습니다. 그 외 위치면 `export CONDA_BASE=<경로>`.

---

## 5. `tv` 환경 생성

스크립트가 `tv` 라는 이름을 씁니다. 이름을 바꾸지 마세요.

```bash
# xr_teleoperate 공식 절차와 같은 구성 (pinocchio 는 반드시 conda-forge — 7-1 참고)
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge -y
conda activate tv
```

> `CondaToSNonInteractiveError` 가 나오면:
> ```bash
> conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
> conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
> ```

---

## 6. 프로젝트 받기

```bash
mkdir -p $HOME/project && cd $HOME/project
git clone https://github.com/leeyunjai82/g1-motion-control.git
cd g1-motion-control
```

---

## 7. Python 패키지

네이티브 빌드용 시스템 라이브러리 먼저:

```bash
sudo apt install -y libssl-dev bison flex     # cyclonedds
sudo apt install -y libsuitesparse-dev        # scikit-sparse (cholmod.h)
```

```bash
conda activate tv
pip install --upgrade pip
pip install -r requirements.txt
```

### 7-1. requirements.txt 에 없는 패키지 (확인 필요)

코드는 아래 패키지를 import 하지만 `requirements.txt` 에는 들어 있지 않습니다 (기준 소스 그대로).
**버전은 기존 G1 운영 PC 에서 확인해 같은 버전으로 맞추세요:**

```bash
# 기존 운영 PC 에서
pip freeze | grep -iE "^(pin|pinocchio|casadi|meshcat|logging.mp|openvino)"
```

| import | 사용처 | 설치 |
| --- | --- | --- |
| `pinocchio`, `pinocchio.casadi` | IK (`common/ctrl/robot_arm_ik.py`), arm_server, robot_server | 5단계 `conda create ... pinocchio=3.1.0 -c conda-forge` |
| `casadi` | IK 최적화 | conda-forge pinocchio 와 함께 설치됨 (버전 확인 필요) |
| `meshcat` | IK 시각화 (import 는 항상 함) | `pip install meshcat==0.3.2` (xr_teleoperate requirements 값) |
| `logging_mp` | IK 로거 | `pip install logging-mp` (unitreerobotics/logging-mp, 운영 PC 버전 확인 필요) |
| `openvino` | 박스 인식 (ultralytics `intel:cpu`), `utils/get_dev.py` | `pip install openvino` (운영 PC 버전 확인 필요) |

> ⚠️ **pip 의 `pin` 패키지는 쓰지 마세요.** PyPI `pin` 휠에는 `pinocchio.casadi` 가 들어 있지 않아 `from pinocchio import casadi` 가 `ImportError` 로 실패합니다 (2026-10 확인). conda-forge `pinocchio` 를 쓰세요.

> ⚠️ IK 모델 캐시(`robots/g1/g1_29_model_cache.pkl`)는 만든 pinocchio 버전에 묶입니다. 버전이 다르면 `pickle.load` 에서 `class version ...` 오류 → 캐시를 지우면 URDF 에서 다시 만듭니다.

### 7-2. NumPy 1.x 유지

`opencv-python 4.10` 은 NumPy 1.x ABI 입니다. 2.x 가 들어오면 정상 배열에도 `cv2.imencode` 가 `img is not a numpy array` 오류를 냅니다.

```bash
python -c "import numpy; print(numpy.__version__)"   # 반드시 < 2
pip install "numpy<2"                                 # 2.x 면
```

> `unitree_sdk2py` 는 requirements.txt 의 GitHub 고정 커밋(`f559291`)으로 설치됩니다. 이 커밋에는 `unitree_sdk2py/h2` 가 **없습니다** — H2 지원 단계에서 SDK 버전을 올려야 합니다.

---

## 8. sudo 설정 (`start_fsm.sh`)

`start_fsm.sh` 는 tv 환경 python 을 `sudo` 로 실행합니다. 비밀번호 없이 쓰려면 NOPASSWD 규칙을 추가합니다.

```bash
sudo visudo -f /etc/sudoers.d/g1-motion
```
```
<사용자> ALL=(root) NOPASSWD: /home/<사용자>/miniconda3/envs/tv/bin/python
```

- 경로는 스크립트가 실제 실행하는 python 과 **정확히 같아야** 합니다. 확인:
  ```bash
  source robot_env.sh && echo "$TV_PY"
  sudo -n "$TV_PY" -c "print('ok')"     # 비밀번호를 묻지 않아야 정상
  ```
- `ROBOT` 은 sudo 뒤에서 **명령 인자**로 넘기므로(`init_fsm.py <mode> <robot>`, `run_launcher.py <robot>`) sudoers 에 `SETENV` 를 줄 필요가 없습니다.
- `launcher.sh` 는 시작 시 `sudo -v` 로 비밀번호를 한 번 묻습니다 (NOPASSWD 불필요).

---

## 9. 네트워크

G1 기본 IP `192.168.123.161`. PC 유선 LAN 을 같은 대역으로 설정합니다 (G1/H2 공통).

```bash
nmcli connection show                     # 유선 연결 이름 확인
sudo nmcli connection modify "<연결이름>" ipv4.addresses 192.168.123.222/24 ipv4.method manual
sudo nmcli connection up "<연결이름>"
ping 192.168.123.161
```

- 방화벽: Ubuntu 기본 `ufw` 는 비활성입니다 (`sudo ufw status`). 켜져 있다면 로봇 대역을 허용하세요: `sudo ufw allow from 192.168.123.0/24`.
- DDS 인터페이스: 서버들은 `ChannelFactoryInitialize(0)` 로 인터페이스를 자동 선택합니다. Wi-Fi 와 유선이 같이 켜진 PC 에서 로봇 데이터가 안 들어오면 Wi-Fi 를 끄고 확인하세요 (인터페이스 지정 옵션은 3단계에서 공통 설정으로 추가 예정).

---

## 10. 손 컨트롤러 시리얼 (`/dev/ttyACM0`)

```bash
sudo usermod -aG dialout $USER     # 다시 로그인 후 적용
ls -l /dev/ttyACM0
```

- Ubuntu 의 `ModemManager` 가 새로 꽂힌 ttyACM 장치를 모뎀으로 검사하며 잠시 점유할 수 있습니다. 손 연결이 간헐적으로 실패하면: `sudo systemctl disable --now ModemManager` (모뎀을 안 쓰는 PC 에서만).

---

## 11. 성능: BLAS 스레드 1개 고정 (중요)

양팔 IK(pinocchio + CasADi)는 작은 선형계를 풉니다. 멀티스레드 OpenBLAS 에서는 동기화 오버헤드가 커서 IK 1회가 ~1.5 ms → ~40 ms 로 느려질 수 있습니다.

```bash
mkdir -p $HOME/miniconda3/envs/tv/etc/conda/activate.d
cat > $HOME/miniconda3/envs/tv/etc/conda/activate.d/threads.sh << 'EOF'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
EOF
conda deactivate && conda activate tv && echo $OMP_NUM_THREADS   # 1
```

> start 스크립트는 `activate_tv.sh` 로 tv 를 활성화하므로 위 훅이 적용됩니다.

---

## 12. 실행 권한

```bash
chmod +x robot_env.sh activate_tv.sh start_fsm.sh start_robot.sh start_simulator.sh start_mission.sh start_sim.sh launcher.sh
```

---

## 13. 설치 검증

```bash
source activate_tv.sh

# (1) ROBOT 미지정 → 거부되어야 정상
python -c "import sys; sys.path.insert(0,'common'); import robot_env"     # ❌ 메시지 + 종료코드 2

# (2) ROBOT=g1 경로 확인
ROBOT=g1 python -c "import sys; sys.path.insert(0,'common'); import robot_env as r; print(r.URDF_PATH, r.MOTIONS_DIR)"

# (3) 핵심 패키지
python -c "import numpy, cv2, torch, pyrealsense2, pinocchio, casadi, unitree_sdk2py; print('OK')"

# (4) NumPy 1.x + OpenCV 인코딩
python -c "
import numpy as np, cv2
assert np.__version__ < '2', np.__version__
print('imencode', cv2.imencode('.jpg', np.zeros((480,640,3), np.uint8))[0])"

# (5) RealSense
python -c "import pyrealsense2 as rs; print(rs.context().devices[0].get_info(rs.camera_info.name))"

# (6) (선택) IK 벤치마크 — 스레드 고정 시 수 ms
cd common && ROBOT=g1 python -c "
import time, numpy as np, pinocchio as pin
from ctrl.robot_arm_ik import G1_29_ArmIK
ik = G1_29_ArmIK(); q=np.zeros(14); dq=np.zeros(14)
L=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3, 0.2,0.1])).homogeneous
R=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3,-0.2,0.1])).homogeneous
ik.solve_ik(L,R,q,dq); t=time.time()
for _ in range(50): ik.solve_ik(L,R,q,dq)
print(f'IK avg: {(time.time()-t)/50*1000:.1f} ms')"; cd ..

# (7) 로봇 식별값 (로봇 연결 후, 읽기 전용) — 3단계 안전장치 기준값 측정용
python utils/check_robot_id.py
```

---

## 문제 해결

| 증상 | 원인 / 해결 |
| --- | --- |
| `ROBOT 이 지정되지 않았습니다 — 실행 거부` | `export ROBOT=g1` 또는 `ROBOT=g1 ./start_robot.sh` |
| `conda 를 찾지 못했습니다` | `export CONDA_BASE=<conda 설치 경로>` |
| sudo 가 계속 비밀번호 요구 (FSM) | sudoers 경로 ≠ `$TV_PY`. 8단계 확인 |
| `CondaToSNonInteractiveError` | 5단계 ToS 승인 |
| `cyclonedds` 빌드 실패 | `libssl-dev bison flex` 설치 |
| `scikit-sparse`: `cholmod.h` 없음 | `libsuitesparse-dev` 설치 |
| OpenCV `img is not a numpy array` | NumPy 2.x. `pip install "numpy<2"` |
| IK 가 수십 ms | BLAS 스레드 고정 (11단계) |
| `librealsense2-dkms` 설치/빌드 실패 | 24.04 커널 미지원 — 설치하지 않음 (3단계) |
| RealSense 인식 안 됨 / 권한 오류 | USB 3.0 포트, udev 규칙(`librealsense2-udev-rules`) 설치 후 재연결 |
| 손 `/dev/ttyACM0` 열기 실패 | `dialout` 그룹, ModemManager (10단계) |
| `pickle.load` 시 `class version ...` | 다른 pinocchio 버전의 캐시 → `robots/g1/g1_29_model_cache.pkl` 삭제 후 재생성 |
| 서버가 남아 있음 | `ROBOT=g1 ./start_robot.sh` 가 시작 시 TERM → 대기 → KILL 로 정리. 수동: `pgrep -af "python.*(rs_stream|arm_server|robot_server)"` 확인 후 `kill` |
