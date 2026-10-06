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

## 2. OS 기본 패키지 (시스템 — 최소)

Python 쪽은 전부 `tv` 가상환경에 넣고, 시스템에는 꼭 필요한 것만 설치합니다.

```bash
sudo apt update
sudo apt install -y wget curl
```

- `curl` : `start_sim.sh` / `start_simulator.sh` 가 서버 응답 확인에 사용 (가상환경 활성화 전에 호출).
- 빌드 도구(build-essential, libssl-dev, libsuitesparse-dev 등)는 **필요 없습니다** — cyclonedds 는 pip 바이너리 휠,
  scikit-sparse 는 conda-forge 패키지로 설치합니다.
- OpenCV 가 `libGL.so.1` 을 찾지 못하면 (서버판 Ubuntu 등): `sudo apt install -y libgl1 libglib2.0-0`
- 로그 타임스탬프(`awk strftime`) : 24.04 기본 `mawk 1.3.4 20240123` 에서 동작 확인.
  mawk 는 파이프 입력을 블록 단위로 모아 읽어 로그가 늦게 기록되므로 `robot_env.sh` 의 `stamp()` 가 `-W interactive` 로 줄 단위 처리.

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

## 4. Miniconda — `.bashrc` 를 건드리지 않고 설치

로봇을 돌리는 계정 하나에만 설치합니다. **`conda init` 을 하지 않으므로** 다른 계정과 이 계정의 평소 셸에는 영향이 없습니다.

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3     # -b: 묻지 않음, .bashrc 수정 없음
rm Miniconda3-latest-Linux-x86_64.sh
$HOME/miniconda3/bin/conda --version
```

> `conda init` 을 실행하지 마세요. 필요할 때만 활성화합니다 (아래 5단계 · "평소 사용").
> 다른 위치(예: `/opt/miniconda3`)에 설치했다면 `export CONDA_BASE=/opt/miniconda3` — 스크립트가 이 값을 씁니다.

---

## 5. `tv` 가상환경 생성 (conda-forge 만 사용)

```bash
source $HOME/miniconda3/bin/activate          # 이 터미널에서만 conda 사용
conda create -y -n tv --override-channels -c conda-forge \
    python=3.10 pinocchio=3.1.0 numpy=1.24.4 scikit-sparse=0.4.16 git
conda activate tv
```

- `--override-channels -c conda-forge` : Anaconda 기본 채널을 쓰지 않으므로 약관(ToS) 승인 절차가 없습니다 (conda 26.7.1 확인).
- `pinocchio=3.1.0` : xr_teleoperate 공식 절차와 같은 버전. **pip 의 `pin` 패키지는 `pinocchio.casadi` 가 없어 쓰면 안 됩니다.**
- `numpy=1.24.4` : requirements.txt 와 같은 값 (pip 가 numpy 를 다시 바꾸지 않게). OpenCV 4.10 때문에 2.x 금지.
- `scikit-sparse` : conda-forge 바이너리 → 시스템 `libsuitesparse-dev` 불필요.
- `git` : 가상환경 안의 git (unitree_sdk2py 를 GitHub 에서 받을 때 사용).

---

## 6. 프로젝트 받기

```bash
mkdir -p $HOME/project && cd $HOME/project
git clone https://github.com/leeyunjai82/g1-motion-control.git      # tv 활성화 상태 → 가상환경의 git
cd g1-motion-control
```

---

## 7. Python 패키지 (전부 `tv` 안)

```bash
# (tv 활성화 상태, 프로젝트 폴더에서)
pip install --upgrade pip

# PyTorch 는 CPU 판 (Intel mini PC — CUDA 판은 수 GB 의 nvidia 패키지가 같이 깔림)
pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cpu

# 나머지 — unitree_sdk2py 소스는 가상환경 안($CONDA_PREFIX/src)에 받는다
pip install -r requirements.txt --src "$CONDA_PREFIX/src"

# requirements.txt 에 없는 것
pip install meshcat==0.3.2 logging-mp openvino
```

확인:
```bash
python -c "import numpy, torch, cv2, pinocchio, sksparse, unitree_sdk2py, openvino; from pinocchio import casadi; print('numpy', numpy.__version__, '| torch', torch.__version__)"
# numpy 1.24.4 | torch 2.4.1+cpu 가 나와야 정상
```

- 검증 (2026-10, Ubuntu 24.04): 위 순서로 설치 후 `ROBOT=h2 ./start_sim.sh` 잡기 시퀀스 10단계 완료.
  단, 검증 환경에서는 `download.pytorch.org` 접근이 막혀 CPU 판 대신 CUDA 판 torch 로 확인했습니다 (CPU 판 설치 명령은 확인 필요 — 위 확인 명령에서 `+cpu` 인지 보세요).
- `openvino`, `logging-mp` 는 버전을 고정하지 않았습니다 — 기존 G1 운영 PC 의 `pip freeze` 값과 맞추는 것을 권장 (확인 필요).
- `unitree_sdk2py` 는 requirements 의 고정 커밋 `f559291` (G1). 이 커밋에는 `unitree_sdk2py.h2` 가 없습니다 — H2 실기 FSM 은 SDK 업그레이드 후.

### 평소 사용 (conda 는 필요할 때만)

- `start_*.sh` / `launcher.sh` 는 스스로 `tv` 를 활성화합니다 → **그냥 실행하면 됩니다.**
- 터미널에서 직접 python 을 쓸 때만:
  ```bash
  source $HOME/miniconda3/bin/activate tv      # 또는 프로젝트 폴더에서: source activate_tv.sh
  ...
  conda deactivate                             # 끝나면
  ```

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
| `CondaToSNonInteractiveError` | 기본 채널을 쓴 경우 — 5단계처럼 `--override-channels -c conda-forge` 로 다시 생성 |
| `cyclonedds` 빌드 실패 | Python 3.10 휠이 안 받아진 경우 — `python -V` 가 3.10 인지(tv 활성화) 확인 |
| `scikit-sparse`: `cholmod.h` 없음 | pip 로 빌드하려 한 것 — 5단계 conda-forge `scikit-sparse` 로 설치 |
| OpenCV `img is not a numpy array` | NumPy 2.x. `pip install "numpy<2"` |
| IK 가 수십 ms | BLAS 스레드 고정 (11단계) |
| `librealsense2-dkms` 설치/빌드 실패 | 24.04 커널 미지원 — 설치하지 않음 (3단계) |
| RealSense 인식 안 됨 / 권한 오류 | USB 3.0 포트, udev 규칙(`librealsense2-udev-rules`) 설치 후 재연결 |
| 손 `/dev/ttyACM0` 열기 실패 | `dialout` 그룹, ModemManager (10단계) |
| `pickle.load` 시 `class version ...` | 다른 pinocchio 버전의 캐시 → `robots/g1/g1_29_model_cache.pkl` 삭제 후 재생성 |
| 서버가 남아 있음 | `ROBOT=g1 ./start_robot.sh` 가 시작 시 TERM → 대기 → KILL 로 정리. 수동: `pgrep -af "python.*(rs_stream|arm_server|robot_server)"` 확인 후 `kill` |
