"""
robot_env.py — 로봇 설정(robots/g1/robot.yaml) 읽기. 이 저장소는 G1 전용.

  python robot_server.py            # ROBOT 을 안 주면 g1

  · ROBOT 환경변수는 남겨 두었지만 g1 만 받는다 (다른 값이면 import 시점에 종료).
  · robot.yaml 의 enabled=false 면 실행 거부 (시뮬 ROBOT_SIM=1 · 모터 번호 확인 ROBOT_CHECK=1 제외).
  · 로봇 파일(URDF, 메시, 모션 JSON, IK 모델 캐시, robot.yaml)은 robots/g1/ 에 있다.
  · unitree_sdk2py 는 tv 환경에 설치된 것 (requirements.txt 고정 커밋 f559291).
"""
import os
import sys

import numpy as np
import yaml

REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMON_DIR = os.path.join(REPO_ROOT, "common")
ROBOTS_DIR = os.path.join(REPO_ROOT, "robots")


def _available():
    try:
        return sorted(d for d in os.listdir(ROBOTS_DIR)
                      if os.path.isfile(os.path.join(ROBOTS_DIR, d, "robot.yaml")))
    except OSError:
        return []


def _fail(msg):
    print(f"[robot_env] ❌ {msg}", file=sys.stderr)
    print(f"[robot_env]    이 저장소는 G1 전용 — ROBOT 은 비우거나 g1   (robots/: {', '.join(_available())})",
          file=sys.stderr)
    sys.exit(2)


def load_config(robot):
    """robots/<robot>/robot.yaml 을 읽어 dict 로 (검증 포함). 실패 시 ValueError."""
    path = os.path.join(ROBOTS_DIR, robot, "robot.yaml")
    if not os.path.isfile(path):
        raise ValueError(f"설정 파일 없음: {path}")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict) or cfg.get("name") != robot:
        raise ValueError(f"{path}: name 이 '{robot}' 가 아님")

    j = cfg["joints"]
    n = int(j["motor_slots"])
    slots = list(j["arm"]) + list(j["waist"]) + list(j.get("head") or []) + [j["weight_slot"]]
    if len(j["arm"]) != 14 or len(j["waist"]) != 3:
        raise ValueError(f"{path}: arm 14개 / waist 3개 이어야 함")
    if any(not (0 <= int(s) < n) for s in slots + list(j["init_slots"]) + list(j["map"].values())):
        raise ValueError(f"{path}: 슬롯 번호가 0~{n - 1} 범위 밖")
    if len(set(slots)) != len(slots):
        raise ValueError(f"{path}: arm/waist/head/weight 슬롯 중복")
    for k in ("x", "y", "z", "pitch_deg"):
        if cfg["camera"].get(k) is None:
            raise ValueError(f"{path}: camera.{k} 없음")
    _k = cfg["camera"].get("intrinsics")
    if not (isinstance(_k, list) and len(_k) == 4 and all(isinstance(v, (int, float)) for v in _k)):
        raise ValueError(f"{path}: camera.intrinsics [fx, fy, ppx, ppy] (컬러 640×480) 4개 필요")

    # 실행에 꼭 필요한 값 — enabled: true 일 때만 검사 (enabled: false 로봇은 "확인 필요" 를 null 로 둔다)
    if cfg.get("enabled", False):
        if not isinstance(cfg.get("default_arm_deg"), list) or len(cfg["default_arm_deg"]) != 14:
            raise ValueError(f"{path}: default_arm_deg 14개 필요")
        if cfg.get("frames", {}).get("pelvis_to_torso") is None or cfg.get("grab", {}).get("z_offset") is None:
            raise ValueError(f"{path}: frames.pelvis_to_torso / grab.z_offset 필요")
        ident = cfg.get("identity") or {}
        if ident.get("required") and ident.get("mode_machine") is None:
            raise ValueError(f"{path}: identity.required 인데 mode_machine 이 null")
        if j.get("head") and (cfg["gains"].get("kp_head") is None or cfg["gains"].get("kd_head") is None):
            raise ValueError(f"{path}: head 슬롯이 있으면 gains.kp_head / kd_head 필요")
    return cfg


# 시뮬레이션 모드 (ROBOT_SIM=1, start_sim.sh) — sim/fake_robot.py 가 로봇 역할.
#   DDS 도메인 1 을 써서 실기(도메인 0)와 절대 섞이지 않게 한다.
#   enabled: false 로봇(H2 등)도 시뮬에서는 실행 허용 (실기 명령 경로는 계속 닫힘).
SIM = os.environ.get("ROBOT_SIM", "").strip() == "1"
DDS_DOMAIN = 1 if SIM else 0

# 실기 모터 번호 확인 모드 (ROBOT_CHECK=1, start_simulator.sh real) — enabled: false 로봇을 실기에서
# '확인 목적으로만' 띄운다. 허용 프로세스: arm_server / simulator / dashboard 뿐 (잡기·보행 서버는 거부).
CHECK = os.environ.get("ROBOT_CHECK", "").strip() == "1"
CHECK_ALLOWED = {"arm_server.py", "simulator.py", "dashboard.py", "init_fsm.py", "robot_state.py", "cam_marker_check.py", "hand_to_marker.py"}   # init_fsm: start_fsm.sh, robot_state: 읽기 전용 진단

ROBOT = os.environ.get("ROBOT", "").strip().lower() or "g1"
if ROBOT != "g1":
    _fail(f"ROBOT='{ROBOT}' — 이 저장소는 G1 전용 (H2 는 h2-motion-control)")

ROBOT_DIR = os.path.join(ROBOTS_DIR, ROBOT)
try:
    CFG = load_config(ROBOT)
except Exception as e:  # noqa: BLE001 — 어떤 오류든 실행 거부
    _fail(f"ROBOT='{ROBOT}' 설정 오류 — 실행 거부: {e}")
if CHECK and os.path.basename(sys.argv[0]) not in CHECK_ALLOWED:
    _fail(f"ROBOT_CHECK=1 (모터 번호 확인 모드) 에서는 {sorted(CHECK_ALLOWED)} 만 실행 — "
          f"{os.path.basename(sys.argv[0])} 거부")
if not CFG.get("enabled", False):
    if not (SIM or CHECK):
        _fail(f"ROBOT='{ROBOT}' 는 robot.yaml 에서 enabled: false — 실행 거부 "
              f"({CFG.get('disabled_reason', '사유 미기재')})")
    if SIM:
        print(f"[robot_env] ⚠️ 시뮬레이션: ROBOT='{ROBOT}' enabled: false 지만 ROBOT_SIM=1 (DDS 도메인 {DDS_DOMAIN}) 로 실행",
              file=sys.stderr)
    else:
        print(f"[robot_env] ⚠️⚠️ 실기 모터 번호 확인 모드: ROBOT='{ROBOT}' enabled: false — "
              f"{os.path.basename(sys.argv[0])} 만 실행 (잡기·보행 서버 거부). 로봇을 지지한 상태에서만 사용",
              file=sys.stderr)
if SIM:
    # 실기에서 아직 재지 않은 값 — 시뮬 전용 값(robot.yaml sim:)으로 채운다 (실기 경로에는 영향 없음)
    _sim = CFG.get("sim") or {}
    if CFG.get("default_arm_deg") is None:
        CFG["default_arm_deg"] = list(_sim.get("default_arm_deg") or [0.0] * 14)
        print(f"[robot_env] ⚠️ 시뮬레이션: default_arm_deg 미측정 → sim 값 {CFG['default_arm_deg']}", file=sys.stderr)

URDF_PATH     = os.path.join(ROBOT_DIR, CFG["urdf"])
MESH_DIR      = os.path.join(ROBOT_DIR, "meshes")
MOTIONS_DIR   = os.path.join(ROBOT_DIR, "motions")
IK_CACHE_PATH = os.path.join(ROBOT_DIR, CFG["ik_cache"])
VENDOR_DIR    = os.path.join(COMMON_DIR, "assets", "vendor")   # three.min.js (로봇 무관)

# ---- 자주 쓰는 값 ----
CAMERA       = CFG["camera"]
CAMERA_X     = float(CAMERA["x"])
CAMERA_Y     = float(CAMERA["y"])
CAMERA_Z     = float(CAMERA["z"])
CAMERA_PITCH_DEG = float(CAMERA["pitch_deg"])
CAMERA_PITCH = float(np.radians(CAMERA_PITCH_DEG))
CAMERA_K     = tuple(float(v) for v in CAMERA["intrinsics"])   # (fx, fy, ppx, ppy) 컬러 640×480 — 시뮬 가상 카메라 전용 (실기 detect_* 는 rs_stream /intrinsics)

JOINTS = CFG["joints"]
FSM    = CFG["fsm"]


_dds_inited = None

# DDS 네트워크 인터페이스 (실기만). 비우면 자동 선택 — 기존과 같음.
#   Wi-Fi 와 유선이 같이 켜진 PC 에서 자동 선택이 Wi-Fi 를 잡으면 로봇(192.168.123.x) 데이터가 안 들어온다.
#   DDS_IFACE=enp3s0 ./start_robot.sh
#   sudo 는 환경변수를 지우므로 init_fsm.py / run_launcher.py 는 인자 iface=<이름> 으로 받아 여기에 다시 넣는다.
#   시뮬(ROBOT_SIM=1, 로컬 도메인 1)에는 적용하지 않는다 — 케이블을 뽑은 PC 에서도 시뮬이 돌게.
DDS_IFACE = os.environ.get("DDS_IFACE", "").strip()


def dds_init(interface=None):
    """ChannelFactoryInitialize(DDS_DOMAIN[, interface]).

    interface 를 안 주면 실기에서는 DDS_IFACE (없으면 자동 선택).
    실기: 기존과 똑같이 매번 호출한다 (동작 변경 없음).
    시뮬: 같은 프로세스에서 두 번째 호출은 건너뛴다 — 일부 환경에서 cyclonedds 가 같은 도메인을
          다시 만들 때 'create domain error' 로 실패하기 때문 (arm_server → robot_arm 이 두 번 부름).
    """
    global _dds_inited
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    if SIM and _dds_inited is not None:
        return
    iface = interface or ("" if SIM else DDS_IFACE)
    if iface and not os.path.isdir(os.path.join("/sys/class/net", iface)):
        print(f"[robot_env] ❌ DDS 인터페이스 '{iface}' 없음 — ip -br link 로 이름 확인 (DDS_IFACE 를 비우면 자동 선택)",
              file=sys.stderr)
        sys.exit(2)
    if iface:
        if _dds_inited is None:
            print(f"[robot_env] DDS 인터페이스: {iface}", file=sys.stderr)
        ChannelFactoryInitialize(DDS_DOMAIN, iface)
    else:
        ChannelFactoryInitialize(DDS_DOMAIN)
    _dds_inited = DDS_DOMAIN


def own_by_sudo_user(path):
    """sudo 로 실행돼 root 가 만든 파일을 원래 사용자(SUDO_UID/SUDO_GID) 소유로 — h2-motion-control 7b4e607 과 같은 방식.
    root 소유로 남으면 나중에 일반 계정으로 실행할 때 덮어쓰기 실패. sudo 가 아니면 아무것도 안 함."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid and gid and os.geteuid() == 0:
        try:
            os.chown(path, int(uid), int(gid))
        except OSError:
            pass


def loco_client_class():
    """robot.yaml sdk.loco_module 의 LocoClient 클래스 (import 실패 시 ImportError)."""
    import importlib
    return importlib.import_module(CFG["sdk"]["loco_module"]).LocoClient


def check_identity(lowstate_msg):
    """rt/lowstate 메시지로 연결된 로봇이 설정과 같은지 확인 → (ok, 메시지).

    unitree_hg LowState_.motor_state 는 35칸 고정이라 관절 수로는 구분할 수 없어
    mode_machine 을 기준으로 쓴다. 기준값이 null 이면 검사 생략(경고).
    """
    want = (CFG.get("identity") or {}).get("mode_machine")
    got = int(getattr(lowstate_msg, "mode_machine", -1))
    if want is None:
        return True, (f"[robot_env] ⚠️ ROBOT={ROBOT} identity.mode_machine 미설정 — 로봇 확인 생략 "
                      f"(수신 mode_machine={got}, utils/check_robot_id.py 로 측정 후 robot.yaml 에 기입)")
    if got != int(want):
        return False, (f"[robot_env] ❌ 연결된 로봇 mode_machine={got} ≠ robots/{ROBOT}/robot.yaml "
                       f"identity.mode_machine={want} — 다른 로봇이 연결된 것으로 보고 거부")
    return True, f"[robot_env] ✓ 로봇 확인: ROBOT={ROBOT} mode_machine={got}"
