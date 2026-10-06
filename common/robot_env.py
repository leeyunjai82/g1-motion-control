"""
robot_env.py — 실행 대상 로봇 선택 (ROBOT 환경변수) + 로봇별 설정(robot.yaml)

  ROBOT=g1 python robot_server.py

  · ROBOT 미지정 / 미지원 / robot.yaml 의 enabled=false 이면 import 시점에 바로 종료한다 (기본값 없음).
    다른 로봇을 연결해 둔 채 잘못된 설정으로 지령을 보내는 사고를 막기 위함.
  · 로봇별 파일(URDF, 메시, 모션 JSON, IK 모델 캐시, robot.yaml)은 robots/<ROBOT>/ 에 둔다.
  · 로봇마다 다른 값은 robots/<ROBOT>/robot.yaml → CFG 로 읽는다.
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
    print(f"[robot_env]    사용 예: ROBOT=g1 ./start_robot.sh   (robots/: {', '.join(_available())})",
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


ROBOT = os.environ.get("ROBOT", "").strip().lower()
if not ROBOT:
    _fail("ROBOT 환경변수가 없습니다 — 실행 거부 (기본값 없음)")

ROBOT_DIR = os.path.join(ROBOTS_DIR, ROBOT)
try:
    CFG = load_config(ROBOT)
except Exception as e:  # noqa: BLE001 — 어떤 오류든 실행 거부
    _fail(f"ROBOT='{ROBOT}' 설정 오류 — 실행 거부: {e}")
if not CFG.get("enabled", False):
    _fail(f"ROBOT='{ROBOT}' 는 robot.yaml 에서 enabled: false — 실행 거부 "
          f"({CFG.get('disabled_reason', '사유 미기재')})")

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
CAMERA_PITCH = float(np.radians(CAMERA_PITCH_DEG))   # rad (G1 47.6° → 0.8307767239493009, 비트 동일)

JOINTS = CFG["joints"]
FSM    = CFG["fsm"]


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
