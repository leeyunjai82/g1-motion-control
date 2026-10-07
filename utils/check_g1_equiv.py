"""G1 동등성 검사 — g1-motion-control.red(731075b) 원본 상수/동작 vs robot.yaml 로 바꾼 코드.

로봇 없이 실행 (unitree_sdk2py 는 가짜 모듈로 대체, numpy·pyyaml·fastapi·uvicorn 필요).
  git clone https://github.com/leeyunjai82/g1-motion-control.red /tmp/red && git -C /tmp/red checkout 731075b
  python utils/check_g1_equiv.py /tmp/red .

검사: 카메라·잡기·프레임·기본 팔 자세 상수, 관절 맵(순서 포함), IK 잠금/EE, URDF·IK 캐시 파일,
      robot_arm.py 초기화 후 LowCmd 35 슬롯 (mode/q/dq/tau/kp/kd), run_launcher HTML 원문·FSM 표·allowed() 진리표
"""
import ast, os, sys, types, time, threading, importlib, importlib.util, copy
import numpy as np

RED, NEW = sys.argv[1], sys.argv[2]
os.environ["ROBOT"] = "g1"
sys.path.insert(0, os.path.join(NEW, "common"))
import robot_env as RE

fails = []
def check(name, a, b):
    ok = (a == b) if not isinstance(a, np.ndarray) else np.array_equal(a, b)
    print(("  ✓ " if ok else "  ✗ ") + name + ("" if ok else f"\n      원본={a!r}\n      신규={b!r}"))
    if not ok: fails.append(name)

def consts(path, names):
    tree = ast.parse(open(path, encoding="utf-8").read())
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id in names:
                    out[t.id] = ast.literal_eval(node.value)
                if isinstance(t, ast.Tuple):
                    ids = [e.id for e in t.elts if isinstance(e, ast.Name)]
                    if set(ids) & set(names):
                        vals = ast.literal_eval(node.value)
                        out.update(dict(zip(ids, vals)))
    return out

H = lambda p: os.path.join(RED, "high", p)

print("[1] robot_server / arm_server / mission_server / detect_* 상수")
c = consts(H("robot_server.py"), {"CAMERA_X","CAMERA_Y","CAMERA_Z","CAMERA_PITCH_URDF","GRAB_Z_OFFSET","PELVIS_TO_TORSO","DEFAULT_ARM_DEG"})
check("robot_server CAMERA_X", c["CAMERA_X"], RE.CAMERA_X)
check("robot_server CAMERA_Y", c["CAMERA_Y"], RE.CAMERA_Y)
check("robot_server CAMERA_Z", c["CAMERA_Z"], RE.CAMERA_Z)
check("robot_server CAMERA_PITCH_URDF (비트)", c["CAMERA_PITCH_URDF"], RE.CAMERA_PITCH)
check("GRAB_Z_OFFSET", c["GRAB_Z_OFFSET"], float(RE.CFG["grab"]["z_offset"]))
check("PELVIS_TO_TORSO", c["PELVIS_TO_TORSO"], tuple(float(v) for v in RE.CFG["frames"]["pelvis_to_torso"]))
check("robot_server DEFAULT_ARM_DEG", c["DEFAULT_ARM_DEG"], [float(v) for v in RE.CFG["default_arm_deg"]])
c = consts(H("robot_server.py"), {"HANDOVER_X"})
check("HANDOVER_X", c["HANDOVER_X"], float(RE.CFG["grab"]["handover_x"]))
c = consts(H("robot_server.py"), {"WAIST_BASE_PITCH"})
check("WAIST_BASE_PITCH", c["WAIST_BASE_PITCH"], float(RE.CFG["grab"]["waist_base_pitch_deg"]))
check("G1 waist_locked = false (yaw 정렬·좌우 건네기 유지)", False, bool(RE.CFG["grab"].get("waist_locked", False)))
check("G1 features.locomotion = true (보행·추종 유지)", True, bool(RE.CFG.get("features", {}).get("locomotion", True)))
check("G1 frames.exact_ik_frame = false (카메라 torso 좌표를 그대로 IK 목표로 — 기존)", False, bool(RE.CFG["frames"].get("exact_ik_frame", False)))
check("실기 모드 DDS 도메인 0", 0, RE.DDS_DOMAIN)
c = consts(H("arm_server.py"), {"DEFAULT_ARM_DEG"})
check("arm_server DEFAULT_ARM_DEG", c["DEFAULT_ARM_DEG"], [float(v) for v in RE.CFG["default_arm_deg"]])
c = consts(H("mission_server.py"), {"CAMERA_X","CAMERA_Y","CAMERA_Z","CAMERA_PITCH"})
check("mission_server camera", (c["CAMERA_X"],c["CAMERA_Y"],c["CAMERA_Z"],c["CAMERA_PITCH"]),
      (RE.CAMERA_X, RE.CAMERA_Y, RE.CAMERA_Z, RE.CAMERA_PITCH))
for f in ("ctrl/detect_box.py", "ctrl/detect_box_conv.py", "ctrl/detect_marker.py"):
    c = consts(H(f), {"CAMERA_X","CAMERA_Y","CAMERA_Z","CAMERA_PITCH_URDF","CAM_TILT_DEG"})
    check(f"{f} camera", (c["CAMERA_X"],c["CAMERA_Y"],c["CAMERA_Z"],c["CAMERA_PITCH_URDF"]),
          (RE.CAMERA_X, RE.CAMERA_Y, RE.CAMERA_Z, RE.CAMERA_PITCH))
    if "CAM_TILT_DEG" in c:
        check(f"{f} CAM_TILT_DEG", c["CAM_TILT_DEG"], RE.CAMERA_PITCH_DEG)
        check(f"{f} GRAVITY_CAM (비트)", np.array([0.0, np.cos(np.radians(c['CAM_TILT_DEG'])), np.sin(np.radians(c['CAM_TILT_DEG']))]),
              np.array([0.0, np.cos(np.radians(RE.CAMERA_PITCH_DEG)), np.sin(np.radians(RE.CAMERA_PITCH_DEG))]))

print("[2] dashboard / urdf_sim JOINT_TO_MOTOR (순서 포함)")
for f in ("dashboard.py", "urdf_sim.py"):
    old = consts(H(f), {"JOINT_TO_MOTOR"})["JOINT_TO_MOTOR"]
    new = {str(k): int(v) for k, v in RE.JOINTS["map"].items()}
    check(f"{f} JOINT_TO_MOTOR", list(old.items()), list(new.items()))
check("dashboard NUM_MOTORS", consts(H("dashboard.py"), {"NUM_MOTORS"})["NUM_MOTORS"], int(RE.JOINTS["motor_slots"]))

print("[3] IK (robot_arm_ik.py)")
src = open(H("ctrl/robot_arm_ik.py"), encoding="utf-8").read()
i = src.index("self.mixed_jointsToLockIDs = ["); j = src.index("]", i)
old_lock = ast.literal_eval(src[src.index("[", i):j + 1])
check("lock_joints", old_lock, list(RE.CFG["ik"]["lock_joints"]))
check("ee_joints", ["left_wrist_yaw_joint", "right_wrist_yaw_joint"], list(RE.CFG["ik"]["ee_joints"]))
check("ee_offset", [0.05, 0, 0], list(RE.CFG["ik"]["ee_offset"]))
check("URDF 파일 동일", open(H("assets/g1/g1_29dof_rev_1_0.urdf"),"rb").read(), open(RE.URDF_PATH,"rb").read())
check("IK 캐시 동일", open(H("ctrl/g1_29_model_cache.pkl"),"rb").read(), open(RE.IK_CACHE_PATH,"rb").read())

print("[4] robot_arm.py — 가짜 DDS 로 초기화 후 LowCmd 내용 비교")
class MC:
    def __init__(self): self.mode=0; self.q=0.0; self.dq=0.0; self.tau=0.0; self.kp=0.0; self.kd=0.0
class LowCmd:
    def __init__(self): self.mode_pr=0; self.mode_machine=0; self.motor_cmd=[MC() for _ in range(35)]; self.crc=0
class MS:
    def __init__(self, i): self.q=0.01*i+0.003; self.dq=0.001*i
class IMU: rpy=[0,0,0]; accelerometer=[0,0,0]; gyroscope=[0,0,0]
class LS:
    def __init__(self): self.motor_state=[MS(i) for i in range(35)]; self.imu_state=IMU(); self.mode_machine=7
WRITES = []
class Pub:
    def __init__(self, topic, t): self.topic=topic
    def Init(self): pass
    def Write(self, m): WRITES.append((self.topic, copy.deepcopy(m)))
class Sub:
    def __init__(self, topic, t): pass
    def Init(self, *a, **k): pass
    def Read(self, *a): time.sleep(0.001); return LS()
def stub():
    for name in ["unitree_sdk2py","unitree_sdk2py.core","unitree_sdk2py.core.channel","unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_hg","unitree_sdk2py.idl.unitree_hg.msg","unitree_sdk2py.idl.unitree_hg.msg.dds_",
                 "unitree_sdk2py.idl.default","unitree_sdk2py.utils","unitree_sdk2py.utils.crc"]:
        sys.modules[name] = types.ModuleType(name)
    ch = sys.modules["unitree_sdk2py.core.channel"]
    ch.ChannelPublisher, ch.ChannelSubscriber, ch.ChannelFactoryInitialize = Pub, Sub, (lambda *a: None)
    d = sys.modules["unitree_sdk2py.idl.unitree_hg.msg.dds_"]; d.LowCmd_ = LowCmd; d.LowState_ = LS
    sys.modules["unitree_sdk2py.idl.default"].unitree_hg_msg_dds__LowCmd_ = LowCmd
    sys.modules["unitree_sdk2py.utils.crc"].CRC = type("CRC", (), {"Crc": lambda self, m: 0})
stub()

def run_arm(path, modname, cls):
    WRITES.clear()
    spec = importlib.util.spec_from_file_location(modname, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    a = getattr(m, cls)(motion_mode=True, simulation_mode=True)
    time.sleep(0.2)
    a.ctrl_dual_arm(np.linspace(-0.5, 0.5, 14), np.zeros(14))
    a.ctrl_waist([0.1, -0.05, 0.02])
    time.sleep(0.2)
    last = WRITES[-1]
    snap = [(c.mode, round(c.q, 9), c.dq, c.tau, c.kp, c.kd) for c in last[1].motor_cmd]
    return last[0], snap, a.get_current_dual_arm_q().tolist(), a.get_waist_q().tolist(), a.get_weight()

o = run_arm(H("ctrl/robot_arm.py"), "red_robot_arm", "G1_29_ArmController")
n = run_arm(os.path.join(NEW, "common/ctrl/robot_arm.py"), "new_robot_arm", "G1_29_ArmController")
check("토픽", o[0], n[0])
for i in range(35):
    if o[1][i] != n[1][i]:
        check(f"motor_cmd[{i}] (mode,q,dq,tau,kp,kd)", o[1][i], n[1][i])
check("motor_cmd 35 슬롯 전체", o[1], n[1])
check("get_current_dual_arm_q", o[2], n[2])
check("get_waist_q", o[3], n[3])
check("weight", o[4], n[4])

print("[5] run_launcher — HTML / allowed() / FSM 표")
import importlib.util as iu
def load_launcher(path, name):
    spec = iu.spec_from_file_location(name, path)
    m = iu.module_from_spec(spec)
    sys.argv = [path, "g1"]
    spec.loader.exec_module(m)
    return m
lo = load_launcher(os.path.join(RED, "run_launcher.py"), "red_launcher")
ln = load_launcher(os.path.join(NEW, "run_launcher.py"), "new_launcher")
check("HTML 원문 동일 (i18n 키 포함)", lo.HTML, ln.HTML)
for k in ("FSM_NAME", "FSM_BAL", "STANDING", "STEPS"):
    check(k, getattr(lo, k), getattr(ln, k))
tbl_o = {(t, c, r): lo.allowed(t, c, r) for t in (1, 3, 4, 501, 999) for c in (None, 0, 1, 2, 3, 4, 500, 501, 702, 801) for r in (False, True)}
tbl_n = {(t, c, r): ln.allowed(t, c, r) for t in (1, 3, 4, 501, 999) for c in (None, 0, 1, 2, 3, 4, 500, 501, 702, 801) for r in (False, True)}
check(f"allowed() 진리표 {len(tbl_o)}건 (사유 문구 포함)", tbl_o, tbl_n)

print("\n결과:", "✓ 전부 동일" if not fails else f"✗ 불일치 {len(fails)}건: {fails}")
sys.exit(1 if fails else 0)
