import os
import sys
import time

# 로봇 선택 — start_fsm.sh 가 sudo 뒤에서 두 번째 인자로 넘긴다 (sudo 가 환경변수를 지우므로).
#   python init_fsm.py <stand|sit|bal|damp|no-bal> <robot>
# FSM ID 는 robots/<robot>/robot.yaml fsm (G1: damp 1 / lock 4 / run 501 / sit 3).
if len(sys.argv) < 3 or sys.argv[1] not in ("stand", "sit", "bal", "damp", "no-bal"):
    print("Usage: python init_fsm.py [stand|sit|bal|damp|no-bal] <robot>")
    sys.exit(1)

os.environ["ROBOT"] = sys.argv[2]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT 미지정 / robot.yaml 없음 / enabled: false 면 여기서 종료

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

LocoClient = robot_env.loco_client_class()   # robot.yaml sdk.loco_module
F = robot_env.FSM
DAMP, LOCK, RUN, SIT = int(F["damp"]), int(F["lock"]), int(F["run"]), int(F["sit"])

def setfsmid(n):
    print(f"[setfsmid-START]: id({n})")
    res = loco_client.SetFsmId(n)
    print(f"[setfsmid-END]: Result({res})")

mode = sys.argv[1]

ChannelFactoryInitialize(0)

# 연결된 로봇 확인 (robot.yaml identity.mode_machine) — 다르면 FSM 명령을 보내지 않는다
_sub = ChannelSubscriber("rt/lowstate", hg_LowState)
_sub.Init()
_msg = None
_t_end = time.time() + 3.0
while _msg is None and time.time() < _t_end:
    _msg = _sub.Read(0.5)
if _msg is None:
    print("[init_fsm] ⚠️ rt/lowstate 수신 없음 — 로봇 확인 불가")
    if (robot_env.CFG.get("identity") or {}).get("mode_machine") is not None:
        print("[init_fsm] ❌ identity.mode_machine 이 설정된 로봇은 확인 없이 명령하지 않음")
        sys.exit(3)
else:
    _ok, _why = robot_env.check_identity(_msg)
    print(_why)
    if not _ok:
        sys.exit(3)

loco_client = LocoClient()
loco_client.Init()
loco_client.SetTimeout(10.0)

if mode == "stand":
    setfsmid(DAMP)
    time.sleep(5)
    setfsmid(LOCK)
    time.sleep(10)
    setfsmid(RUN)
elif mode == "sit":
    time.sleep(3)
    setfsmid(SIT)
elif mode == "bal":
    time.sleep(3)
    setfsmid(RUN)
elif mode == "no-bal":
    time.sleep(3)
    setfsmid(LOCK)
elif mode == "damp":
    time.sleep(3)
    setfsmid(DAMP)
