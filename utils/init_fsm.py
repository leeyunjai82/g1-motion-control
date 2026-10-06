import os
import sys
import time

# 로봇 선택 — start_fsm.sh 가 sudo 뒤에서 두 번째 인자로 넘긴다 (sudo 가 환경변수를 지우므로).
#   python init_fsm.py <stand|sit|bal|damp|no-bal> <robot>
# FSM ID(1 → 4 → 501 등)는 G1 기준값이다. 다른 로봇은 확인 전까지 실행 거부.
if len(sys.argv) < 3 or sys.argv[1] not in ("stand", "sit", "bal", "damp", "no-bal"):
    print("Usage: python init_fsm.py [stand|sit|bal|damp|no-bal] <robot>")
    sys.exit(1)

os.environ["ROBOT"] = sys.argv[2]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT 미지정/미지원이면 여기서 종료

if robot_env.ROBOT != "g1":
    print(f"[init_fsm] ❌ ROBOT={robot_env.ROBOT} — FSM ID 가 G1 기준이라 실행 거부")
    sys.exit(2)

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

def setfsmid(n):
    print(f"[setfsmid-START]: id({n})")
    res = loco_client.SetFsmId(n)
    print(f"[setfsmid-END]: Result({res})")

mode = sys.argv[1]

ChannelFactoryInitialize(0)
loco_client = LocoClient()
loco_client.Init()
loco_client.SetTimeout(10.0)

if mode == "stand":
    setfsmid(1)
    time.sleep(5)
    setfsmid(4)
    time.sleep(10)
    setfsmid(501)
elif mode == "sit":
    time.sleep(3)
    setfsmid(3)
elif mode == "bal":
    time.sleep(3)
    setfsmid(501)
elif mode == "no-bal":
    time.sleep(3)
    setfsmid(4)
elif mode == "damp":
    time.sleep(3)
    setfsmid(1)
