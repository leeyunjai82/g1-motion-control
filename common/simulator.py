"""
simulator.py — H2 Motion Editor 통합본 (관절 + IK)
Version: 7.2

simulator.py(관절 편집기 v5.3) + simulator_ik.py(IK 편집기 v6.3) 통합.

구조 변경 (arm_server 분리 반영):
  - 팔/허리 지령은 arm_server(50022) 경유 (ctrl/arm_http.ArmHttpClient).
    → robot_server 와 동시에 떠도 arm_sdk 이중 송신이 없다.
    → 단독 사용 시에도 arm_server 를 먼저 띄워야 한다:
         python arm_server.py   →   python simulator.py
  - 보행 없음 (H2: robot.yaml features.locomotion false) — 이 편집기는 걷기 명령
    (LocoClient Move/StopMove 등)을 보내지 않는다. 이동 패드·/set_loco_motion·
    모션의 이동 프레임 생성은 v7.2 에서 제거.
  - 손: HandController (단일 동글) 직접

UI:
  /   통합 simulator.html (좌측 패널 Joint/IK 모드 토글, 타임라인 공용)

엔드포인트 = 두 편집기의 합집합:
  공통: /hand_motions /set_hand /set_motion /stop_motion /go_home
  관절: /set_motor /set_waist /set_all_motors /joint_info
  IK  : /set_ik /ik_position
  확인: /check (joint_check — 모터 번호 확인)
  /set_motion 은 프레임에 pose(관절)와 left_xyz/right_xyz(IK)가 섞여 있어도
  프레임별로 자동 판별해 실행한다.
  예전(G1) 모션 파일의 locomotion(걷기) 프레임은 실행하지 않고 경고를 출력한 뒤
  건너뛴다 (같은 프레임의 팔/손 동작은 실행, 걷기만 있는 프레임은 대기 없이 통과).

안전 변경:
  - /stop_motion(긴급 정지)은 이동하지 않는다 — 보간 중단 +
    현재 자세 동결(freeze) + 손 펴기. (구버전은 관절 0도/홈으로 '이동'했는데,
    0도는 앞으로 나란히라 정지 중 팔이 크게 움직였음)
  - 기동 시 자동 홈 이동/허리 리셋 없음 (arm_server 가 자세를 이미 유지 중)
  - 자세 복귀가 필요하면 POST /go_home (IK 홈 자세로 이동)
"""

import os
import asyncio
from typing import Any, List, Optional

import uvicorn
import numpy as np
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager

import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import robot_env   # ROBOT 미지정/미지원이면 여기서 종료

USE_HAND_CONTROL = True

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import (JOINT_INFO, JOINT_NAMES,
                                         GLOBAL_TO_INTERNAL)
from ctrl.arm_http import ArmHttpClient

hand_controller = None
available_hand_motions = []
if USE_HAND_CONTROL:
    try:
        from ctrl.mandro3 import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드. 모션 {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리 없음: {e}")
        USE_HAND_CONTROL = False


# ==========================================
# Pydantic (두 편집기 합집합)
# ==========================================
class MotorCommand(BaseModel):
    motor_index: int
    target_degree: float
    duration: float = 1.0

class AllMotorsCommand(BaseModel):
    target_degrees: List[float]     # 14개 (팔만)
    duration: float = 1.0

class WaistCommand(BaseModel):
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.0

class HandCommand(BaseModel):
    hand: str                        # left | right | both
    motion: str
    release: bool = False

class IKMoveCommand(BaseModel):
    left_xyz:  List[float]
    right_xyz: List[float]
    left_rpy:  Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    duration:  float = 1.0

class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class HandMotionData(BaseModel):
    hand: str
    motion: str

class MotionFrame(BaseModel):
    """관절(pose)과 IK(left_xyz/right_xyz) 프레임 공용 — 프레임별 자동 판별.

    locomotion: 예전(G1) 모션 파일 호환용으로 받기만 한다 (형식 무관 — 422 로 막지 않음).
                H2 는 보행이 없으므로 실행하지 않고 경고 후 건너뛴다.
    """
    duration: float
    pose: Optional[PoseData] = None
    left_xyz:  Optional[List[float]] = None
    right_xyz: Optional[List[float]] = None
    left_rpy:  Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    locomotion: Optional[Any] = None
    hand_motion: Optional[HandMotionData] = None


# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmHttpClient]     = None
STOP_REQUESTED = False

current_ik_position = {"left": [0.1, 0.2, 0.2], "right": [0.1, -0.2, 0.2]}
current_rpy         = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}

IK_HOME_LEFT  = [0.1,  0.2, 0.2]
IK_HOME_RIGHT = [0.1, -0.2, 0.2]


# ==========================================
# 손 제어
# ==========================================
def execute_hand_motion_sync(hand: str, motion: str, release: bool = False):
    if not USE_HAND_CONTROL or not hand_controller:
        return
    try:
        if release:
            hand_controller.send_release(selector=hand)
        else:
            hand_controller.send_motion(motion, selector=hand)
    except Exception as e:
        print(f"[Hand] 에러: {e}")


async def execute_hand_motion(hand: str, motion: str, release: bool = False):
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, execute_hand_motion_sync, hand, motion, release)


# ==========================================
# 긴급 정지 — 이동 없이 그 자리 동결
# ==========================================
async def emergency_stop():
    print("!!! 긴급 정지 (동결) !!!")
    if arm:
        loop = asyncio.get_running_loop()
        try:
            await loop.run_in_executor(None, arm.stop_motion)
            await loop.run_in_executor(None, arm.freeze)
        except Exception as e:
            print(f"[정지] arm 동결 실패: {e}")
    if USE_HAND_CONTROL and hand_controller:
        try:
            hand_controller.send_motion('unfold_a', selector='both')
        except Exception as e:
            print(f"[Hand 정지] 에러: {e}")
    print("!!! 긴급 정지 완료 !!!")


def _move_ik(left_xyz, right_xyz, left_rpy, right_rpy, duration):
    """arm_server /hands 호출 (rpy는 deg). 완료까지 블로킹."""
    import json as _json
    import urllib.request as _u
    body = {"left_xyz": [float(v) for v in left_xyz],
            "right_xyz": [float(v) for v in right_xyz],
            "duration": float(duration), "frequency": 100}
    if left_rpy and any(v != 0 for v in left_rpy):
        body["left_rpy"] = [float(v) for v in left_rpy]
    if right_rpy and any(v != 0 for v in right_rpy):
        body["right_rpy"] = [float(v) for v in right_rpy]
    req = _u.Request("http://localhost:50022/hands",
                     data=_json.dumps(body).encode(),
                     headers={"Content-Type": "application/json"}, method="POST")
    with _u.urlopen(req, timeout=duration + 30.0) as r:
        return _json.loads(r.read())


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global hand_controller, arm
    print(f"--- {robot_env.ROBOT.upper()} Motion Editor 통합본 (v7.2) ---")
    print("  팔/허리: arm_server(50022) 경유 · 보행 없음 · 손: 단일 동글")

    robot_env.dds_init()   # /check (joint_check) 의 rt/lowstate 구독용

    try:
        arm = ArmHttpClient()          # arm_server 가 먼저 떠 있어야 함
    except Exception as e:
        print(f"⚠️ arm_server 연결 실패: {e}")
        print("   → 먼저 실행: python arm_server.py")
        arm = None

    if USE_HAND_CONTROL:
        try:
            hand_controller = HandController('/dev/ttyACM0')
            print("✅ 손 컨트롤러 연결 (/dev/ttyACM0)")
        except Exception as e:
            print(f"⚠️ 손 컨트롤러 실패: {e}")

    print(f"[시스템] 준비 완료  http://localhost:8000/   모터 번호 확인: http://localhost:8000/check  "
          f"({'가상 ROBOT_SIM' if robot_env.SIM else '실기'})")
    yield
    print("--- 서버 종료 (자세 유지 — arm_server 관리) ---")


app = FastAPI(title="H2 Motion Editor (통합)", version="7.2", lifespan=lifespan)

# 모터 번호 확인 화면 (/check) — 가상/실기 모두
from joint_check import router as joint_check_router
app.include_router(joint_check_router)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True,
                   allow_methods=["*"], allow_headers=["*"])


# ==========================================
# 손 API
# ==========================================
@app.get("/hand_motions")
async def get_hand_motions():
    connected = hand_controller is not None
    return {"enabled": USE_HAND_CONTROL, "left_connected": connected,
            "right_connected": connected, "single_dongle": True,
            "motions": available_hand_motions}


@app.post("/set_hand")
async def set_hand(command: HandCommand):
    if not USE_HAND_CONTROL:
        return {"status": "disabled"}
    if not hand_controller:
        return {"status": "error", "message": "Hand controller not connected"}
    if command.motion not in available_hand_motions:
        return {"status": "error", "message": f"Unknown motion: {command.motion}"}
    await execute_hand_motion(command.hand, command.motion, command.release)
    return {"status": "success"}


# ==========================================
# 관절 API (구 simulator.py)
# ==========================================
@app.get("/joint_info")
async def get_joint_info():
    return {"status": "success",
            "joint_info": [{"internal": i[0], "global": i[1], "name": i[2]}
                           for i in JOINT_INFO],
            "joint_names": JOINT_NAMES}


@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    """단일 모터 (허리 0~2 / 팔 15~28). 현재 타겟 기준 한 축만 변경."""
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    try:
        loop = asyncio.get_running_loop()
        idx, deg, dur = command.motor_index, command.target_degree, command.duration
        if 0 <= idx <= 2:
            waist = np.degrees(arm.arm_ctrl.waist_q_target).tolist()
            waist[idx] = deg
            await loop.run_in_executor(None, lambda: arm.move_waist_smooth(
                yaw=waist[0], roll=waist[1], pitch=waist[2], duration=dur))
        elif 15 <= idx <= 28:
            targets = np.degrees(arm.arm_ctrl.q_target).tolist()
            targets[GLOBAL_TO_INTERNAL[idx]] = deg
            await loop.run_in_executor(None, arm.move_joints_smooth, targets, dur)
        else:
            return {"status": "error", "message": f"invalid index: {idx}"}
        return {"status": "success"}
    except Exception as e:
        print(f"[set_motor Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_waist")
async def set_waist(command: WaistCommand):
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: arm.move_waist_smooth(
            yaw=command.yaw, roll=command.roll,
            pitch=command.pitch, duration=command.duration))
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/set_all_motors")
async def set_all_motors(command: AllMotorsCommand):
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    if len(command.target_degrees) != 14:
        return {"status": "error", "message": "target_degrees must have 14 elements"}
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, arm.move_joints_smooth,
                                   command.target_degrees, command.duration)
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ==========================================
# IK API (구 simulator_ik.py)
# ==========================================
@app.get("/ik_position")
async def get_ik_position():
    return {"status": "success",
            "left_xyz": current_ik_position["left"],
            "right_xyz": current_ik_position["right"],
            "left_rpy": current_rpy["left"],
            "right_rpy": current_rpy["right"]}


@app.post("/set_ik")
async def set_ik(command: IKMoveCommand):
    global current_ik_position, current_rpy
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    if len(command.left_xyz) != 3 or len(command.right_xyz) != 3:
        return {"status": "error", "message": "XYZ must have 3 elements"}
    left_rpy = command.left_rpy or [0.0, 0.0, 0.0]
    right_rpy = command.right_rpy or [0.0, 0.0, 0.0]
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, _move_ik, command.left_xyz,
                                   command.right_xyz, left_rpy, right_rpy,
                                   command.duration)
        current_ik_position = {"left": command.left_xyz, "right": command.right_xyz}
        current_rpy = {"left": left_rpy, "right": right_rpy}
        return {"status": "success"}
    except Exception as e:
        print(f"[IK Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/go_home")
async def go_home():
    """IK 홈 자세로 이동 (구 긴급정지의 '홈 복귀'를 명시적 동작으로 분리)."""
    global current_ik_position, current_rpy
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, _move_ik, IK_HOME_LEFT, IK_HOME_RIGHT,
                               [0, 0, 0], [0, 0, 0], 1.5)
    current_ik_position = {"left": IK_HOME_LEFT, "right": IK_HOME_RIGHT}
    current_rpy = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}
    return {"status": "success"}


# ==========================================
# 모션 시퀀스 — 관절/IK 프레임 자동 판별
#   보행 없음 (H2) — locomotion 프레임은 경고 후 건너뜀
# ==========================================
@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    global STOP_REQUESTED, current_ik_position, current_rpy
    STOP_REQUESTED = False
    print(f"[모션] 시작: {len(motion_sequence)}개 프레임")
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if STOP_REQUESTED:
            print(f"[모션] 중단: 프레임 {i+1}")
            break
        has_body = bool((frame.left_xyz and frame.right_xyz)
                        or (frame.pose and frame.pose.targets) or frame.hand_motion)
        if frame.locomotion is not None:
            # 예전(G1) 모션 파일의 걷기 프레임 — H2 는 보행 없음. 걷기 명령은 절대 보내지 않는다.
            if not has_body:
                print(f"[모션] ⚠️ 프레임 {i+1}/{len(motion_sequence)}: 걷기(locomotion) 프레임 "
                      f"{frame.locomotion!r} — 보행 없는 로봇이라 건너뜀")
                continue
            print(f"[모션] ⚠️ 프레임 {i+1}/{len(motion_sequence)}: 걷기(locomotion) "
                  f"{frame.locomotion!r} 무시 — 팔/손 동작만 실행")

        print(f"[모션] 프레임 {i+1}/{len(motion_sequence)} ({frame.duration}초)")

        hand_future = None
        if frame.hand_motion and USE_HAND_CONTROL and hand_controller:
            hand_future = loop.run_in_executor(
                None, execute_hand_motion_sync,
                frame.hand_motion.hand, frame.hand_motion.motion, False)

        did_arm = False
        # --- IK 프레임 ---
        if frame.left_xyz and frame.right_xyz and arm:
            left_rpy = frame.left_rpy or [0.0, 0.0, 0.0]
            right_rpy = frame.right_rpy or [0.0, 0.0, 0.0]
            await loop.run_in_executor(None, _move_ik, frame.left_xyz,
                                       frame.right_xyz, left_rpy, right_rpy,
                                       frame.duration)
            current_ik_position = {"left": frame.left_xyz, "right": frame.right_xyz}
            current_rpy = {"left": left_rpy, "right": right_rpy}
            did_arm = True

        # --- 관절 프레임 ---
        elif frame.pose and frame.pose.targets and arm:
            arm_targets = np.degrees(arm.arm_ctrl.q_target).tolist()
            waist_targets = np.degrees(arm.arm_ctrl.waist_q_target).tolist()
            has_waist = False
            for t in frame.pose.targets:
                if 0 <= t.motor_index <= 2:
                    waist_targets[t.motor_index] = t.target_degree
                    has_waist = True
                elif 15 <= t.motor_index <= 28:
                    arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree
                elif 3 <= t.motor_index <= 16:
                    # 구버전 호환 (내부 인덱스로 저장된 모션 파일)
                    arm_targets[t.motor_index] = t.target_degree
            tasks = [loop.run_in_executor(None, arm.move_joints_smooth,
                                          arm_targets, frame.duration)]
            if has_waist:
                tasks.append(loop.run_in_executor(
                    None, lambda: arm.move_waist_smooth(
                        yaw=waist_targets[0], roll=waist_targets[1],
                        pitch=waist_targets[2], duration=frame.duration)))
            await asyncio.gather(*tasks)
            did_arm = True

        if not did_arm:
            await asyncio.sleep(frame.duration)

        if hand_future:
            await hand_future

    if STOP_REQUESTED:
        await emergency_stop()
        STOP_REQUESTED = False
    else:
        print("[모션] 완료")
    return {"status": "success"}


@app.post("/stop_motion")
async def stop_motion():
    global STOP_REQUESTED
    print("[정지] 요청")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


# ==========================================
# UI — 통합 simulator.html 단일 파일
# ==========================================
@app.get("/i18n.js", include_in_schema=False)
async def i18n_js():
    return FileResponse(os.path.join(current_dir, "i18n.js"), media_type="application/javascript")


def _joint_limits_js():
    """에디터 슬라이더 한계 = H2 URDF [deg] — 팔은 모터 슬롯 번호(15–28), 허리는 0 yaw / 1 roll / 2 pitch.
    simulator.html 의 JOINT_LIMITS(예전 G1 값)를 페이지를 내보낼 때 이것으로 바꾼다. 못 읽으면 None (html 값 그대로)."""
    try:
        import json
        import re as _re
        import pinocchio as pin
        m = pin.buildModelFromUrdf(robot_env.URDF_PATH)
        name_of = {int(v): k for k, v in robot_env.JOINTS["map"].items()}

        def lim(slot):
            q = m.joints[m.getJointId(name_of[int(slot)])].idx_q
            return [round(float(np.degrees(m.lowerPositionLimit[q])), 1), round(float(np.degrees(m.upperPositionLimit[q])), 1)]
        out = {i: lim(slot) for i, slot in enumerate(robot_env.JOINTS["waist"])}        # 0 yaw, 1 roll, 2 pitch
        out.update({int(slot): lim(slot) for slot in robot_env.JOINTS["arm"]})
        return "const JOINT_LIMITS = " + json.dumps(out) + ";", _re
    except Exception as e:      # noqa: BLE001
        print(f"[simulator] ⚠️ URDF 관절 한계 못 읽음 — html 기본값 사용: {e}")
        return None, None


_JL = _joint_limits_js()


@app.get("/", response_class=HTMLResponse)
async def read_root():
    p = os.path.join(current_dir, "simulator.html")
    if not os.path.exists(p):
        return HTMLResponse("simulator.html 없음")
    html = open(p, encoding="utf-8").read()
    js, _re = _JL
    if js:
        html, n = _re.subn(r"const JOINT_LIMITS = \{[^}]*\};", lambda _m: js, html, count=1)
        if n != 1:
            print("[simulator] ⚠️ simulator.html 에서 JOINT_LIMITS 를 못 찾음 — 슬라이더 한계는 html 값")
    return HTMLResponse(html)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
