"""
Unitree G1 IK Motion Editor - Backend Server
Version: 6.1 (IK + Rotation 지원)

구조:
- 팔 제어: IK (XYZ 좌표 + RPY 회전)
- 걷기: LocoClientWrapper
- 손: HandController
"""

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import os, time, asyncio, threading
from typing import List, Optional
import numpy as np
import pinocchio as pin

#os.system('sudo chown unitree:unitree /dev/ttyACM0')
#os.system('sudo chown unitree:unitree /dev/ttyACM1')

USE_HAND_CONTROL = False

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from lib.arm_controller_wrapper import (
    ArmControllerWrapper,
    LocoClientWrapper,
)
print("✅ arm_controller_wrapper 로드 성공")

# 손 제어
hand_left = None
hand_right = None
available_hand_motions = []

if USE_HAND_CONTROL:
    try:
        from lib.mandro import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드 성공. 모션: {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리 없음: {e}")
        USE_HAND_CONTROL = False


# --- 헬퍼 함수 ---

def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """RPY (degree) → Pinocchio Quaternion"""
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return pin.Quaternion(w, x, y, z).normalized()


# --- Pydantic 모델 ---

class IKPosition(BaseModel):
    """IK 위치 (XYZ)"""
    left_xyz: List[float]   # [x, y, z]
    right_xyz: List[float]  # [x, y, z]

class IKMoveCommand(BaseModel):
    """IK 이동 명령 (XYZ + RPY)"""
    left_xyz: List[float]
    right_xyz: List[float]
    left_rpy: Optional[List[float]] = None   # [roll, pitch, yaw] in degrees
    right_rpy: Optional[List[float]] = None  # [roll, pitch, yaw] in degrees
    duration: float = 1.0

class LocoCommand(BaseModel):
    direction: str

class HandCommand(BaseModel):
    hand: str
    motion: str
    release: Optional[bool] = False

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand: str
    motion: str

class MotionFrame(BaseModel):
    """모션 프레임 (IK 기반 + Rotation)"""
    duration: float
    left_xyz: Optional[List[float]] = None
    right_xyz: Optional[List[float]] = None
    left_rpy: Optional[List[float]] = None   # [roll, pitch, yaw] in degrees
    right_rpy: Optional[List[float]] = None  # [roll, pitch, yaw] in degrees
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None


# --- FastAPI ---
app = FastAPI()

arm_wrapper: Optional[ArmControllerWrapper] = None
loco_wrapper: Optional[LocoClientWrapper] = None
STOP_REQUESTED = False

# 현재 IK 위치 (초기값)
current_ik_position = {
    "left": [0.1, 0.2, 0.2],
    "right": [0.1, -0.2, 0.2]
}

# 현재 RPY 값 (초기값)
current_rpy = {
    "left": [0.0, 0.0, 0.0],
    "right": [0.0, 0.0, 0.0]
}

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- 손 제어 ---
def execute_hand_motion_sync(hand: str, motion: str, release: bool = False):
    if not USE_HAND_CONTROL:
        return
    try:
        if hand == "left" and hand_left:
            hand_left.send_release(motion) if release else hand_left.send_motion(motion)
        elif hand == "right" and hand_right:
            hand_right.send_release(motion) if release else hand_right.send_motion(motion)
        elif hand == "both":
            threads = []
            for h in [hand_left, hand_right]:
                if h:
                    t = threading.Thread(target=h.send_release if release else h.send_motion, args=(motion,))
                    threads.append(t)
                    t.start()
            for t in threads:
                t.join()
    except Exception as e:
        print(f"[Hand] 에러: {e}")


async def execute_hand_motion(hand: str, motion: str, release: bool = False):
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, execute_hand_motion_sync, hand, motion, release)


def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    """RPY를 Quaternion으로 변환하여 move_hands 호출"""
    left_rot = None
    right_rot = None
    
    if left_rpy and any(v != 0 for v in left_rpy):
        left_rot = rpy_to_quaternion(left_rpy[0], left_rpy[1], left_rpy[2])
    
    if right_rpy and any(v != 0 for v in right_rpy):
        right_rot = rpy_to_quaternion(right_rpy[0], right_rpy[1], right_rpy[2])
    
    arm_wrapper.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)


async def emergency_stop():
    """긴급 정지 - IK 홈 위치로"""
    global current_ik_position, current_rpy
    print("!!! 긴급 정지 !!!")

    if loco_wrapper:
        loco_wrapper.stop()

    if arm_wrapper:
        # 홈 위치
        home_left = [0.1, 0.2, 0.2]
        home_right = [0.1, -0.2, 0.2]
        home_rpy = [0.0, 0.0, 0.0]

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            move_hands_with_rotation,
            home_left,
            home_right,
            home_rpy,
            home_rpy,
            1.0,
            100
        )
        current_ik_position = {"left": home_left, "right": home_right}
        current_rpy = {"left": home_rpy.copy(), "right": home_rpy.copy()}

    if USE_HAND_CONTROL:
        try:
            await execute_hand_motion("both", "unfold_a", release=False)
        except:
            pass

    print("!!! 긴급 정지 완료 !!!")


@app.on_event("startup")
async def startup_event():
    global hand_left, hand_right, arm_wrapper, loco_wrapper
    print("--- IK Motion Editor 서버 시작 (v6.1 + Rotation) ---")
    print("=" * 50)
    print("  - 팔 제어: IK (XYZ 좌표 + RPY 회전)")
    print("  - 걷기: LocoClientWrapper")
    print("  - 손: HandController")
    print("=" * 50)

    ChannelFactoryInitialize(0)

    try:
        loco_wrapper = LocoClientWrapper()
        print("✅ LocoClientWrapper 초기화 성공")
    except Exception as e:
        print(f"⚠️ LocoClientWrapper 실패: {e}")
        loco_wrapper = None

    try:
        arm_wrapper = ArmControllerWrapper(
            motion_mode=True,
            simulation_mode=False,
            visualization=False,
            use_motor_control=True
        )
        arm_wrapper.start()
        print("✅ ArmControllerWrapper 초기화 성공")
    except Exception as e:
        print(f"⚠️ ArmControllerWrapper 실패: {e}")
        arm_wrapper = None

    if USE_HAND_CONTROL:
        try:
            hand_left = HandController('/dev/ttyACM0')
            print("✅ 왼손 연결")
        except:
            hand_left = None
        try:
            hand_right = HandController('/dev/ttyACM1')
            print("✅ 오른손 연결")
        except:
            hand_right = None

    await asyncio.sleep(3)
    await emergency_stop()
    print("[시스템] 준비 완료")


@app.on_event("shutdown")
async def shutdown_event():
    print("--- 서버 종료 ---")
    if arm_wrapper:
        arm_wrapper.go_home()


# ==================== 손 API ====================

@app.get("/hand_motions")
async def get_hand_motions():
    return {
        "enabled": USE_HAND_CONTROL,
        "left_connected": hand_left is not None,
        "right_connected": hand_right is not None,
        "motions": available_hand_motions
    }


@app.post("/set_hand")
async def set_hand(command: HandCommand):
    if not USE_HAND_CONTROL:
        return {"status": "disabled"}
    if command.motion not in available_hand_motions:
        return {"status": "error", "message": f"Unknown motion: {command.motion}"}
    await execute_hand_motion(command.hand, command.motion, command.release)
    return {"status": "success"}


# ==================== IK API ====================

@app.get("/ik_position")
async def get_ik_position():
    """현재 IK 위치 및 RPY 조회"""
    return {
        "status": "success",
        "left_xyz": current_ik_position["left"],
        "right_xyz": current_ik_position["right"],
        "left_rpy": current_rpy["left"],
        "right_rpy": current_rpy["right"]
    }


@app.post("/set_ik")
async def set_ik(command: IKMoveCommand):
    """IK로 양팔 이동 (RPY 포함)"""
    global current_ik_position, current_rpy

    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    if len(command.left_xyz) != 3 or len(command.right_xyz) != 3:
        return {"status": "error", "message": "XYZ must have 3 elements"}

    left_rpy = command.left_rpy if command.left_rpy else [0.0, 0.0, 0.0]
    right_rpy = command.right_rpy if command.right_rpy else [0.0, 0.0, 0.0]

    print(f"[IK] left={command.left_xyz}, right={command.right_xyz}")
    print(f"[IK] left_rpy={left_rpy}, right_rpy={right_rpy}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            move_hands_with_rotation,
            command.left_xyz,
            command.right_xyz,
            left_rpy,
            right_rpy,
            command.duration,
            100
        )

        current_ik_position = {
            "left": command.left_xyz,
            "right": command.right_xyz
        }
        current_rpy = {
            "left": left_rpy,
            "right": right_rpy
        }

        return {"status": "success"}

    except Exception as e:
        print(f"[IK Error] {e}")
        return {"status": "error", "message": str(e)}


# ==================== Locomotion API ====================

last_loco_command = {"direction": None, "timestamp": 0}
loco_lock = asyncio.Lock()

@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    global last_loco_command

    if not loco_wrapper:
        return {"status": "error", "message": "LocoClientWrapper not initialized"}

    async with loco_lock:
        now = time.time()
        if (command.direction == last_loco_command["direction"] and
            now - last_loco_command["timestamp"] < 0.1):
            return {"status": "skipped"}
        last_loco_command = {"direction": command.direction, "timestamp": now}

    try:
        loop = asyncio.get_running_loop()
        direction_map = {
            "forward": loco_wrapper.forward,
            "backward": loco_wrapper.backward,
            "left": loco_wrapper.left,
            "right": loco_wrapper.right,
            "turn_left": loco_wrapper.turn_left,
            "turn_right": loco_wrapper.turn_right,
            "stop": loco_wrapper.stop,
        }
        if command.direction in direction_map:
            await loop.run_in_executor(None, direction_map[command.direction])
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ==================== 모션 시퀀스 API ====================

@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    """IK 기반 모션 시퀀스 실행 (RPY 포함)"""
    global STOP_REQUESTED, current_ik_position, current_rpy
    STOP_REQUESTED = False

    print(f"[모션] 시작: {len(motion_sequence)}개 프레임")
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if STOP_REQUESTED:
            print(f"[모션] 중단: 프레임 {i+1}")
            break

        print(f"[모션] 프레임 {i+1}/{len(motion_sequence)} ({frame.duration}초)")

        # 손 모션
        hand_task = None
        if frame.hand_motion and USE_HAND_CONTROL:
            hand_task = asyncio.create_task(
                execute_hand_motion(frame.hand_motion.hand, frame.hand_motion.motion)
            )

        # IK 이동 (RPY 포함)
        if frame.left_xyz and frame.right_xyz and arm_wrapper:
            left_rpy = frame.left_rpy if frame.left_rpy else [0.0, 0.0, 0.0]
            right_rpy = frame.right_rpy if frame.right_rpy else [0.0, 0.0, 0.0]
            
            await loop.run_in_executor(
                None,
                move_hands_with_rotation,
                frame.left_xyz,
                frame.right_xyz,
                left_rpy,
                right_rpy,
                frame.duration,
                100
            )
            current_ik_position = {
                "left": frame.left_xyz,
                "right": frame.right_xyz
            }
            current_rpy = {
                "left": left_rpy,
                "right": right_rpy
            }

        # 걷기
        if frame.locomotion and loco_wrapper:
            direction = frame.locomotion.direction
            direction_methods = {
                "forward": loco_wrapper.forward,
                "backward": loco_wrapper.backward,
                "left": loco_wrapper.left,
                "right": loco_wrapper.right,
                "turn_left": loco_wrapper.turn_left,
                "turn_right": loco_wrapper.turn_right,
            }
            if direction in direction_methods:
                start_time = time.time()
                while time.time() - start_time < frame.duration:
                    if STOP_REQUESTED:
                        break
                    direction_methods[direction]()
                    await asyncio.sleep(0.02)
                if not STOP_REQUESTED:
                    loco_wrapper.stop()
        elif not (frame.left_xyz and frame.right_xyz):
            await asyncio.sleep(frame.duration)

        if hand_task:
            await hand_task

    if STOP_REQUESTED:
        await emergency_stop()
        STOP_REQUESTED = False
    else:
        print("[모션] 완료")
        if loco_wrapper:
            loco_wrapper.stop()

    return {"status": "success"}


@app.post("/stop_motion")
async def stop_motion():
    global STOP_REQUESTED
    print("[정지] 요청")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_file_path = os.path.join(os.path.dirname(__file__), "simulator_ik.html")
    if os.path.exists(html_file_path):
        return FileResponse(html_file_path)
    return HTMLResponse("<h1>Error: simulator_ik.html not found</h1>")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
