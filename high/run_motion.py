"""
G1 Motion Runner Server
- 관절값 포맷 (simulator.py)  → /run, /run_file
- IK 포맷     (simulator_ik.py) → /run_ik, /run_ik_file

실행: python motion_runner.py
docs: http://로봇IP:8001/docs
"""

import uvicorn
import asyncio
import json
import time
import numpy as np
from contextlib import asynccontextmanager
from typing import List, Optional
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

import os
import sys
import pinocchio as pin

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import ArmControllerWrapper, LocoClientWrapper, GLOBAL_TO_INTERNAL

# 손 제어
try:
    from ctrl.mandro3 import HandController, motions as hand_motions
    HAND_AVAILABLE = True
except ImportError:
    HAND_AVAILABLE = False


# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmControllerWrapper] = None
loco: Optional[LocoClientWrapper]    = None
hand: Optional[object]               = None

is_running = False
STOP_FLAG  = False


# ==========================================
# Pydantic 모델 - 관절값 포맷
# ==========================================
class MotorTarget(BaseModel):
    motor_index:   int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand:   str
    motion: str

class MotionFrame(BaseModel):
    duration:    float
    pose:        Optional[PoseData]              = None
    locomotion:  Optional[LocomotionData]        = None
    hand_motion: Optional[HandMotionData]        = None


# ==========================================
# Pydantic 모델 - IK 포맷
# ==========================================
class IKMotionFrame(BaseModel):
    duration:    float
    left_xyz:    Optional[List[float]]           = None
    right_xyz:   Optional[List[float]]           = None
    left_rpy:    Optional[List[float]]           = None
    right_rpy:   Optional[List[float]]           = None
    locomotion:  Optional[LocomotionData]        = None
    hand_motion: Optional[HandMotionData]        = None


# ==========================================
# 헬퍼
# ==========================================
def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll  = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw   = np.radians(yaw_deg)
    cr, sr = np.cos(roll/2),  np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2),   np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, y, z).normalized()


def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    left_rot  = rpy_to_quaternion(*left_rpy)  if left_rpy  and any(v != 0 for v in left_rpy)  else None
    right_rot = rpy_to_quaternion(*right_rpy) if right_rpy and any(v != 0 for v in right_rpy) else None
    arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)


def execute_hand_motion_sync(h: str, motion: str):
    if hand:
        hand.send_motion(motion, selector=h)


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm, loco, hand

    print("[Motion Runner] 시작")
    ChannelFactoryInitialize(0)

    try:
        loco = LocoClientWrapper()
        print("✅ Loco 초기화 성공")
    except Exception as e:
        print(f"⚠️ Loco 초기화 실패: {e}")

    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("✅ Arm 초기화 성공")
    except Exception as e:
        print(f"⚠️ Arm 초기화 실패: {e}")

    if HAND_AVAILABLE:
        try:
            hand = HandController('/dev/ttyACM0')
            print("✅ 손 초기화 성공")
        except Exception as e:
            print(f"⚠️ 손 초기화 실패: {e}")

    print("[Motion Runner] 준비 완료")

    yield

    if arm:
        arm.go_home()
    print("[Motion Runner] 종료")


# ==========================================
# FastAPI 앱
# ==========================================
app = FastAPI(
    title="G1 Motion Runner",
    description="""
모션 파일을 받아서 G1 로봇에 실행하는 서버

## 포맷 구분
- **관절값 포맷** (`simulator.py` 저장): `/run`, `/run_file`
- **IK 포맷** (`simulator_ik.py` 저장): `/run_ik`, `/run_ik_file`
""",
    version="1.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================================
# 공통 - 걷기 루프
# ==========================================
async def _run_loco(direction: str, duration: float):
    direction_map = {
        "forward":    loco.forward,
        "backward":   loco.backward,
        "left":       loco.left,
        "right":      loco.right,
        "turn_left":  loco.turn_left,
        "turn_right": loco.turn_right,
    }
    method = direction_map.get(direction)
    if method and loco:
        start = time.time()
        while time.time() - start < duration:
            if STOP_FLAG: break
            method()
            await asyncio.sleep(0.02)
        if not STOP_FLAG and loco:
            loco.stop()


# ==========================================
# 관절값 포맷 실행
# ==========================================
async def _execute_frames(frames: List[MotionFrame]):
    global is_running, STOP_FLAG
    is_running = True
    STOP_FLAG  = False
    loop = asyncio.get_running_loop()

    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG:
                print(f"[Runner] 중단: 프레임 {i+1}")
                break

            print(f"[Runner] 프레임 {i+1}/{len(frames)} ({frame.duration}s)")

            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(
                    None, execute_hand_motion_sync,
                    frame.hand_motion.hand, frame.hand_motion.motion
                )

            if frame.pose and frame.pose.targets and arm:
                with arm.arm_ctrl.ctrl_lock:
                    arm_targets = np.degrees(arm.arm_ctrl.q_target.copy())
                try:
                    with arm.arm_ctrl.ctrl_lock:
                        waist_targets = np.degrees(
                            getattr(arm.arm_ctrl, 'waist_q_target', np.zeros(3)).copy()
                        )
                except:
                    waist_targets = np.zeros(3)

                has_waist = False
                for t in frame.pose.targets:
                    if 0 <= t.motor_index <= 2:
                        waist_targets[t.motor_index] = t.target_degree
                        has_waist = True
                    elif 15 <= t.motor_index <= 28:
                        arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree

                tasks = [
                    loop.run_in_executor(
                        None, arm.move_joints_smooth,
                        arm_targets.tolist(), frame.duration
                    )
                ]
                if has_waist:
                    tasks.append(loop.run_in_executor(
                        None, arm.move_waist_smooth,
                        float(waist_targets[0]), float(waist_targets[1]),
                        float(waist_targets[2]), frame.duration
                    ))
                await asyncio.gather(*tasks)

            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)

            if hand_future:
                await hand_future

    finally:
        is_running = False
        if loco: loco.stop()
        print("[Runner] 완료")


# ==========================================
# IK 포맷 실행
# ==========================================
async def _execute_ik_frames(frames: List[IKMotionFrame]):
    global is_running, STOP_FLAG
    is_running = True
    STOP_FLAG  = False
    loop = asyncio.get_running_loop()

    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG:
                print(f"[IK Runner] 중단: 프레임 {i+1}")
                break

            print(f"[IK Runner] 프레임 {i+1}/{len(frames)} ({frame.duration}s)")

            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(
                    None, execute_hand_motion_sync,
                    frame.hand_motion.hand, frame.hand_motion.motion
                )

            if frame.left_xyz and frame.right_xyz and arm:
                left_rpy  = frame.left_rpy  or [0.0, 0.0, 0.0]
                right_rpy = frame.right_rpy or [0.0, 0.0, 0.0]
                await loop.run_in_executor(
                    None, move_hands_with_rotation,
                    frame.left_xyz, frame.right_xyz,
                    left_rpy, right_rpy, frame.duration, 100
                )

            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)

            if hand_future:
                await hand_future

    finally:
        is_running = False
        if loco: loco.stop()
        print("[IK Runner] 완료")


# ==========================================
# API 엔드포인트
# ==========================================

@app.get("/status", summary="실행 상태 확인")
async def status():
    """현재 모션 실행 중 여부와 하드웨어 연결 상태를 반환합니다."""
    return {
        "is_running": is_running,
        "arm_ready":  arm  is not None,
        "loco_ready": loco is not None,
        "hand_ready": hand is not None,
    }


# ---- 관절값 포맷 ----

@app.post("/run", summary="관절값 모션 실행 (JSON body)",
          description="simulator.py에서 저장한 모션 JSON을 body로 전송합니다.")
async def run_motion(frames: List[MotionFrame]):
    """
    ```bash
    curl -X POST http://로봇IP:8001/run \\
      -H "Content-Type: application/json" -d @motion.json
    ```
    """
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames)}


@app.post("/run_file", summary="관절값 모션 파일 업로드 후 실행",
          description="simulator.py에서 저장한 .json 파일을 업로드합니다.")
async def run_motion_file(file: UploadFile = File(...)):
    """
    ```bash
    curl -X POST http://로봇IP:8001/run_file -F "file=@motion.json"
    ```
    """
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    try:
        data   = json.loads(await file.read())
        frames = [MotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}


# ---- IK 포맷 ----

@app.post("/run_ik", summary="IK 모션 실행 (JSON body)",
          description="simulator_ik.py에서 저장한 모션 JSON을 body로 전송합니다.")
async def run_ik_motion(frames: List[IKMotionFrame]):
    """
    ```bash
    curl -X POST http://로봇IP:8001/run_ik \\
      -H "Content-Type: application/json" -d @ik_motion.json
    ```
    """
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames)}


@app.post("/run_ik_file", summary="IK 모션 파일 업로드 후 실행",
          description="simulator_ik.py에서 저장한 .json 파일을 업로드합니다.")
async def run_ik_motion_file(file: UploadFile = File(...)):
    """
    ```bash
    curl -X POST http://로봇IP:8001/run_ik_file -F "file=@ik_motion.json"
    ```
    """
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    try:
        data   = json.loads(await file.read())
        frames = [IKMotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}


# ---- 공통 제어 ----

@app.post("/stop", summary="실행 중인 모션 정지")
async def stop_motion():
    """실행 중인 모션을 중단하고 홈 포지션으로 복귀합니다."""
    global STOP_FLAG
    STOP_FLAG = True
    if loco: loco.stop()
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 1.0),
            loop.run_in_executor(None, arm.move_waist_smooth,  0.0, 0.0, 0.0, 1.0),
        )
    return {"status": "stopped"}


@app.post("/home", summary="홈 포지션으로 이동")
async def go_home():
    """즉시 홈 포지션으로 이동합니다."""
    global STOP_FLAG
    STOP_FLAG = True
    await asyncio.sleep(0.1)
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 2.0),
            loop.run_in_executor(None, arm.move_waist_smooth,  0.0, 0.0, 0.0, 2.0),
        )
    return {"status": "home"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
