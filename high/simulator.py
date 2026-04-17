"""
Unitree G1 Motion Editor - Backend Server
Version: 5.1 (Waist 3축 제어 추가)

구조:
- 팔 제어 (15~28): ArmControllerWrapper 사용
  - move_joint_smooth(): 단일 관절 보간 이동
  - move_joints_smooth(): 전체 관절 보간 이동
- 허리 제어 (전역 0~2): ArmControllerWrapper.move_waist_smooth() 사용
- 걷기: LocoClientWrapper 사용
- 손 제어: HandController 유지
"""

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import os, time, asyncio, threading
from typing import List, Optional
import numpy as np

os.system('sudo chown unitree:unitree /dev/ttyACM0')
os.system('sudo chown unitree:unitree /dev/ttyACM1')

# --- 전역 설정 ---
USE_HAND_CONTROL = True

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# --- Wrapper 임포트 ---
from lib.arm_controller_wrapper import (
    ArmControllerWrapper,
    LocoClientWrapper,
    JOINT_INFO,
    JOINT_NAMES,
    GLOBAL_TO_INTERNAL
)
print("✅ arm_controller_wrapper 로드 성공")

# --- 손 제어 라이브러리 임포트 ---
hand_left = None
hand_right = None
available_hand_motions = []

if USE_HAND_CONTROL:
    try:
        from lib.mandro import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드 성공. 사용 가능한 모션: {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리를 찾을 수 없습니다: {e}")
        USE_HAND_CONTROL = False

# --- Pydantic 모델 ---
class MotorCommand(BaseModel):
    motor_index: int  # 0~2 (허리), 15~28 (팔)
    target_degree: float
    duration: float = 1.0

class AllMotorsCommand(BaseModel):
    target_degrees: List[float]  # 14개 (팔만)
    duration: float = 1.0

class WaistCommand(BaseModel):
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.0

class LocoCommand(BaseModel):
    direction: str

class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand: str
    motion: str

class MotionFrame(BaseModel):
    duration: float
    pose: Optional[PoseData] = None
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None

class HandCommand(BaseModel):
    hand: str
    motion: str
    release: Optional[bool] = False

# --- FastAPI 앱 ---
app = FastAPI()

# 전역 인스턴스
arm_wrapper: Optional[ArmControllerWrapper] = None
loco_wrapper: Optional[LocoClientWrapper] = None

STOP_REQUESTED = False

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- 손 제어 함수 ---
def execute_hand_motion_sync(hand: str, motion: str, release: bool = False):
    """손 모션 실행 (동기)"""
    if not USE_HAND_CONTROL:
        return

    try:
        if hand == "left" and hand_left:
            if release:
                hand_left.send_release(motion)
            else:
                hand_left.send_motion(motion)
        elif hand == "right" and hand_right:
            if release:
                hand_right.send_release(motion)
            else:
                hand_right.send_motion(motion)
        elif hand == "both":
            threads = []
            if hand_left:
                t = threading.Thread(
                    target=hand_left.send_release if release else hand_left.send_motion,
                    args=(motion,)
                )
                threads.append(t)
                t.start()
            if hand_right:
                t = threading.Thread(
                    target=hand_right.send_release if release else hand_right.send_motion,
                    args=(motion,)
                )
                threads.append(t)
                t.start()
            for t in threads:
                t.join()
    except Exception as e:
        print(f"[Hand] 에러: {e}")


async def execute_hand_motion(hand: str, motion: str, release: bool = False):
    """손 모션 실행 (비동기)"""
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, execute_hand_motion_sync, hand, motion, release)


async def emergency_stop():
    """긴급 정지 - 팔 14개 + 허리 3축 동시 홈으로"""
    print("!!! 긴급 정지 실행 !!!")

    # 걷기 정지
    if loco_wrapper:
        loco_wrapper.stop()

    if arm_wrapper:
        loop = asyncio.get_running_loop()
        # 팔과 허리 동시 복귀
        tasks = [
            loop.run_in_executor(None, arm_wrapper.move_joints_smooth, [0] * 14, 1.0),
            loop.run_in_executor(None, arm_wrapper.move_waist_smooth, 0.0, 0.0, 0.0, 1.0),
        ]
        await asyncio.gather(*tasks)

    # 손 초기화
    if USE_HAND_CONTROL:
        try:
            await execute_hand_motion("both", "unfold_a", release=False)
        except:
            pass

    print("!!! 긴급 정지 완료 !!!")


@app.on_event("startup")
async def startup_event():
    global hand_left, hand_right, arm_wrapper, loco_wrapper
    print("--- FastAPI 서버 시작 ---")
    print("=" * 50)
    print("구조:")
    print("  - 허리 제어: ArmControllerWrapper.move_waist_smooth() (전역 0~2)")
    print("  - 팔 제어: ArmControllerWrapper (전역 15~28)")
    print("    - move_joint_smooth(): 단일 관절")
    print("    - move_joints_smooth(): 전체 관절")
    print("  - 걷기: LocoClientWrapper")
    print("  - 손: HandController")
    print("=" * 50)

    ChannelFactoryInitialize(0)

    try:
        loco_wrapper = LocoClientWrapper()
        print("✅ LocoClientWrapper 초기화 성공")
    except Exception as e:
        print(f"⚠️ LocoClientWrapper 초기화 실패: {e}")
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
        print(f"⚠️ ArmControllerWrapper 초기화 실패: {e}")
        arm_wrapper = None

    # 손 제어 초기화
    if USE_HAND_CONTROL:
        try:
            hand_left = HandController('/dev/ttyACM0')
            print("✅ 왼손 컨트롤러 연결 성공")
        except Exception as e:
            print(f"⚠️ 왼손 컨트롤러 연결 실패: {e}")
            hand_left = None

        try:
            hand_right = HandController('/dev/ttyACM1')
            print("✅ 오른손 컨트롤러 연결 성공")
        except Exception as e:
            print(f"⚠️ 오른손 컨트롤러 연결 실패: {e}")
            hand_right = None

    await asyncio.sleep(3)
    await emergency_stop()
    print("[시스템] 준비 완료")


@app.on_event("shutdown")
async def shutdown_event():
    """서버 종료시 정리"""
    print("--- 서버 종료 ---")
    if arm_wrapper:
        arm_wrapper.go_home()


# ==================== 손 제어 API ====================

@app.get("/hand_motions")
async def get_hand_motions():
    """손 모션 목록"""
    return {
        "enabled": USE_HAND_CONTROL,
        "left_connected": hand_left is not None,
        "right_connected": hand_right is not None,
        "motions": available_hand_motions
    }


@app.post("/set_hand")
async def set_hand(command: HandCommand):
    """손 모션 실행"""
    if not USE_HAND_CONTROL:
        return {"status": "disabled", "message": "Hand control is disabled"}

    if command.motion not in available_hand_motions:
        return {"status": "error", "message": f"Unknown motion: {command.motion}"}

    await execute_hand_motion(command.hand, command.motion, command.release)
    return {"status": "success"}


# ==================== 모터 제어 API ====================

@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    """
    단일 모터 제어
    - 허리: motor_index 0~2 (전역: WaistYaw, WaistRoll, WaistPitch)
    - 팔: motor_index 15~28 (전역) 또는 0~13 (내부)
    move_joint_smooth() 사용 - 보간 적용
    """
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    print(f"[set_motor] index={command.motor_index}, deg={command.target_degree}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_joint_smooth,
            command.motor_index,
            command.target_degree,
            command.duration
        )
        return {"status": "success"}
    except ValueError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        print(f"[set_motor Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_waist")
async def set_waist(command: WaistCommand):
    """
    허리 3축 동시 제어
    - yaw: WaistYaw (좌우 회전)
    - roll: WaistRoll (좌우 기울기)
    - pitch: WaistPitch (앞뒤 기울기)
    """
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    print(f"[set_waist] yaw={command.yaw}, roll={command.roll}, pitch={command.pitch}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_waist_smooth,
            command.yaw, command.roll, command.pitch, command.duration
        )
        return {"status": "success"}
    except Exception as e:
        print(f"[set_waist Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_all_motors")
async def set_all_motors(command: AllMotorsCommand):
    """
    전체 팔 모터 제어 (14개)
    move_joints_smooth() 사용 - 보간 적용
    """
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    if len(command.target_degrees) != 14:
        return {"status": "error", "message": "target_degrees must have 14 elements"}

    print(f"[set_all_motors] degrees={command.target_degrees}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_joints_smooth,
            command.target_degrees,
            command.duration
        )
        return {"status": "success"}
    except Exception as e:
        print(f"[set_all_motors Error] {e}")
        return {"status": "error", "message": str(e)}



# ==================== 걷기(Locomotion) API ====================

last_loco_command = {"direction": None, "timestamp": 0}
loco_lock = asyncio.Lock()

@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    """걷기 명령 (LocoClientWrapper 사용)"""
    global last_loco_command

    if not loco_wrapper:
        return {"status": "error", "message": "LocoClientWrapper not initialized"}

    async with loco_lock:
        now = time.time()
        if (command.direction == last_loco_command["direction"] and
            now - last_loco_command["timestamp"] < 0.1):
            return {"status": "skipped", "reason": "duplicate"}
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
        else:
            return {"status": "error", "message": f"Unknown direction: {command.direction}"}

        return {"status": "success"}

    except Exception as e:
        print(f"[Loco Error] {e}")
        return {"status": "error", "message": str(e)}


# ==================== 관절 정보 API ====================
@app.get("/joint_info")
async def get_joint_info():
    """관절 정보 조회 (인덱스 매핑)"""
    return {
        "status": "success",
        "joint_info": [
            {"internal": info[0], "global": info[1], "name": info[2]}
            for info in JOINT_INFO
        ],
        "joint_names": JOINT_NAMES
    }


# ==================== 모션 시퀀스 API ====================

@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    """모션 시퀀스 실행"""
    global STOP_REQUESTED
    STOP_REQUESTED = False

    print(f"[모션] 시작: {len(motion_sequence)}개 프레임")
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if STOP_REQUESTED:
            print(f"[모션] 중단: 프레임 {i+1}")
            break

        print(f"[모션] 프레임 {i+1}/{len(motion_sequence)} ({frame.duration}초)")

        # 손 모션 (비동기 시작)
        hand_task = None
        if frame.hand_motion and USE_HAND_CONTROL:
            hand_task = asyncio.create_task(
                execute_hand_motion(frame.hand_motion.hand, frame.hand_motion.motion)
            )

        # 자세(포즈) 명령 - arm_wrapper 사용
        if frame.pose and frame.pose.targets and arm_wrapper:
            # 현재 팔 타겟 읽기
            with arm_wrapper.arm_ctrl.ctrl_lock:
                current_arm_targets = np.degrees(arm_wrapper.arm_ctrl.q_target.copy())

            # 현재 허리 타겟 읽기
            try:
                with arm_wrapper.arm_ctrl.ctrl_lock:
                    curr_waist = np.degrees(
                        getattr(arm_wrapper.arm_ctrl, 'waist_q_target', np.zeros(3)).copy()
                    )
            except:
                curr_waist = np.zeros(3)
            waist_targets = curr_waist.copy()
            has_waist = False

            for target in frame.pose.targets:
                if 0 <= target.motor_index <= 2:  # 허리 전역 (WaistYaw=0, WaistRoll=1, WaistPitch=2)
                    waist_targets[target.motor_index] = target.target_degree
                    has_waist = True
                elif 15 <= target.motor_index <= 28:  # 팔 전역
                    internal_idx = GLOBAL_TO_INTERNAL[target.motor_index]
                    current_arm_targets[internal_idx] = target.target_degree
                elif 3 <= target.motor_index <= 16:  # 팔 내부 인덱스 (하위 호환)
                    current_arm_targets[target.motor_index] = target.target_degree
                # else: 범위 외 무시

            # 팔 이동 (항상)
            move_tasks = [
                loop.run_in_executor(
                    None, arm_wrapper.move_joints_smooth,
                    current_arm_targets.tolist(), frame.duration
                )
            ]
            # 허리 이동 (타겟이 있을 때만)
            if has_waist:
                move_tasks.append(
                    loop.run_in_executor(
                        None, arm_wrapper.move_waist_smooth,
                        float(waist_targets[0]), float(waist_targets[1]),
                        float(waist_targets[2]), frame.duration
                    )
                )
            await asyncio.gather(*move_tasks)

        # 걷기 명령
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
        elif not frame.pose:
            # pose가 없으면 duration만큼 대기
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
    """정지"""
    global STOP_REQUESTED
    print("[정지] 요청 수신")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_file_path = os.path.join(os.path.dirname(__file__), "simulator.html")
    if os.path.exists(html_file_path):
        return FileResponse(html_file_path)
    return HTMLResponse("<h1>Error: simulator.html not found</h1>")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
