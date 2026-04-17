import os
import sys
import time
import json
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# g1_motor_high.py 파일이 같은 디렉토리에 있다고 가정합니다.
from g1_motor_high import G1JointIndex, Custom

# JSON의 motor_index를 g1_motor_high.py의 G1JointIndex Enum으로 변환하는 매핑 테이블
# 제공된 g1_motor_high.py의 G1JointIndex 클래스 정의에 맞춰 수정되었습니다.
MOTOR_ID_TO_JOINT_INDEX = {
    # Waist
    12: G1JointIndex.WaistYaw,
    13: G1JointIndex.WaistRoll,
    14: G1JointIndex.WaistPitch,
    # Left Arm
    15: G1JointIndex.LeftShoulderPitch,
    16: G1JointIndex.LeftShoulderRoll,
    17: G1JointIndex.LeftShoulderYaw,
    18: G1JointIndex.LeftElbow,
    19: G1JointIndex.LeftWristRoll,
    20: G1JointIndex.LeftWristPitch,
    21: G1JointIndex.LeftWristYaw,
    # Right Arm
    22: G1JointIndex.RightShoulderPitch,
    23: G1JointIndex.RightShoulderRoll,
    24: G1JointIndex.RightShoulderYaw,
    25: G1JointIndex.RightElbow,
    26: G1JointIndex.RightWristRoll,
    27: G1JointIndex.RightWristPitch,
    28: G1JointIndex.RightWristYaw,
}

# JSON의 방향 문자열을 로코모션 속도 벡터(vx, vy, vyaw)로 변환하는 매핑
# 값은 필요에 따라 조정할 수 있습니다. (vx: 전후, vy: 좌우, vyaw: 회전)
LOCO_DIRECTION_MAP = {
    "forward": (0.3, 0.0, 0.0),
    "backward": (-0.3, 0.0, 0.0),
    "left": (0.0, 0.2, 0.0),
    "right": (0.0, -0.2, 0.0),
    "turn_left": (0.0, 0.0, 0.4),
    "turn_right": (0.0, 0.0, -0.4),
    "stop": (0.0, 0.0, 0.0),
}


def set_motion(custom, motion_data):
    """
    파싱된 JSON 모션 데이터를 기반으로 로봇 동작(포즈 및 보행)을 실행합니다.

    Args:
        custom (Custom): 초기화된 로봇 제어 객체.
        motion_data (list): JSON 파일에서 로드된 동작 데이터 리스트.
    """
    print("🤖 JSON 파일에 정의된 모션 시퀀스를 시작합니다.")

    # JSON 파일의 각 동작 단계를 순차적으로 실행
    for i, action in enumerate(motion_data):
        duration = float(action.get("duration", 1.0))
        print(f"\n[단계 {i+1}/{len(motion_data)}] - 지속 시간: {duration}초")

        # 'pose' 동작 처리 (팔/허리 관절 제어)
        if "pose" in action and "targets" in action["pose"]:
            targets = action["pose"]["targets"]
            
            # 한 단계에 있는 모든 모터 명령을 동시에 전송
            for target in targets:
                motor_id = target.get("motor_index")
                target_deg = float(target.get("target_degree", 0.0))

                if motor_id in MOTOR_ID_TO_JOINT_INDEX:
                    custom.command_new_move(motor_id, target_deg, duration)
                else:
                    print(f"    ⚠️ 경고: 모터 ID {motor_id}에 대한 매핑 정보가 없습니다. 이 동작은 건너뜁니다.")
            
            # 포즈 변경 명령들이 모두 적용될 때까지 대기
            time.sleep(duration + 0.1)

        # 'locomotion' 동작 처리 (보행 제어)
        elif "locomotion" in action:
            direction = action["locomotion"].get("direction", "stop")
            
            if direction in LOCO_DIRECTION_MAP:
                vx, vy, vyaw = LOCO_DIRECTION_MAP[direction]
                print(vx, vy, vyaw)
                print(f"  ▶️ 이동 명령: '{direction}' 방향으로 {duration}초 동안 이동합니다.")
                
                # 이동 시작
                custom.execute_loco_command("Move", vx, vy, vyaw)
                # 지정된 시간만큼 이동
                time.sleep(duration)
                # 이동 정지
                print(f"  ▶️ 이동 정지.")
                custom.execute_loco_command("Move", 0.0, 0.0, 0.0)
                time.sleep(0.5) # 정지 후 안정을 위해 잠시 대기
            else:
                print(f"    ⚠️ 경고: 알 수 없는 이동 방향 '{direction}'입니다.")

    print("\n✅ 모든 모션 시퀀스를 완료했습니다.")


if __name__ == '__main__':
    # 1. 커맨드 라인 인자 확인
    if len(sys.argv) < 2:
        print("오류: 실행할 모션 파일 경로를 입력해주세요.")
        print("사용법: python3 main.py eth0 <json_file_path>")
        sys.exit(1)
        
    motion_file_path = sys.argv[2]

    # 2. JSON 파일 로드
    try:
        with open(motion_file_path, 'r', encoding='utf-8') as f:
            motion_data = json.load(f)
    except FileNotFoundError:
        print(f"오류: '{motion_file_path}' 파일을 찾을 수 없습니다.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"오류: '{motion_file_path}' 파일이 올바른 JSON 형식이 아닙니다.")
        sys.exit(1)

    # 3. 로봇 초기화 및 준비
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    # 로봇의 FSM(Finite State Machine) 상태를 변경하여 제어 준비
    print("로봇 상태를 설정합니다...")
    os.system(f'../g1_cmd {sys.argv[1]} --set_fsm_id=1'); time.sleep(5)
    os.system(f'../g1_cmd {sys.argv[1]} --set_fsm_id=4'); time.sleep(5)
    os.system(f'../g1_cmd {sys.argv[1]} --set_fsm_id=500'); time.sleep(5)

    # 통신 채널 초기화 (네트워크 인터페이스 지정)
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # 커스텀 제어 객체 생성 및 초기화
    custom = Custom()
    custom.Init()
    custom.Start()
    
    # 로봇이 제어 명령을 받을 준비가 될 때까지 대기
    print("로봇 제어 초기화 중... (3초)")
    time.sleep(5)
    print("초기화 완료. 명령을 시작합니다.")

    # 4. JSON 데이터에 따라 동작 실행
    set_motion(custom, motion_data)

    # 5. 모든 동작 완료 후 초기 자세로 복귀
    print("\n동작 완료. 2초 후 모든 팔/허리 관절을 초기 자세로 되돌립니다.")
    time.sleep(2)
    for n in custom.arm_joints:
        custom.command_new_move(n, 0, 1.5)
    time.sleep(1.6)

    print("프로그램이 실행 중입니다. 종료하려면 Ctrl+C를 누르세요.")
    while True:
        time.sleep(1)

