"""
로봇 이동 제어 (인터랙티브)
lib/arm_controller_wrapper.py의 LocoClientWrapper 사용

사용법:
- w/f: 전진 (forward)
- s/b: 후진 (backward)
- a/l: 좌측 이동 (left)
- d/r: 우측 이동 (right)
- q/tl: 좌회전 (turn left)
- e/tr: 우회전 (turn right)
- x/stop: 정지
- damp: 긴급 정지 (관절 힘 빠짐)
- quit: 종료
"""
import os
import sys
import readline

# ==========================================
# 1. 경로 및 라이브러리 설정
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

try:
    from ctrl.arm_controller_wrapper import LocoClientWrapper
    print("[시스템] Locomotion 라이브러리 로드 성공")
except ImportError as e:
    print(f"[오류] 라이브러리를 찾을 수 없습니다: {e}")
    print("lib 폴더가 상위 디렉토리에 있는지 확인해주세요.")
    sys.exit(1)


# ==========================================
# 2. 명령어 처리
# ==========================================

# 명령어 매핑
COMMANDS = {
    # 전진
    'w': 'forward',
    'f': 'forward',
    'forward': 'forward',
    # 후진
    's': 'backward',
    'b': 'backward',
    'backward': 'backward',
    # 좌측 이동
    'a': 'left',
    'l': 'left',
    'left': 'left',
    # 우측 이동
    'd': 'right',
    'r': 'right',
    'right': 'right',
    # 좌회전
    'q': 'turn_left',
    'tl': 'turn_left',
    'turn_left': 'turn_left',
    # 우회전
    'e': 'turn_right',
    'tr': 'turn_right',
    'turn_right': 'turn_right',
    # 정지
    'x': 'stop',
    'stop': 'stop',
    # 긴급 정지
    'damp': 'damp',
}

COMMAND_DESCRIPTIONS = {
    'forward': '⬆️  전진',
    'backward': '⬇️  후진',
    'left': '⬅️  좌측 이동',
    'right': '➡️  우측 이동',
    'turn_left': '↩️  좌회전',
    'turn_right': '↪️  우회전',
    'stop': '⏹️  정지',
    #'damp': '🛑 긴급 정지 (Damp)',
}


def execute_command(loco, cmd, speed=0.3):
    """명령어 실행"""
    action = COMMANDS.get(cmd.lower())
    
    if action is None:
        return False, "알 수 없는 명령어"
    
    if action == 'forward':
        loco.forward(speed)
    elif action == 'backward':
        loco.backward(speed)
    elif action == 'left':
        loco.left(speed)
    elif action == 'right':
        loco.right(speed)
    elif action == 'turn_left':
        loco.turn_left(speed)
    elif action == 'turn_right':
        loco.turn_right(speed)
    elif action == 'stop':
        loco.stop()
    #elif action == 'damp':
    #    loco.damp()
    
    return True, COMMAND_DESCRIPTIONS[action]


def print_help():
    """도움말 출력"""
    print("\n" + "=" * 50)
    print("명령어 목록:")
    print("-" * 50)
    print("  w, f, forward    : 전진")
    print("  s, b, backward   : 후진")
    print("  a, l, left       : 좌측 이동")
    print("  d, r, right      : 우측 이동")
    print("  q, tl, turn_left : 좌회전")
    print("  e, tr, turn_right: 우회전")
    print("  x, stop          : 정지")
    print("  damp             : 긴급 정지 (관절 힘 빠짐)")
    print("-" * 50)
    print("  speed 0.5        : 속도 변경 (기본: 0.3)")
    print("  help, h          : 도움말")
    print("  quit, exit       : 종료")
    print("=" * 50)


# ==========================================
# 3. 메인 실행
# ==========================================

if __name__ == '__main__':
    print("=" * 50)
    print("로봇 이동 제어 (인터랙티브)")
    print("=" * 50)

    loco = LocoClientWrapper()
    speed = 0.3

    print("\n" + "=" * 50)
    user_input = input("시작하려면 's' 입력 후 Enter: ")

    if user_input.lower() != 's':
        print("종료합니다.")
        sys.exit(0)

    print_help()

    try:
        while True:
            print()
            user_input = input(f"명령어 (속도={speed}): ").strip()

            # 종료 체크
            if user_input.lower() in ['quit', 'exit']:
                print("\n종료 요청...")
                break

            # 도움말
            if user_input.lower() in ['help', 'h', '?']:
                print_help()
                continue

            # 빈 입력
            if not user_input:
                continue

            # 속도 변경
            if user_input.lower().startswith('speed'):
                parts = user_input.split()
                if len(parts) == 2:
                    try:
                        new_speed = float(parts[1])
                        if 0.1 <= new_speed <= 1.0:
                            speed = new_speed
                            print(f"  속도 변경: {speed}")
                        else:
                            print("  [오류] 속도는 0.1~1.0 범위로 입력해주세요.")
                    except ValueError:
                        print("  [오류] 올바른 숫자를 입력해주세요.")
                else:
                    print(f"  현재 속도: {speed}")
                continue

            # 명령어 실행
            success, message = execute_command(loco, user_input, speed)
            if success:
                print(f"  {message}")
            else:
                print(f"  [오류] {message}")
                print("  'help' 입력으로 명령어 목록 확인")

    except KeyboardInterrupt:
        print("\n\n[중단] Ctrl+C 감지")

    finally:
        print("\n[종료] 정지 명령 전송...")
        loco.stop()
        print("프로그램 종료")
