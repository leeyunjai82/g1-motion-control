"""
로봇 양손을 특정 좌표로 이동시키는 예제
lib/arm_controller_wrapper.py 사용

사용법:
- x,y,z 입력 (예: 0.3,0.2,0.1)
- 왼손: [x, y, z], 오른손: [x, -y, z] 로 적용
- quit 입력시 종료
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
    from lib.arm_controller_wrapper import ArmControllerWrapper, parse_xyz_input
    print("[시스템] 로봇 라이브러리 로드 성공")
except ImportError as e:
    print(f"[오류] 로봇 라이브러리를 찾을 수 없습니다: {e}")
    print("lib 폴더가 상위 디렉토리에 있는지 확인해주세요.")
    sys.exit(1)


# ==========================================
# 2. 입력 검증 함수
# ==========================================

def validate_and_parse(user_input):
    """
    사용자 입력 파싱 및 범위 검증
    "0.3,0.2,0.1" -> [0.3, 0.2, 0.1] 또는 None (실패시)
    """
    coords = parse_xyz_input(user_input)
    if coords is None:
        return None

    x, y, z = coords

    # 범위 검증
    is_valid, error_msg = ArmControllerWrapper.validate_position(x, y, z)
    if not is_valid:
        print(f"  [경고] {error_msg}")
        return None

    return coords


# ==========================================
# 3. 메인 실행
# ==========================================

if __name__ == '__main__':
    print("=" * 50)
    print("로봇 양손 위치 제어 (인터랙티브)")
    print("=" * 50)

    arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
    print("\n" + "=" * 50)
    user_input = input("시작하려면 's' 입력 후 Enter: ")

    if user_input.lower() != 's':
        print("종료합니다.")
        sys.exit(0)

    arm.start()
    print("\n[시작] 속도 점진적 증가 활성화")

    print("\n" + "=" * 50)
    print("사용법:")
    print("  - x,y,z 입력 (예: 0.3,0.2,0.1)")
    print("  - 왼손: [x, +y, z], 오른손: [x, -y, z]")
    print("  - quit 입력시 종료")
    print("=" * 50)

    try:
        while True:
            print()
            user_input = input("좌표 입력 (x,y,z) 또는 quit: ").strip()

            # 종료 체크
            if user_input.lower() == 'quit':
                print("\n종료 요청...")
                break

            # 입력 파싱 및 검증
            coords = validate_and_parse(user_input)

            if coords is None:
                print("  [오류] 잘못된 입력입니다. 다시 입력해주세요.")
                print("  [예시] 0.3,0.2,0.1")
                continue

            x, y, z = coords

            print(f"  왼손: [{x}, {y}, {z}]")
            print(f"  오른손: [{x}, {-y}, {z}]")
            print("  이동 중... (2초)")

            arm.move_to([x, y, z], duration=2.0)
            print("  완료!")

    except KeyboardInterrupt:
        print("\n\n[중단] Ctrl+C 감지")

    finally:
        print("\n[종료] 홈 위치로 이동 중...")
        arm.go_home()
        print("프로그램 종료")
