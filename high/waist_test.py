#!/usr/bin/env python3
import sys
import time
from ctrl.arm_controller_wrapper import ArmControllerWrapper

def print_waist_menu():
    print("\n" + "="*50)
    print("      G1 Waist Control Dedicated CLI")
    print("="*50)
    print(" [명령어 형식]")
    print("  w yaw,roll,pitch : 허리 3축 동시 제어 (도 단위)")
    print("                     예: w 20,0,10 (좌회전 20, 숙이기 10)")
    print("\n [개별 축 제어]")
    print("  y deg  : Yaw 제어 (좌+/우-)")
    print("  r deg  : Roll 제어 (좌+/우-)")
    print("  p deg  : Pitch 제어 (앞+/뒤-)")
    print("\n [기타]")
    print("  0      : 허리 정중앙 복귀 (All 0)")
    print("  h      : 도움말 표시")
    print("  q      : 종료")
    print("="*50)

def main():
    is_sim = "--sim" in sys.argv
    print(f"🤖 허리 제어 모듈 초기화 중... (Sim: {is_sim})")

    try:
        # 허리 제어를 위해 ArmControllerWrapper 활용
        # (허리 모터도 해당 컨트롤러의 제어 범위에 포함됨)
        bot = ArmControllerWrapper(motion_mode=not is_sim, simulation_mode=is_sim)
        bot.start()
        
        # 현재 상태 저장 (개별 축 제어용)
        curr_y, curr_r, curr_p = 0.0, 0.0, 0.0
        
        print("✅ 연결 완료.")
        print_waist_menu()

        while True:
            cmd_input = input("\nWaist >> ").strip().lower()
            
            if not cmd_input: continue
            if cmd_input == 'q': break
            if cmd_input == 'h':
                print_waist_menu()
                continue
            if cmd_input == '0':
                print("🔄 허리 정중앙 복귀...")
                curr_y, curr_r, curr_p = 0.0, 0.0, 0.0
                bot.move_waist_smooth(0, 0, 0, duration=1.5)
                continue

            try:
                # 1. 3축 동시 제어 (w 10,0,5)
                if cmd_input.startswith('w '):
                    vals = [float(v) for v in cmd_input[2:].split(',')]
                    if len(vals) == 3:
                        curr_y, curr_r, curr_p = vals
                        bot.move_waist_smooth(curr_y, curr_r, curr_p)
                        print(f"🎯 이동: Yaw={curr_y}°, Roll={curr_r}°, Pitch={curr_p}°")
                
                # 2. 개별 축 제어 (y 10 / r 5 / p -5)
                elif len(cmd_input.split()) == 2:
                    axis, val = cmd_input.split()
                    val = float(val)
                    if axis == 'y': curr_y = val
                    elif axis == 'r': curr_r = val
                    elif axis == 'p': curr_p = val
                    else:
                        print("⚠️ 잘못된 축 이름입니다 (y, r, p 중 선택)")
                        continue
                    
                    bot.move_waist_smooth(curr_y, curr_r, curr_p, duration=1.0)
                    print(f"🎯 {axis.upper()} 조정 완료 -> 현재: [{curr_y}, {curr_r}, {curr_p}]")

            except ValueError:
                print("⚠️ 숫자 형식이 잘못되었습니다. (예: w 10,0,5 또는 p 15)")

    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        if 'bot' in locals():
            bot.stop_motion()
        print("시스템 종료.")

if __name__ == "__main__":
    main()
