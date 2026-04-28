from mandro3 import HandControler
import readline
import time
try:
    hand = HandControler('/dev/ttyACM0') # L 컨트롤러 L동글 부터 연결
    print("컨트롤러 초기화 성공")
except Exception as e:
    print(f"컨트롤러 초기화 실패: {e}")
    exit()
 
while True:
    # Example usage
    command_name = input("Enter command name,selector(left|right|both) (or 'exit'): ")
    if command_name == "exit":
        break

    if command_name == "reset":
        hand.reset()
        continue
    if command_name == "preset":
        hand.preset()
        continue
    try:
        name, selector = command_name.split(',')
    except Exception as e:
        print(e)
        continue
    hand.send_motion(name, selector)

while False:
    hand.send_motion("fold_a")
    time.sleep(1.8)
    hand.send_motion("unfold_a")
    time.sleep(1.8)
