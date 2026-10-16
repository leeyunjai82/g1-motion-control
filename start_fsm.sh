#!/bin/bash
# 자세(FSM) 전환 — 사용: ./start_fsm.sh [stand|sit|bal|damp|no-bal]
#   ROBOT_CHECK=1 ./start_fsm.sh stand   robot.yaml enabled: false 일 때 모터 번호 확인 모드로만 (yes 확인, G1 은 true)
#     G1: stand = 1(Damping) → 5초 → 4(Lock Standing) → 10초 → 501(Walk 3DoF waist) — robot.yaml fsm
#         sit = 3 (Sit Down) — 팔을 옆으로 내리고 사람이 지지한 상태에서
# sudo 는 환경변수를 지우므로 ROBOT / check 는 init_fsm.py 인자로 넘긴다.
# sudoers NOPASSWD 규칙의 python 경로는 아래 TV_PY 와 정확히 같아야 한다 (INSTALL.md 8단계).
if [ "$#" -ne 1 ] || { [ "$1" != "stand" ] && [ "$1" != "sit" ] && [ "$1" != "bal" ] && [ "$1" != "damp" ] && [ "$1" != "no-bal" ] && ! [[ "$1" =~ ^[0-9]+$ ]]; }; then
    echo "Usage: $0 [stand|sit|bal|damp|no-bal|<FSM ID>]   (FSM ID 는 robot.yaml fsm.names 에 있는 것만)"
    echo "       ROBOT_CHECK=1 $0 [stand|damp|...]   (모터 번호 확인 모드)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/robot_env.sh" || exit 1

CHECK_ARG=""
if [ "${ROBOT_CHECK:-}" = "1" ]; then
    if [ -z "${ROBOT:-}" ] || [ ! -f "$SCRIPT_DIR/robots/${ROBOT}/robot.yaml" ]; then
        echo "❌ robots/${ROBOT}/robot.yaml 없음" >&2; exit 2
    fi
    echo "⚠️  모터 번호 확인 모드 FSM 전환 (ROBOT=$ROBOT, $1) — 로봇을 거치대에 걸고, 주변을 비우고, E-STOP 을 손에 두세요."
    read -r -p "   계속하려면 yes 입력: " ans
    [ "$ans" = "yes" ] || { echo "취소"; exit 1; }
    CHECK_ARG="check"
else
    require_robot
fi

sudo "$TV_PY" "$SCRIPT_DIR/utils/init_fsm.py" "$1" "$ROBOT" $CHECK_ARG
