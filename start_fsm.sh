#!/bin/bash
# 자세(FSM) 전환 — 사용: ROBOT=g1 ./start_fsm.sh [stand|sit|bal|damp|no-bal]
# sudo 는 환경변수를 지우므로 ROBOT 은 init_fsm.py 의 두 번째 인자로 넘긴다.
# sudoers NOPASSWD 규칙의 python 경로는 아래 TV_PY 와 정확히 같아야 한다 (INSTALL.md 8단계).
if [ "$#" -ne 1 ] || { [ "$1" != "stand" ] && [ "$1" != "sit" ] && [ "$1" != "bal" ] && [ "$1" != "damp" ] && [ "$1" != "no-bal" ]; }; then
    echo "Usage: ROBOT=<robot> $0 [stand|sit|bal|damp|no-bal]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/robot_env.sh" || exit 1
require_robot

sudo "$TV_PY" "$SCRIPT_DIR/utils/init_fsm.py" "$1" "$ROBOT"
