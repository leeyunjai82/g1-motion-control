#!/bin/bash
# tv conda 환경 활성화 — 계정명 하드코딩 없음 (conda 위치는 robot_env.sh 가 찾는다)
#   source activate_tv.sh
# 주의: 인자를 명시해서 activate 를 부른다. 인자 없이 source 하면 호출한 스크립트의 인자($1, 예: real-cam)가
#       그대로 conda activate 로 넘어가 "Could not find conda environment: real-cam" 오류가 난다.
_ACT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$_ACT_DIR/robot_env.sh" || return 1 2>/dev/null || exit 1
source "$CONDA_BASE/bin/activate" tv || return 1 2>/dev/null || exit 1
