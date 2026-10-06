#!/bin/bash
# tv conda 환경 활성화 — 계정명 하드코딩 없음 (conda 위치는 robot_env.sh 가 찾는다)
#   source activate_tv.sh
_ACT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$_ACT_DIR/robot_env.sh" || return 1 2>/dev/null || exit 1
source "$CONDA_BASE/bin/activate" || return 1 2>/dev/null || exit 1
conda activate tv || return 1 2>/dev/null || exit 1
