#!/usr/bin/env bash
# robot_env.sh — start_*.sh / launcher.sh / activate_tv.sh 공용 (source 해서 사용)
#
#   ROBOT          : G1 전용 — 안 주면 g1 (source 할 때 export). g1 이 아니면 require_robot 이 거부
#   require_robot  : robots/<ROBOT>/robot.yaml 없음 / enabled: false 면 실행 거부
#   CONDA_BASE     : conda 설치 위치 (계정명 하드코딩 없음)
#   TV_PY          : tv 환경 python 절대경로 (sudo 실행용 — sudoers NOPASSWD 경로와 같아야 함)
#   unitree_sdk2py : tv 환경에 설치된 것 (requirements.txt 고정 커밋 f559291) 그대로 — third_party 불필요
#
# conda 위치 우선순위: $CONDA_BASE → $HOME/miniconda3 → $HOME/anaconda3 → $HOME/miniforge3 → /opt/conda

_ROBOT_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT="${ROBOT:-g1}"      # G1 전용 저장소

# 지원 로봇 = robots/<robot>/robot.yaml 이 있고 enabled: true 인 것
_enabled_robots() {
  local f
  for f in "$_ROBOT_ENV_DIR"/robots/*/robot.yaml; do
    [ -f "$f" ] && grep -Eq '^enabled:[[:space:]]*true' "$f" && basename "$(dirname "$f")"
  done | tr '\n' ' '
}

require_robot() {
  local r="${ROBOT:-}" ok
  ok="$(_enabled_robots)"
  if [ "$r" != "g1" ]; then
    echo "❌ ROBOT='$r' — 이 저장소는 G1 전용 (ROBOT 을 비우거나 g1)" >&2
    exit 2
  fi
  case " $ok " in
    *" $r "*) ;;
    *) echo "❌ 지원하지 않거나 비활성(enabled: false) ROBOT='$r' — 실행 거부 (지원: $ok)" >&2; exit 2 ;;
  esac
  export ROBOT="$r"
}

# 로그 타임스탬프 필터 — 각 줄 앞에 [YYYY-MM-DD HH:MM:SS], 줄마다 즉시 기록
#   Ubuntu 기본 awk(mawk)는 파이프 입력을 블록 단위로 모아 읽어 로그가 늦게(수 KB 단위) 써진다
#   → mawk 면 -W interactive (줄 단위 읽기/쓰기). gawk 는 그대로 줄 단위.
stamp() {
  if awk -W version 2>&1 | grep -qi mawk; then
    awk -W interactive '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
  else
    awk '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
  fi
}

_find_conda_base() {
  local c
  for c in "${CONDA_BASE:-}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" "/opt/conda"; do
    [ -n "$c" ] && [ -f "$c/bin/activate" ] && { echo "$c"; return 0; }
  done
  return 1
}

if ! CONDA_BASE="$(_find_conda_base)"; then
  echo "❌ conda 를 찾지 못했습니다 (\$HOME/miniconda3 등). CONDA_BASE=<경로> 로 지정하세요" >&2
  return 1 2>/dev/null || exit 1     # 터미널에서 source 한 경우 셸을 닫지 않게
fi
export CONDA_BASE
TV_PY="$CONDA_BASE/envs/tv/bin/python"
