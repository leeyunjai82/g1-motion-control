#!/usr/bin/env bash
# robot_env.sh — start_*.sh / launcher.sh / activate_tv.sh 공용 (source 해서 사용)
#
#   require_robot  : ROBOT 미지정/미지원이면 실행 거부 (기본값 없음)
#   CONDA_BASE     : conda 설치 위치 (계정명 하드코딩 없음)
#   TV_PY          : tv 환경 python 절대경로 (sudo 실행용 — sudoers NOPASSWD 경로와 같아야 함)
#
# conda 위치 우선순위: $CONDA_BASE → $HOME/miniconda3 → $HOME/anaconda3 → $HOME/miniforge3 → /opt/conda

SUPPORTED_ROBOTS="g1"

require_robot() {
  local r="${ROBOT:-}"
  if [ -z "$r" ]; then
    echo "❌ ROBOT 이 지정되지 않았습니다 — 실행 거부 (기본값 없음)" >&2
    echo "   사용 예: ROBOT=g1 $0 ...   (지원: $SUPPORTED_ROBOTS)" >&2
    exit 2
  fi
  case " $SUPPORTED_ROBOTS " in
    *" $r "*) ;;
    *) echo "❌ 지원하지 않는 ROBOT='$r' — 실행 거부 (지원: $SUPPORTED_ROBOTS)" >&2; exit 2 ;;
  esac
  export ROBOT="$r"
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
