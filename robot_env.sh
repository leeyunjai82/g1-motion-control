#!/usr/bin/env bash
# robot_env.sh — start_*.sh / launcher.sh / activate_tv.sh 공용 (source 해서 사용)
#
#   ROBOT          : G1 전용 — 안 주면 g1 (source 할 때 export). g1 이 아니면 require_robot 이 거부
#   require_robot  : robots/<ROBOT>/robot.yaml 없음 / enabled: false 면 실행 거부
#   CONDA_BASE     : conda 설치 위치 (계정명 하드코딩 없음)
#   TV_PY          : tv 환경 python 절대경로 (sudo 실행용 — sudoers NOPASSWD 경로와 같아야 함)
#   unitree_sdk2py : tv 환경에 설치된 것 (requirements.txt 고정 커밋 f559291) 그대로 — third_party 불필요
#   DDS_IFACE      : DDS 네트워크 인터페이스 (예: enp3s0). 비우면 자동 선택 (기존과 같음) — common/robot_env.py dds_init
#                    sudo 로 도는 init_fsm.py / run_launcher.py 에는 start_fsm.sh / launcher.sh 가 인자(iface=<이름>)로 넘긴다
#   with_pdeath    : 자식 프로세스를 setpriv --pdeathsig KILL 로 감싸 실행 (이 스크립트가 죽으면 같이 죽게)
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

# DDS 인터페이스 — 지정했는데 없는 이름이면 바로 거부 (오타면 로봇 데이터가 조용히 안 들어옴)
export DDS_IFACE="${DDS_IFACE:-}"
if [ -n "$DDS_IFACE" ] && [ ! -d "/sys/class/net/$DDS_IFACE" ]; then
  echo "❌ DDS_IFACE='$DDS_IFACE' — 이 PC 에 없는 네트워크 인터페이스 (ip -br link 로 확인, 비우면 자동 선택)" >&2
  return 1 2>/dev/null || exit 1
fi

# 자식 프로세스 실행 — setpriv --pdeathsig KILL (util-linux 2.33+, Ubuntu 24.04 기본 포함)
#   부모 스크립트가 SIGKILL 등으로 정리(trap cleanup) 없이 죽어도 서버가 고아로 남지 않게 한다.
#   정상 종료(Ctrl+C / SIGTERM)는 trap cleanup 이 TERM → 대기 → KILL 로 먼저 끝내므로 영향 없음
#   (arm_server 가 weight 를 반납할 시간은 그대로). setpriv 가 python 으로 exec 하므로 $! · pgrep 패턴도 그대로.
#   사용: with_pdeath python -u arm_server.py > >(stamp >> "$LOG") 2>&1 &
#   ※ 반드시 & (백그라운드 서브셸) 로만 — exec 로 서브셸을 python 으로 바꿔야 $! 가 python PID 가 된다.
#     exec 없이 감싸면 $! 가 서브셸이라 TERM 이 python 에 안 가고, 서브셸이 죽는 순간 pdeathsig 로 python 이 KILL 된다.
if command -v setpriv >/dev/null 2>&1 && setpriv --help 2>&1 | grep -q -- --pdeathsig; then
  _PDEATH=(setpriv --pdeathsig KILL --)
else
  echo "⚠️ setpriv --pdeathsig 없음 (util-linux) — 스크립트가 강제 종료되면 서버가 남을 수 있음" >&2
  _PDEATH=()
fi
with_pdeath() {
  if [ "${BASHPID:-$$}" = "$$" ]; then     # 메인 셸에서 부르면 exec 가 스크립트 자체를 바꿔 버림
    echo "❌ with_pdeath 는 & 로만 실행: with_pdeath $* &" >&2
    return 1
  fi
  exec "${_PDEATH[@]}" "$@"
}
