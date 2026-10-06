"""
robot_env.py — 실행 대상 로봇 선택 (ROBOT 환경변수)

  ROBOT=g1 python robot_server.py

  · ROBOT 미지정 / 미지원 값이면 import 시점에 바로 종료한다 (기본값 없음).
    다른 로봇을 연결해 둔 채 잘못된 설정으로 지령을 보내는 사고를 막기 위함.
  · 로봇별 파일(URDF, 메시, 모션 JSON, IK 모델 캐시)은 robots/<ROBOT>/ 에 둔다.
  · 지원 목록에 없는 로봇(h2 등)은 robots/<robot>/ 가 준비되고 실기 확인이 끝난 뒤 추가한다.
"""
import os
import sys

REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMON_DIR = os.path.join(REPO_ROOT, "common")
ROBOTS_DIR = os.path.join(REPO_ROOT, "robots")

# 로봇별 파일 이름 (경로만 옮김 — 값/파일 내용은 기존 high/ 와 동일)
_ROBOT_FILES = {
    "g1": {
        "urdf":     "g1_29dof_rev_1_0.urdf",
        "ik_cache": "g1_29_model_cache.pkl",
    },
}
SUPPORTED = tuple(_ROBOT_FILES.keys())


def _fail(msg):
    print(f"[robot_env] ❌ {msg}", file=sys.stderr)
    print(f"[robot_env]    사용 예: ROBOT=g1 ./start_robot.sh   (지원: {', '.join(SUPPORTED)})",
          file=sys.stderr)
    sys.exit(2)


ROBOT = os.environ.get("ROBOT", "").strip().lower()
if not ROBOT:
    _fail("ROBOT 환경변수가 없습니다 — 실행 거부 (기본값 없음)")
if ROBOT not in _ROBOT_FILES:
    _fail(f"지원하지 않는 ROBOT='{ROBOT}' — 실행 거부")

ROBOT_DIR = os.path.join(ROBOTS_DIR, ROBOT)
if not os.path.isdir(ROBOT_DIR):
    _fail(f"로봇 폴더가 없습니다: {ROBOT_DIR}")

URDF_PATH     = os.path.join(ROBOT_DIR, _ROBOT_FILES[ROBOT]["urdf"])
MESH_DIR      = os.path.join(ROBOT_DIR, "meshes")
MOTIONS_DIR   = os.path.join(ROBOT_DIR, "motions")
IK_CACHE_PATH = os.path.join(ROBOT_DIR, _ROBOT_FILES[ROBOT]["ik_cache"])
VENDOR_DIR    = os.path.join(COMMON_DIR, "assets", "vendor")   # three.min.js (로봇 무관)
