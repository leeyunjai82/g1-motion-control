"""H2 제자리 박스 옮기기 시퀀스 IK 가능 범위 (오프라인 근사, 로봇 불필요).

사용: python utils/h2_seq.py robots/h2/H2.urdf <박스폭m> <박스높이m>   (pinocchio 만 필요)

조건 (사용자 결정 2026-10):
  · 허리 0 고정 (yaw/roll/pitch 모두 잠금) — 상체 숙이기/돌리기 없음
  · 손 자세 = 단위 회전 (G1 wrist_params 기본값 = 손바닥 마주보기 가정, 3D 뷰어로 확인 예정)
  · 옆으로 옮기기는 팔만으로 (박스 중심 y 를 dy 만큼 이동)
시퀀스 점 (G1 robot_server.grab_box 와 같은 오프셋):
  접근  : y = c ± (W/2 + GRIP_EXTRA + APPROACH_EXTRA), z = top + 0.10
  하강  : 같은 y, z = top - h/2 + GRAB_Z_OFFSET
  잡기  : y = c ± (W/2 + GRIP_EXTRA)
  들기  : z = top + 0.15
  옮기기: c → c + dy (들기 높이)
  내리기: z = 잡기 높이,  손 벌림: y = c+dy ± (W/2 + GRIP_EXTRA + APPROACH_EXTRA)
모든 점을 이전 해에서 이어 풀어(warm start) 1 cm / 5° 이내면 가능.
pelvis 기준 좌표. 바닥 기준 높이 = pelvis 높이(실측 필요) + z.
"""
import os
import sys
import numpy as np
import pinocchio as pin
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ik_reach import build, solve

GRIP_EXTRA, APPROACH_EXTRA, GRAB_Z_OFFSET = -0.05, 0.10, 0.08   # G1 robot_server 값 그대로
W, H = float(sys.argv[2]), float(sys.argv[3])                    # 박스 폭(y), 높이 [m]

m = build(sys.argv[1], None, ["head_pitch_joint", "head_yaw_joint"])
d = m.createData()


def seq_ok(x, top, dy):
    grip = W / 2 + GRIP_EXTRA
    app = grip + APPROACH_EXTRA
    gz, az, lz = top - H / 2 + GRAB_Z_OFFSET, top + 0.10, top + 0.15
    pts = [(0, app, az), (0, app, gz), (0, grip, gz), (0, grip, lz),
           (dy, grip, lz), (dy, grip, gz), (dy, app, gz)]
    q = np.zeros(m.nq)
    worst = 0.0
    for c, off, z in pts:
        TL = pin.SE3(np.eye(3), np.array([x, c + off, z]))
        TR = pin.SE3(np.eye(3), np.array([x, c - off, z]))
        best = None
        for q0 in (q, np.zeros(m.nq)):
            r = solve(m, d, TL, TR, q0)
            if best is None or r[1] + r[2] / 500 < best[1] + best[2] / 500:
                best = r
        q, pe, re = best
        if pe > 0.01 or re > 5:
            return False, pe
        worst = max(worst, pe)
    return True, worst


print(f"박스 W={W:.2f} m, H={H:.2f} m, 허리 0, 손 단위 회전")
xs = [0.25, 0.30, 0.35, 0.40, 0.45]
tops = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
for dy in (0.0, 0.10, 0.15, 0.20, 0.25):
    print(f"\n옆으로 dy = {dy:+.2f} m (왼쪽).  행: 박스 윗면 z(pelvis 기준), 열: 박스 중심 x")
    print("  top\\x " + "".join(f"{x:>7.2f}" for x in xs))
    for top in tops:
        print(f"  {top:+.2f} " + "".join(f"{'   ✓   ' if seq_ok(x, top, dy)[0] else '   ·   '}" for x in xs))
