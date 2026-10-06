"""H2 정면 건네기 시퀀스 IK 가능 범위 (오프라인 근사).  사용: python utils/h2_handover.py robots/h2/H2.urdf"""
import os
import sys, numpy as np, pinocchio as pin
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ik_reach import build, solve
GE, AE, GZ = -0.05, 0.10, 0.08; W, H = 0.28, 0.12; HX = 0.30
m = build(sys.argv[1], None, ["head_pitch_joint", "head_yaw_joint"]); d = m.createData()
def ok(x, top, by=0.0):
    g = W/2+GE; a = g+AE; gz, az, lz = top-H/2+GZ, top+0.10, top+0.15
    pts = [(x,by,a,az),(x,by,a,gz),(x,by,g,gz),(x,by,g,lz),(HX,0,g,lz),(HX,0,a,lz)]   # 접근,하강,잡기,들기,건네기,손벌림
    q = np.zeros(m.nq)
    for px,c,off,z in pts:
        best=None
        for q0 in (q, np.zeros(m.nq)):
            r = solve(m, d, pin.SE3(np.eye(3), np.array([px,c+off,z])), pin.SE3(np.eye(3), np.array([px,c-off,z])), q0)
            if best is None or r[1]+r[2]/500 < best[1]+best[2]/500: best=r
        q = best[0]
        if best[1] > 0.01 or best[2] > 5: return False
    return True
xs=[0.25,0.30,0.35,0.40]
for by in (0.0, 0.05, 0.10):
    print(f"건네기(정면, 허리 0) — 박스 중심 y={by:+.2f}   행 top(pelvis 기준) / 열 x")
    for top in (0.10,0.15,0.20,0.25):
        print(f"  {top:+.2f} "+"".join("  ✓  " if ok(x,top,by) else "  ·  " for x in xs))
