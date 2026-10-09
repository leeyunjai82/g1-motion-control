#!/usr/bin/env python3
"""
H2 등판 손잡이 (미니 PC 걸이) — 카메라 거치대 요크 받침판 위에 겹쳐 등판 M6 4개로 같이 고정.
사용자 back.stl(고리 달린 판, 수정 없이 사용)의 고리를 손잡이 핀(fin) 윗변에 걸어 둠.

  v2 (2026-10-09): 위 볼트를 손으로 돌릴 수 있게 — v1 은 머리가 받침대 속 Ø12.5 구멍 31 mm 안쪽이라 손이 안 들어감.
      위 볼트 둘레를 뒤에서 R 12 로 파서 머리가 5 mm 얇은 판 위에 드러나게 (아래 볼트처럼 면에 바로).
      받침대·핀은 볼트 안쪽 |y| 48–62 로, 띠를 안쪽으로 넓힌 판(TAB, 앞면 x −94, 두께 8)에 붙임.

  python handle.py <out> [back.stl]   # out/handle_print.stl·step (출력 방향), handle_torso_frame.stl, (back.stl 주면) 걸린 모습

좌표: torso_link [mm] (x 앞, y 왼쪽, z 위). 요크 받침판 뒷면 x −90 (위 구멍 둘레), 아래 구멍 3 mm 돋움 뒷면 x −93.
back.stl 고리 단면 (실측): 입구 7 mm (z_part 8–15), 입구→천장 22 mm, 안쪽 폭 12 mm, 천장 평면 z_part 8–18.
걸린 자세: back.stl 의 x_part → 아래, z_part → 로봇 쪽(앞), 밑판(z_part 0)이 바깥 = 미니 PC 쪽 (사용자 그림).

볼트 (모두 M6 접시머리, 머리 Ø11.1 · 90°):
  위 구멍 2: M6×50 (지금 것 그대로) — 머리가 x −95 판(두께 5)에 앉음, 뒤·위·바깥쪽이 트여 손·L 렌치가 바로 닿음. 박힘 10.55 (도면 깊이 18)
  아래 구멍 2: M6×40 → M6×50 로 교체 — 손잡이 뒷면 x −102 에 앉음, 박힘 11.55 (도면 깊이 13)
"""
import math, os, sys
import cadquery as cq

OUT = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(OUT, exist_ok=True)

HOLE_U = dict(y=75.0, z=389.1, floor_x=-55.3)
HOLE_L = dict(y=88.0, z=289.1, floor_x=-63.3)
PAD_X, BOSS_X, BOSS_R = -90.0, -93.0, 8.0        # 요크 받침판 뒷면, 아래 구멍 돋움 뒷면·반경
BACK_X = -102.0                                  # 손잡이 띠 뒷면
STEP_Z = 315.0                                   # 이 아래는 띠 앞면 x −93 (돋움에 얹힘), 위는 −90 (받침판에 얹힘)
TOP_Z = 400.0                                    # 띠·받침대·핀 윗면 (출력 바닥면)
FIN = dict(x0=-126.0, x1=-120.0, h=26.0, half=62.0)   # 핀: 두께 6 (고리 입구 7), 높이 26 (입구→천장 22 + 4)
POST = dict(y0=48.0, y1=62.0, z0=360.0)          # 받침대 (띠 ↔ 핀) — 위 볼트 안쪽. 고리 판(폭 87 → |y| 43.5)과 4.5 mm
TAB = dict(x1=-94.0, y0=48.0, y1=80.0, z0=360.0) # 받침대를 붙이는 띠 안쪽 넓힘 (앞면 −94: 등 가운데 위쪽이 −88 까지 나옴)
U_SEAT_X = -95.0                                 # 위 볼트 머리 자리 (그 앞 판 5 mm)
SPOT_R = 12.0                                    # 위 볼트 둘레 파기 반경 (뒤에서 머리 자리까지, 위로 트임)
BOLT_D, CSK_D = 6.6, 11.6


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().add(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)))


def cyl_x(r, x0, x1, y, z):
    return cq.Workplane().add(cq.Solid.makeCylinder(r, x1 - x0, cq.Vector(x0, y, z), cq.Vector(1, 0, 0)))


def tear_x(r, x0, x1, y, z):
    """x 방향 눈물방울 기둥 — 뾰족한 쪽 −z (출력은 뒤집어 하므로 출력 때 위쪽 = 받침 없는 천장 없음)."""
    k = r / math.sqrt(2)
    tri = (cq.Workplane("YZ", origin=(x0, 0, 0))
           .polyline([(y - k, z - k), (y, z - r * math.sqrt(2)), (y + k, z - k)]).close().extrude(x1 - x0))
    return cyl_x(r, x0, x1, y, z).union(tri)


def csk(x, y, z):
    h = (CSK_D - BOLT_D) / 2
    return cq.Workplane().add(cq.Solid.makeCone(CSK_D / 2 + 1.0, BOLT_D / 2, h + 1.0, cq.Vector(x - 1.0, y, z), cq.Vector(1, 0, 0)))


def strap_left():
    """위 구멍 r 9 ~ 아래 구멍 r 8 를 잇는 띠 (y-z 평면 윤곽) + 위쪽은 TOP_Z 까지 네모."""
    import numpy as np
    from scipy.spatial import ConvexHull
    yu, zu, yl, zl = HOLE_U["y"], HOLE_U["z"], HOLE_L["y"], HOLE_L["z"]
    ru, rl = 9.0, BOSS_R
    # 두 원 + 윗변(TOP_Z) 의 볼록 껍질 (원은 48 점 근사)
    P = [(yu + ru * math.cos(a), zu + ru * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    P += [(yl + rl * math.cos(a), zl + rl * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    P += [(yu - ru, TOP_Z), (yu + ru, TOP_Z)]
    H = ConvexHull(np.array(P))
    hull = [tuple(map(float, np.array(P)[i])) for i in H.vertices]
    s = cq.Workplane("YZ", origin=(BACK_X, 0, 0)).polyline(hull).close().extrude(PAD_X - BACK_X)
    s = s.cut(box(BOSS_X, PAD_X + 1, 0, 200, 0, STEP_Z))                 # 아래쪽 앞면 x −93 (돋움에 얹힘)
    s = s.union(box(BACK_X, TAB["x1"], TAB["y0"], TAB["y1"], TAB["z0"], TOP_Z))   # 안쪽 넓힘 (받침대 자리)
    return s


def handle():
    s = strap_left()
    s = s.union(box(FIN["x0"], BACK_X, POST["y0"], POST["y1"], POST["z0"], TOP_Z))      # 받침대
    s = s.mirror("XZ", union=True)
    s = s.union(box(FIN["x0"], FIN["x1"], -FIN["half"], FIN["half"], TOP_Z - FIN["h"], TOP_Z))   # 핀
    for sg in (1, -1):
        yu, yl = sg * HOLE_U["y"], sg * HOLE_L["y"]
        # 위 볼트: 뒤에서 머리 자리(x −95)까지 R 12 로 파서 머리가 판 위에 드러남 (위로 트임), 접시, 앞쪽 Ø6.6
        s = s.cut(tear_x(SPOT_R, FIN["x0"] - 1, U_SEAT_X, yu, HOLE_U["z"]))
        s = s.cut(csk(U_SEAT_X, yu, HOLE_U["z"]))
        s = s.cut(cyl_x(BOLT_D / 2, U_SEAT_X - 1, PAD_X + 1, yu, HOLE_U["z"]))
        # 아래 볼트: 뒷면 x −102 접시
        s = s.cut(csk(BACK_X, yl, HOLE_L["z"]))
        s = s.cut(cyl_x(BOLT_D / 2, BACK_X - 1, BOSS_X + 1, yl, HOLE_L["z"]))
    return s


def csk_screw(top, length, head_d=11.1, dia=6.0):
    hh = (head_d - dia) / 2
    head = cq.Workplane().add(cq.Solid.makeCone(head_d / 2, dia / 2, hh, cq.Vector(*top), cq.Vector(1, 0, 0)))
    return head.union(cyl_x(dia / 2, top[0], top[0] + length, top[1], top[2]))


if __name__ == "__main__":
    import numpy as np, trimesh
    H = handle()
    bb = H.val().BoundingBox()
    print(f"handle 부피 {H.val().Volume()/1000:.1f} cm3  x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")
    cq.exporters.export(H, f"{OUT}/handle_torso_frame.stl", tolerance=0.05, angularTolerance=0.1)
    P = H.rotate((0, 0, 0), (1, 0, 0), 180)                          # 출력: 윗면(z 400)을 바닥으로
    pb = P.val().BoundingBox(); P = P.translate((-pb.xmin, -pb.ymin, -pb.zmin))
    cq.exporters.export(P, f"{OUT}/handle_print.stl", tolerance=0.05, angularTolerance=0.1)
    cq.exporters.export(P, f"{OUT}/handle_print.step")
    # 볼트 (머리 윗면 = 자리 + 0.25)
    sink = (CSK_D - 11.1) / 2
    bolts = []
    for sg in (1, -1):
        bolts.append(csk_screw((U_SEAT_X + sink, sg * HOLE_U["y"], HOLE_U["z"]), 50.0))
        bolts.append(csk_screw((BACK_X + sink, sg * HOLE_L["y"], HOLE_L["z"]), 50.0))
    for i, b in enumerate(bolts):
        cq.exporters.export(b, f"{OUT}/bolt{i}_torso_frame.stl", tolerance=0.05, angularTolerance=0.1)
    print("박힘: 위", round(U_SEAT_X + sink + 50 - HOLE_U["floor_x"], 2), " 아래", round(BACK_X + sink + 50 - HOLE_L["floor_x"], 2))
    # back.stl 을 걸린 자세로: (x_p, y_p, z_p) → (z_p − 134.5, y_p − 183.15, TOP_Z − 129.41 − x_p)
    src = sys.argv[2] if len(sys.argv) > 2 else "in/back.stl"
    if not os.path.exists(src):
        print(f"back.stl 없음 ({src}) — 걸린 판 모델은 건너뜀"); print("ok"); sys.exit(0)
    m = trimesh.load(src)
    v = m.vertices
    m.vertices = np.c_[v[:, 2] - 134.5, v[:, 1] - 183.15, TOP_Z - 129.41 - v[:, 0]]
    if np.linalg.det(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])) < 0:
        m.invert()
    m.export(f"{OUT}/back_hung_torso_frame.stl")
    print("back 걸린 자세 bounds", m.bounds.round(1).tolist())
    print("출력 크기", np.round([pb.xlen, pb.ylen, pb.zlen], 1))
    print("ok")
