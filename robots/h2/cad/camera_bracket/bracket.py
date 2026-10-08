#!/usr/bin/env python3
"""
H2 D435i 카메라 거치대 — 한 부품 (FDM PLA). 왼쪽 등판 M6 2개 → 왼쪽 어깨 위로 넘어와 → 가슴 앞 가운데 카메라 (아래로 45°).

  pip install cadquery          # 2.8 에서 확인
  python bracket.py out         # out/ 에 출력용 STL·STEP, 로봇 좌표 STL, 통합 STEP

좌표: torso_link [mm] (x 앞, y 왼쪽, z 위, 원점 = 허리 관절, 허리 0).
로봇 쪽 값: Unitree H2 STEP 'H2_简化模型_260601' 을 URDF torso_link 메시에 정합 (중앙 오차 1.2 mm) + 등판 도면
            (위 2×M6 간격 150 · 나사 깊이 18, 아래 2×M6 간격 176 · 깊이 13, 위아래 100).
            구멍은 등 커버의 지름 10 mm 우물 바닥에 있음 → 스피곳이 우물에 들어가 바닥(나사 시작면)에 닿게 조임.

부품 cam_mount_L : 등 받침판 + 어깨 넘는 띠 + 앞 다리 + 가로대 + 45° 카메라 자리 — 한 덩어리.
                  출력: 바깥면(y 129)을 바닥에 → 가로대가 위로 서는 방향.
볼트: 등판 왼쪽 위 M6×50 · 아래 M6×40 (접시머리), 카메라 M3×5 2개 (D435i 뒷면). 전부 4개.
피하는 것: 머리 전 범위 (숙임 −30..48°, 좌우 ±100° → 어깨 넘는 띠는 y ≥ 105),
          잡기 시퀀스 팔 이동 범위 (등 쪽 띠는 z 335 부터, 아래 구멍 둘레 혀 바깥 끝 y 94.5, 앞 발·가로대는 z 338 부터).
"""
import math, os, sys
import cadquery as cq

OUT = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(OUT, exist_ok=True)

# ---------- 로봇 쪽 값 (왼쪽 구멍만 사용) ----------
HOLE_U = dict(y=75.0, z=389.1, floor_x=-55.3, rim_x=-69.2)   # 위 M6: 우물 바닥 x, 우물 테두리 가장 뒤 x
HOLE_L = dict(y=88.0, z=289.1, floor_x=-63.3, rim_x=-75.7)   # 아래 M6
WELL_D = 10.0

# ---------- 치수 ----------
T = 12.0                                  # 판·띠 두께 (한쪽 지지라 9 → 12, 띠 굽힘 강성 2.4 배)
PX0, PX1 = -93.0, -81.0                   # 등 받침판 뒷면/앞면 x (앞면은 등판 표면 가장 뒤 −77 에서 4 mm)
BAND = (105.0, 129.0)                     # 띠 y (머리 이동 범위 밖)
Z_BACK0 = 335.0                           # 받침판·띠 아래 끝 z (그 아래 y 95–130 은 팔 지나가는 자리, 팔 최고 324)
PAD = dict(y0=66.0, z1=405.0)             # 받침판 (위 구멍 둘레)
LOBE = dict(y0=71.5, y1=94.5, z0=281.5, chamfer=4.0)   # 아래 구멍 둘레 혀 (바깥 아래 모서리 45° 모따기 — 팔 회피)
# 혀 바깥 채움 (출력할 때 혀를 받치는 45° 경사): 아래 경계 (y, z). 잡기 시퀀스 팔 최고점 + 4 mm 이상
FILL = [(94.0, 302.0), (120.3, 328.3), (BAND[1], 328.3)]
R = 30.0                                  # 굽힘 중심선 반경
Z_TOP_C = 432.0                           # 어깨 위 수평부 중심선 z (아랫면 426, 어깨 윗면 최고 408.5)
X_FRONT_C = 86.5                          # 앞 다리 중심선 x (x 80.5–92.5)
FOOT = dict(x0=80.5, x1=121.0, z0=338.0, z1=362.0)
BAR = dict(x0=98.0, x1=121.0, z0=338.0, z1=362.0, y_end=-50.0)   # 가로대: 발에서 카메라 오른쪽 끝(−45) 너머 −50 까지
SPIGOT_D, BOSS_D, BOLT_D = 9.3, 14.0, 6.6
# 볼트: M6 접시머리 렌치볼트 (사용자 M6.stl: 머리 Ø11.1·90°, 길이 = 머리 포함 전체), 와셔 없음
CSK_D = 11.6                              # 90° 접시 자리 윗지름 (머리 11.1 + 0.5) → 머리 윗면이 면보다 0.25 들어감
# 카메라 D435i (90×25×25 — 데이터시트 확인 필요)
LENS = (130.0, 350.0); PITCH = 45.0       # 렌즈 앞면 중심 (x, z), 아래로 숙임
CAM_W, CAM_H, CAM_D = 90.0, 25.0, 25.0
SEAT_Y = (-60.0, 50.0)                    # 45° 자리 y 범위 (오른쪽은 가로대 끝 밖까지)
# 카메라 고정: D435i 뒷면 M3 2개 (간격 45 — 사용자 기존 거치대 실측) + 아래는 45° 자리에 얹힘
BACK_T, BACK_Y0, BACK_Y1, M3_D, M3_PITCH = 4.0, -32.0, 32.0, 3.4, 45.0   # 뒤판: 왼쪽 끝은 45° 경사로 이어짐 (출력 서포트 없음)
M3_CB_D, M3_CB_H = 6.5, 2.0               # M3×5 머리 자리 → 머리 밑 판 2 mm, 카메라에 3 mm 박힘
# USB-C 플러그 자리: 카메라 끝면(|y| 45) 가운데에서 옆으로 나가는 플러그(단면 약 14×9, 길이 ~35) + 위로 꺾이는 케이블.
#   오른쪽(−y)은 가로대가 끝나서 열려 있음, 왼쪽(+y)은 가로대를 파냄 (포트가 어느 쪽인지 확인 필요)
USB_CUT = dict(s=10.0, t0=0.0, t1=40.0, y0=44.0, y1=84.0)    # 카메라 바닥 기준 앞뒤 ±s, 위로 t0..t1, y y0..y1


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().add(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)))


def cyl_x(r, x0, x1, y, z):
    return cq.Workplane().add(cq.Solid.makeCylinder(r, x1 - x0, cq.Vector(x0, y, z), cq.Vector(1, 0, 0)))


def cyl_y(r, y0, y1, x, z):
    return cq.Workplane().add(cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0)))


def tear_x(r, x0, x1, y, z, tip, rmax=None):
    """x 방향 눈물방울 기둥 (옆으로 출력되는 원기둥의 받침 없는 모양). tip ±1 = 뾰족한 쪽 y. rmax: 끝 자르기."""
    c = cyl_x(r, x0, x1, y, z)
    k = r / math.sqrt(2)
    tri = (cq.Workplane("YZ", origin=(x0, 0, 0))
           .polyline([(y + tip * k, z - k), (y + tip * r * math.sqrt(2), z), (y + tip * k, z + k)]).close().extrude(x1 - x0))
    t = c.union(tri)
    if rmax is not None:
        a, b = sorted((y + tip * rmax, y + tip * 2 * r))
        t = t.cut(box(x0 - 1, x1 + 1, a, b, z - r - 1, z + r + 1))
    return t


def csk(x, y, z):
    """+x 방향 90° 접시 자리 (윗지름 CSK_D → BOLT_D), 면 x."""
    h = (CSK_D - BOLT_D) / 2
    return cq.Workplane().add(cq.Solid.makeCone(CSK_D / 2 + 1.0, BOLT_D / 2, h + 1.0, cq.Vector(x - 1.0, y, z), cq.Vector(1, 0, 0)))


def arc_quarter(cx, cz, rin, rout, y0, y1, qx):
    ring = cyl_y(rout, y0, y1, cx, cz).cut(cyl_y(rin, y0 - 1, y1 + 1, cx, cz))
    xs = (cx, cx + rout + 1) if qx > 0 else (cx - rout - 1, cx)
    return ring.intersect(box(xs[0], xs[1], y0 - 1, y1 + 1, cz, cz + rout + 1))


def cam_frame():
    """카메라 바닥면 중심 b0 (x, z) 와 카메라 기준 → 로봇 좌표 변환 (s = 앞 f, t = 위 u)."""
    ph = math.radians(PITCH)
    f = (math.cos(ph), -math.sin(ph)); u = (math.sin(ph), math.cos(ph))
    b0 = (LENS[0] - CAM_D / 2 * f[0] - CAM_H / 2 * u[0], LENS[1] - CAM_D / 2 * f[1] - CAM_H / 2 * u[1])
    return b0, lambda wp: wp.rotate((0, 0, 0), (0, 1, 0), PITCH).translate((b0[0], 0, b0[1]))


def mount():
    """한 부품 (로봇 좌표). 반환: (부품, 카메라 더미, b0)."""
    y0, y1 = BAND
    z_arc = Z_TOP_C - R
    xb = (PX0 + PX1) / 2
    # 등 받침판 + 혀 + 혀 받침 경사
    s = box(PX0, PX1, PAD["y0"], y1, Z_BACK0, PAD["z1"])
    s = s.union(box(PX0, PX1, LOBE["y0"], LOBE["y1"], LOBE["z0"], Z_BACK0 + 1))
    fill = [(FILL[0][0], Z_BACK0 + 1)] + FILL + [(FILL[-1][0], Z_BACK0 + 1)]
    s = s.union(cq.Workplane("YZ", origin=(PX0, 0, 0)).polyline(fill).close().extrude(T))
    c = LOBE["chamfer"]
    s = s.cut(cq.Workplane("YZ", origin=(PX0 - 1, 0, 0))
              .polyline([(LOBE["y1"] - c, LOBE["z0"] - 0.01), (LOBE["y1"] + 0.01, LOBE["z0"] - 0.01), (LOBE["y1"] + 0.01, LOBE["z0"] + c)])
              .close().extrude(T + 2))
    # 어깨 넘는 띠 → 앞 다리 → 발 → 가로대
    s = s.union(arc_quarter(xb + R, z_arc, R - T / 2, R + T / 2, y0, y1, -1))
    s = s.union(box(xb + R, X_FRONT_C - R, y0, y1, Z_TOP_C - T / 2, Z_TOP_C + T / 2))
    s = s.union(arc_quarter(X_FRONT_C - R, z_arc, R - T / 2, R + T / 2, y0, y1, +1))
    s = s.union(box(X_FRONT_C - T / 2, X_FRONT_C + T / 2, y0, y1, FOOT["z0"], z_arc))
    s = s.union(box(FOOT["x0"], FOOT["x1"], y0, y1, FOOT["z0"], FOOT["z1"]))
    s = s.union(box(BAR["x0"], BAR["x1"], BAR["y_end"], y1, BAR["z0"], BAR["z1"]))
    # 등판 볼트: 보스 + 스피곳 (출력 바닥 +y 쪽 눈물방울), 볼트 구멍 (위 −y 쪽 눈물방울), 접시 자리
    for h in (HOLE_U, HOLE_L):
        be = h["rim_x"] - 1.5
        if h is HOLE_U:
            s = s.union(tear_x(BOSS_D / 2, PX1 - 0.5, be, h["y"], h["z"], +1))
        else:   # 아래 보스는 짧고(4.3 mm) 팔 쪽이라 둥글게 Ø12
            s = s.union(cyl_x(6.0, PX1 - 0.5, be, h["y"], h["z"]))
        s = s.union(tear_x(SPIGOT_D / 2, be - 0.5, h["floor_x"], h["y"], h["z"], +1, rmax=WELL_D / 2 - 0.15))
        s = s.cut(tear_x(BOLT_D / 2, PX0 - 1, be, h["y"], h["z"], -1))
        s = s.cut(cyl_x(BOLT_D / 2, be - 1, h["floor_x"] + 1, h["y"], h["z"]))
        s = s.cut(csk(PX0, h["y"], h["z"]))
    # 카메라 자리
    b0, to_robot = cam_frame()

    def local(s0, s1, ya, yb, t0, t1):
        return to_robot(box(s0, s1, ya, yb, t0, t1))
    s = s.cut(local(-40, 40, SEAT_Y[0], SEAT_Y[1], 0, 40))                                         # 45° 자리
    s = s.cut(local(-USB_CUT["s"], USB_CUT["s"], USB_CUT["y0"], USB_CUT["y1"], USB_CUT["t0"], USB_CUT["t1"]))   # USB-C (왼쪽)
    t_top, sb = CAM_H, -CAM_D / 2 - BACK_T
    ramp_end = BACK_Y1 + t_top + 3.0                                                                 # 45° 경사 끝 (t = −3)
    back = (cq.Workplane("YZ", origin=(sb, 0, 0))
            .polyline([(BACK_Y0, -3.0), (ramp_end, -3.0), (BACK_Y1, t_top), (BACK_Y0, t_top)]).close().extrude(BACK_T))
    s = s.union(to_robot(back))                                                                      # 뒤판 (왼쪽 끝 45° 경사)
    for yy in (-M3_PITCH / 2, M3_PITCH / 2):                                                          # M3 구멍 + 머리 자리
        for d, hgt in ((M3_D, BACK_T + 2), (M3_CB_D, M3_CB_H + 1)):
            s = s.cut(to_robot(cq.Workplane().add(cq.Solid.makeCylinder(d / 2, hgt, cq.Vector(sb - 1, yy, CAM_H / 2), cq.Vector(1, 0, 0)))))
    cam = local(-CAM_D / 2, CAM_D / 2, -CAM_W / 2, CAM_W / 2, 0, CAM_H)
    return s, cam, b0


# ---------- 볼트 (조립 확인용 모델, 나사산 생략) ----------
def _plane(origin, d):
    d = cq.Vector(*d).normalized()
    x = cq.Vector(0, 0, 1) if abs(d.z) < 0.9 else cq.Vector(1, 0, 0)
    x = (x - d * x.dot(d)).normalized()
    return cq.Plane(origin=cq.Vector(*origin), xDir=x, normal=d)


def socket_screw(seat, d, dia, length, head_d, head_h, af):
    """렌치볼트: seat = 머리 밑면 중심, d = 나사 방향."""
    pl = _plane(seat, d)
    shank = cq.Workplane(pl).circle(dia / 2).extrude(length)
    head = cq.Workplane(pl).circle(head_d / 2).extrude(-head_h)
    sock = cq.Workplane(_plane(cq.Vector(*seat) - cq.Vector(*d).normalized() * head_h, d)).polygon(6, af / math.cos(math.pi / 6)).extrude(head_h * 0.6)
    return shank.union(head).cut(sock)


def csk_screw(top, d, length, head_d=11.1, dia=6.0, af=4.0):
    """접시머리 렌치볼트: top = 머리 윗면 중심, d = 나사 방향, length = 머리 포함 전체 (사용자 M6.stl 치수)."""
    pl = _plane(top, d)
    hh = (head_d - dia) / 2
    head = cq.Workplane().add(cq.Solid.makeCone(head_d / 2, dia / 2, hh, cq.Vector(*top), cq.Vector(*d)))
    shank = cq.Workplane(pl).circle(dia / 2).extrude(length)
    sock = cq.Workplane(pl).polygon(6, af / math.cos(math.pi / 6)).extrude(hh * 0.9)
    return shank.union(head).cut(sock)


def hardware(b0):
    """(이름, 모델) 목록 — torso_link mm, 조립 위치."""
    sink = (CSK_D - 11.1) / 2                 # 머리 윗면이 면보다 들어가는 깊이
    out = [("M6x50_csk_upper", csk_screw((PX0 + sink, HOLE_U["y"], HOLE_U["z"]), (1, 0, 0), 50.0)),
           ("M6x40_csk_lower", csk_screw((PX0 + sink, HOLE_L["y"], HOLE_L["z"]), (1, 0, 0), 40.0))]
    ph = math.radians(PITCH); f = cq.Vector(math.cos(ph), 0, -math.sin(ph)); u = cq.Vector(math.sin(ph), 0, math.cos(ph))
    for yy in (-M3_PITCH / 2, M3_PITCH / 2):
        seat = cq.Vector(b0[0], yy, b0[1]) + f * (-CAM_D / 2 - BACK_T + M3_CB_H) + u * (CAM_H / 2)
        out.append((f"M3x5_cam_{'L' if yy > 0 else 'R'}", socket_screw((seat.x, seat.y, seat.z), (f.x, f.y, f.z), 3.0, 5.0, 5.5, 3.0, 2.5)))
    return out


def to_bed(part):
    """출력 방향: 바깥면(y 129) 이 바닥 → 높이 = 129 − y."""
    p = part.rotate((0, 0, 0), (1, 0, 0), -90)
    bb = p.val().BoundingBox()
    return p.translate((-bb.xmin, -bb.ymin, -bb.zmin))


if __name__ == "__main__":
    P, CAMBOX, B0 = mount()
    for name, part in (("cam_mount_L", P), ("D435i_dummy", CAMBOX)):
        v = part.val(); bb = v.BoundingBox()
        print(f"{name:12s} 부피 {v.Volume()/1000:6.1f} cm3  bbox x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")
        cq.exporters.export(part, f"{OUT}/{name}_torso_frame.stl", tolerance=0.05, angularTolerance=0.1)
    PP = to_bed(P)
    bb = PP.val().BoundingBox()
    print(f"출력 cam_mount_L {bb.xlen:5.0f} × {bb.ylen:5.0f} × 높이 {bb.zlen:4.0f} mm")
    cq.exporters.export(PP, f"{OUT}/cam_mount_L_print.stl", tolerance=0.05, angularTolerance=0.1)
    cq.exporters.export(PP, f"{OUT}/cam_mount_L_print.step")
    # 한 파일 통합: 부품 + 카메라 더미 + 볼트 (torso_link 좌표, mm)
    asm = cq.Assembly(name="h2_cam_mount")
    asm.add(P, name="cam_mount_L", color=cq.Color(0.9, 0.35, 0.2)).add(CAMBOX, name="D435i_dummy", color=cq.Color(0.15, 0.15, 0.15))
    hw = hardware(B0)
    for name, part in hw:
        asm.add(part, name=name, color=cq.Color(0.75, 0.75, 0.78))
    asm.export(f"{OUT}/h2_cam_bracket_all.step")
    print("볼트", len(hw), "개:", ", ".join(n for n, _ in hw))
    print("ok")
