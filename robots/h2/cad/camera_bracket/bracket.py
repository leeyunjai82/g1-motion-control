#!/usr/bin/env python3
"""
H2 D435i 카메라 거치대 (FDM PLA, 3부품) — 등판 M6 4개 → 어깨 위로 넘어와 가슴 앞 카메라 (아래로 60°).
  v3 (2026-10-09): 가로대 전체 15 mm 위 (요크 발·체결 구멍 +15, 가슴 로고를 안 가리게) · 숙임 45° → 60°.
      가로대는 v1 모양 그대로 각도만 (60° 면 뒤판 아래가 가로대 위로 떠서 뒤판 폭만 작은 받침). 요크 2개 + 가로대 다시 출력

  pip install cadquery          # 2.8 에서 확인
  python bracket.py out         # out/ 에 부품별 STL·STEP (출력 방향 / 로봇 좌표) + 조립 STEP

좌표: torso_link [mm] (x 앞, y 왼쪽, z 위, 원점 = 허리 관절, 허리 0).
로봇 쪽 값: Unitree H2 STEP 'H2_简化模型_260601' 을 URDF torso_link 메시에 정합 (중앙 오차 1.2 mm) + 등판 도면
            (위 2×M6 간격 150 · 나사 깊이 18, 아래 2×M6 간격 176 · 깊이 13, 위아래 100).
            구멍은 등 커버의 지름 10 mm 우물 바닥에 있음 → 스피곳이 우물에 들어가 바닥(나사 시작면)에 닿게 조임.

부품
  yoke_L / yoke_R : 등 받침판 + 어깨 넘는 띠 한 덩어리. 출력: 바깥면을 바닥에 (혀 밑은 팔 범위 밖까지 경사로 채움).
연결: M6 접시머리 — 등판 위 M6×50 2개 · 아래 M6×40 2개, 요크–가로대 M6×40 4개 + 너트 4개, 카메라 M3×5 2개 (D435i 뒷면 나사).
한 파일: out/h2_cam_bracket_all.step (부품 + 카메라 더미 + 볼트·너트·와셔, 로봇 좌표)
  camera_bar      : 가로대 + 60° 카메라 자리 + 뒤판 (D435i 뒷면 M3 2개, 간격 45). 출력: 아랫면을 바닥에,
                    서포트는 뒤판 밑(카메라 자리 쪽)만 — 60° 면 뒤판이 수평에서 30° 로 누움
피하는 것: 머리 전 범위 (숙임 −30..48°, 좌우 ±100° → 띠는 |y| ≥ 105), 잡기 시퀀스 팔 이동 범위
          (등 쪽 띠는 z 335 부터, 아래 구멍 둘레 혀는 바깥 끝 y 95.5, 앞 발·가로대는 z 338 부터).
"""
import math, os, sys
import cadquery as cq

OUT = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(OUT, exist_ok=True)

# ---------- 로봇 쪽 값 ----------
HOLE_U = dict(y=75.0, z=389.1, floor_x=-55.3, rim_x=-69.2)   # 위 M6: 우물 바닥 x, 우물 테두리 가장 뒤 x
HOLE_L = dict(y=88.0, z=289.1, floor_x=-63.3, rim_x=-75.7)   # 아래 M6
WELL_D = 10.0

# ---------- 치수 ----------
T = 9.0                                   # 판·띠 두께
PX0, PX1 = -90.0, -81.0                   # 등 받침판 뒷면/앞면 x (등판 표면 가장 뒤 −77 에서 4 mm)
BAND = (105.0, 129.0)                     # 띠 y (머리 이동 범위 밖)
Z_BACK0 = 335.0                           # 등 쪽 띠·받침판 위쪽 시작 z (팔 최고 325)
PAD = dict(y0=66.0, y1=105.5, z1=405.0)   # 받침판 (위 구멍) z Z_BACK0..405
LOBE = dict(y0=71.5, y1=94.5, z0=281.5, chamfer=4.0)   # 아래 구멍 둘레 혀 (바깥 아래 모서리 45° 모따기 — 팔 회피)
# 혀 바깥(y 94–129) 채움: 잡기 시퀀스 팔 이동 범위(받침판 앞뒤 5 mm, 좌우 5 mm 여유) + 4 mm 위로, 출력 기울기 ≤ 약 50°
#   (y, 아래 경계 z) — y 129 가 출력 바닥면. 이 아래(z 작은 쪽)는 팔이 지나가는 자리라 비워 둠
FILL_PROFILE = [(129.0, 328.3), (125.0, 328.3), (124.5, 327.7), (124.0, 327.3), (122.0, 327.3), (117.0, 321.3),
                (116.5, 320.9), (114.5, 320.9), (110.0, 315.5), (109.5, 315.2), (107.5, 315.2), (94.0, 299.0)]
R = 30.0                                  # 굽힘 중심선 반경
Z_TOP_C = 432.0                           # 어깨 위 수평부 중심선 z (아랫면 427.5, 어깨 윗면 최고 408.5)
X_FRONT_C = 85.0                          # 앞 수직부 중심선 x
UP = 15.0                                 # v3: 가로대·요크 발 전체를 올림 (v1 0) — 가슴 로고를 안 가리게 (사용자 2026-10-09)
FOOT = dict(x0=80.5, x1=121.0, z0=338.0 + UP, z1=362.0 + UP)
BAR = dict(x0=98.0, x1=121.0, z0=338.0 + UP, z1=362.0 + UP, half=105.0)
SPIGOT_D, BOSS_D, BOLT_D = 9.3, 14.0, 6.6
# 볼트: M6 접시머리 렌치볼트 (사용자 모델 M6.stl: 머리 Ø11.1·90°, 길이 = 머리 포함 전체), 와셔 없음
CSK_D = 11.6                              # 90° 접시 자리 윗지름 (머리 11.1 + 0.5) → 머리 윗면이 면보다 0.25 들어감
LOWER_PAD = 3.0                           # 아래 등판 구멍만 3 mm 돋움 (M6×40 박힘 10.6, 도면 나사 깊이 13) — 위는 M6×50 면 맞춤 (박힘 15.6, 깊이 18)
# 요크 발 – 가로대: M6×40 2개 (위아래), 바깥면 접시 자리. 너트(10×10×5) 홈은 끝에서 8 mm, 위아래로 뚫림
#   볼트 끝 |y| 88.75 → 너트(92–97) 밖으로 3 mm, 막힌 구멍 끝 87, USB 자리 84 밖
JOINT = ((109.5, 356.5 + UP), (109.5, 343.5 + UP))  # (x, z) — 위, 아래
NUT_AF, NUT_T, NUT_FROM_END = 10.4, 5.4, 8.0    # 너트 홈 (너트 10 × 10 × 5)
J_HOLE_DEPTH = 18.0
# 카메라 D435i (90×25×25 — 데이터시트 확인 필요)
LENS = (130.0, 350.0 + UP); PITCH = 60.0  # 렌즈 앞면 중심 (x, z), 아래로 숙임 — v3. v1 (출력·장착 2026-10-08): UP 0, 45° → 실측 47.4°
CAM_W, CAM_H, CAM_D = 90.0, 25.0, 25.0
SEAT_HALF = 52.0                          # 카메라 자리 |y|
# 카메라 고정: D435i 뒷면 M3 2개 (간격 45 — 사용자 기존 거치대 실측) + 아래는 경사 자리에 얹힘
BACK_T, BACK_HALF, M3_D, M3_PITCH = 4.0, 32.0, 3.4, 45.0
M3_CB_D, M3_CB_H = 6.5, 2.0               # M3×5 머리 자리 → 머리 밑 판 2 mm, 카메라에 3 mm 박힘 (기존 거치대 판 2 mm 와 같음)
# USB-C 플러그 자리: 카메라 끝면(|y| 45) 가운데에서 옆으로 나가는 플러그(단면 약 14×9, 길이 ~35) + 위로 꺾이는 케이블.
#   포트가 어느 쪽 끝인지 확인 필요 → 양쪽 다. 가로대 끝 M6 체결부(|y| 84–105) 는 남김
USB_CUT = dict(s=10.0, t0=2.0, t1=40.0, y0=44.0, y1=84.0)    # 카메라 몸통 중심 기준 앞뒤 ±s, 바닥에서 t0..t1, |y| y0..y1


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


def csk(face, y, z, d):
    """90° 접시 자리 (윗지름 CSK_D → BOLT_D). face: x 값(+ y, z) 또는 (x, y, z) 점, d: 볼트 들어가는 방향."""
    p = cq.Vector(face, y, z) if y is not None else cq.Vector(*face)
    dv = cq.Vector(*d)
    h = (CSK_D - BOLT_D) / 2
    cone = cq.Solid.makeCone(CSK_D / 2 + 1.0, BOLT_D / 2, h + 1.0, p - dv * 1.0, dv)
    return cq.Workplane().add(cone)


def arc_quarter(cx, cz, rin, rout, y0, y1, qx):
    ring = cyl_y(rout, y0, y1, cx, cz).cut(cyl_y(rin, y0 - 1, y1 + 1, cx, cz))
    xs = (cx, cx + rout + 1) if qx > 0 else (cx - rout - 1, cx)
    return ring.intersect(box(xs[0], xs[1], y0 - 1, y1 + 1, cz, cz + rout + 1))


def yoke_left():
    y0, y1 = BAND
    z_arc = Z_TOP_C - R
    xb = (PX0 + PX1) / 2
    s = box(PX0, PX1, PAD["y0"], y1, Z_BACK0, PAD["z1"])                                  # 받침판 + 띠 뒤 수직 (한 판)
    s = s.union(box(PX0, PX1, LOBE["y0"], LOBE["y1"], LOBE["z0"], Z_BACK0 + 1))           # 아래 혀
    fill = [(FILL_PROFILE[0][0], Z_BACK0 + 1)] + FILL_PROFILE + [(FILL_PROFILE[-1][0], Z_BACK0 + 1)]
    s = s.union(cq.Workplane("YZ", origin=(PX0, 0, 0)).polyline(fill).close().extrude(T))   # 혀 바깥 채움 (팔 범위 밖)
    c = LOBE["chamfer"]
    s = s.cut(cq.Workplane("YZ", origin=(PX0 - 1, 0, 0))
              .polyline([(LOBE["y1"] - c, LOBE["z0"] - 0.01), (LOBE["y1"] + 0.01, LOBE["z0"] - 0.01), (LOBE["y1"] + 0.01, LOBE["z0"] + c)])
              .close().extrude(T + 2))
    s = s.union(arc_quarter(xb + R, z_arc, R - T / 2, R + T / 2, y0, y1, -1))              # 뒤 굽힘
    s = s.union(box(xb + R, X_FRONT_C - R, y0, y1, Z_TOP_C - T / 2, Z_TOP_C + T / 2))    # 어깨 위
    s = s.union(arc_quarter(X_FRONT_C - R, z_arc, R - T / 2, R + T / 2, y0, y1, +1))       # 앞 굽힘
    s = s.union(box(X_FRONT_C - T / 2, X_FRONT_C + T / 2, y0, y1, FOOT["z0"], z_arc))     # 앞 수직
    s = s.union(box(FOOT["x0"], FOOT["x1"], y0, y1, FOOT["z0"], FOOT["z1"]))             # 발
    # 출력은 바깥면(y 129)이 바닥 → 옆으로 눕는 보스·스피곳은 바닥(+y) 쪽 눈물방울, 볼트 구멍은 위(−y) 쪽
    for h in (HOLE_U, HOLE_L):
        be = h["rim_x"] - 1.5
        if h is HOLE_U:
            s = s.union(tear_x(BOSS_D / 2, PX1 - 0.5, be, h["y"], h["z"], +1))
        else:   # 아래 보스는 짧고(3.8 mm) 팔 쪽이라 눈물방울 끝 없이 Ø12
            s = s.union(cyl_x(6.0, PX1 - 0.5, be, h["y"], h["z"]))
        s = s.union(tear_x(SPIGOT_D / 2, be - 0.5, h["floor_x"], h["y"], h["z"], +1, rmax=WELL_D / 2 - 0.15))
        face = PX0 - (LOWER_PAD if h is HOLE_L else 0.0)                                    # 볼트 머리 쪽 면 x
        if h is HOLE_L:
            s = s.union(cyl_x(8.0, face, PX0 + 0.5, h["y"], h["z"]))                         # 아래 구멍 3 mm 돋움 (둥글게 — 팔 쪽)
        s = s.cut(tear_x(BOLT_D / 2, face - 1, be, h["y"], h["z"], -1))
        s = s.cut(cyl_x(BOLT_D / 2, be - 1, h["floor_x"] + 1, h["y"], h["z"]))
        s = s.cut(csk(face, h["y"], h["z"], (1, 0, 0)))                                       # 접시 자리
    for (x, z) in JOINT:
        s = s.cut(cyl_y(BOLT_D / 2, y0 - 1, y1 + 1, x, z))
        s = s.cut(csk((x, y1, z), None, None, (0, -1, 0)))                                     # 바깥면 접시 자리
    return s


def camera_bar():
    ph = math.radians(PITCH)
    f = (math.cos(ph), -math.sin(ph)); u = (math.sin(ph), math.cos(ph))
    b0 = (LENS[0] - CAM_D / 2 * f[0] - CAM_H / 2 * u[0], LENS[1] - CAM_D / 2 * f[1] - CAM_H / 2 * u[1])   # 카메라 바닥면 중심

    def local(s0, s1, y0, y1, t0, t1):   # 카메라 바닥 기준 (s = 앞 f, t = 위 u)
        return box(s0, s1, y0, y1, t0, t1).rotate((0, 0, 0), (0, 1, 0), PITCH).translate((b0[0], 0, b0[1]))
    b = box(BAR["x0"], BAR["x1"], -BAR["half"], BAR["half"], BAR["z0"], BAR["z1"])
    # 뒤판 받침 (v3): 숙임이 크면(60°) 뒤판 아래 끝이 가로대 윗면보다 높아 뜸 → 뒤판 폭(|y| ≤ BACK_HALF)만 가로대 윗면에서
    #   뒤판 아래 끝까지 채움. 아래 자리 파기로 카메라 바닥면 아래만 남음. x 는 가로대 폭 안 (뒤 98 = 가슴 틈 유지)
    z_back = b0[1] + (CAM_D / 2 + BACK_T) * math.sin(ph) + 3.0 * math.cos(ph)     # 뒤판 아래 끝 가장 높은 z
    if z_back > BAR["z1"] - 2.0:
        b = b.union(box(BAR["x0"], BAR["x1"], -BACK_HALF, BACK_HALF, BAR["z1"] - 1.0, z_back + 1.0))
    b = b.cut(local(-40, 40, -SEAT_HALF, SEAT_HALF, 0, 40))                              # 경사 카메라 자리
    for sgn in (-1, 1):                                                                    # USB-C 플러그·케이블 자리 (양 끝)
        ya, yb = sorted((sgn * USB_CUT["y0"], sgn * USB_CUT["y1"]))
        b = b.cut(local(-USB_CUT["s"], USB_CUT["s"], ya, yb, USB_CUT["t0"], USB_CUT["t1"]))
    b = b.union(local(-CAM_D / 2 - BACK_T, -CAM_D / 2, -BACK_HALF, BACK_HALF, -3, CAM_H))      # 카메라 뒤판
    for yy in (-M3_PITCH / 2, M3_PITCH / 2):                                                      # M3 구멍 (뒷면 가운데 높이)
        b = b.cut(cq.Workplane().add(cq.Solid.makeCylinder(M3_D / 2, BACK_T + 2, cq.Vector(-CAM_D / 2 - BACK_T - 1, yy, CAM_H / 2),
                                                              cq.Vector(1, 0, 0)))
                  .rotate((0, 0, 0), (0, 1, 0), PITCH).translate((b0[0], 0, b0[1])))
        b = b.cut(cq.Workplane().add(cq.Solid.makeCylinder(M3_CB_D / 2, M3_CB_H + 1, cq.Vector(-CAM_D / 2 - BACK_T - 1, yy, CAM_H / 2),
                                                              cq.Vector(1, 0, 0)))
                  .rotate((0, 0, 0), (0, 1, 0), PITCH).translate((b0[0], 0, b0[1])))   # 뒷면 머리 자리
    for sgn in (-1, 1):
        end = sgn * BAR["half"]
        for i, (x, z) in enumerate(JOINT):
            a, c = sorted((end, end - sgn * J_HOLE_DEPTH))
            b = b.cut(cyl_y(BOLT_D / 2, a, c, x, z))
            yn = end - sgn * NUT_FROM_END
            na, nc = sorted((yn, yn - sgn * NUT_T))
            # 너트 홈: 위아래로 뚫린 홈 하나 (위 너트는 윗면, 아래 너트는 아랫면에서 넣음 — 육각 꼭짓점 11.5 가 들어가게)
            b = b.cut(box(x - NUT_AF / 2, x + NUT_AF / 2, na, nc, BAR["z0"] - 1, BAR["z1"] + 1))
    cam = local(-CAM_D / 2, CAM_D / 2, -CAM_W / 2, CAM_W / 2, 0, CAM_H)
    return b, cam, b0


# ---------- 볼트·너트·와셔 (조립 확인용 모델, 나사산 생략) ----------
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


def square_nut(face, d, a=10.0, t=5.0, hole=6.0):
    """사용자 M6.stl 너트: 10 × 10 × 5."""
    return cq.Workplane(_plane(face, d)).rect(a, a).circle(hole / 2).extrude(t)


def hardware(b0):
    """(이름, 모델, 색) 목록 — torso_link mm, 조립 위치."""
    out = []
    steel = cq.Color(0.75, 0.75, 0.78)
    sink = (CSK_D - 11.1) / 2                 # 머리 윗면이 면보다 들어가는 깊이
    for sgn, side in ((1, "L"), (-1, "R")):
        hu, hl = HOLE_U, HOLE_L
        out.append((f"M6x50_csk_up_{side}", csk_screw((PX0 + sink, sgn * hu["y"], hu["z"]), (1, 0, 0), 50.0), steel))
        out.append((f"M6x40_csk_low_{side}", csk_screw((PX0 - LOWER_PAD + sink, sgn * hl["y"], hl["z"]), (1, 0, 0), 40.0), steel))
        for (x, z), tag in zip(JOINT, ("top", "bot")):
            out.append((f"M6x40_csk_{tag}_{side}", csk_screw((x, sgn * (BAND[1] - sink), z), (0, -sgn, 0), 40.0), steel))
            out.append((f"nut_M6_{tag}_{side}", square_nut((x, sgn * (BAR["half"] - NUT_FROM_END), z), (0, -sgn, 0)), steel))
    # 카메라 M3×5 (머리 자리 바닥 → 카메라 쪽으로)
    ph = math.radians(PITCH); f = cq.Vector(math.cos(ph), 0, -math.sin(ph)); u = cq.Vector(math.sin(ph), 0, math.cos(ph))
    for yy in (-M3_PITCH / 2, M3_PITCH / 2):
        seat = cq.Vector(b0[0], yy, b0[1]) + f * (-CAM_D / 2 - BACK_T + M3_CB_H) + u * (CAM_H / 2)
        out.append((f"M3x5_cam_{'L' if yy > 0 else 'R'}", socket_screw((seat.x, seat.y, seat.z), (f.x, f.y, f.z), 3.0, 5.0, 5.5, 3.0, 2.5), steel))
    return out


def to_bed(part, axis, angle):
    p = part.rotate((0, 0, 0), axis, angle) if angle else part
    bb = p.val().BoundingBox()
    return p.translate((-bb.xmin, -bb.ymin, -bb.zmin))


if __name__ == "__main__":
    YL = yoke_left(); YR = YL.mirror("XZ")
    BARP, CAMBOX, B0 = camera_bar()
    parts = {"yoke_L": YL, "yoke_R": YR, "camera_bar": BARP}
    for name, part in list(parts.items()) + [("D435i_dummy", CAMBOX)]:
        v = part.val(); bb = v.BoundingBox()
        print(f"{name:12s} 부피 {v.Volume()/1000:6.1f} cm3  bbox x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")
        cq.exporters.export(part, f"{OUT}/{name}_torso_frame.stl", tolerance=0.05, angularTolerance=0.1)
    prints = {"yoke_L": to_bed(YL, (1, 0, 0), -90), "yoke_R": to_bed(YR, (1, 0, 0), 90), "camera_bar": to_bed(BARP, (0, 0, 1), 0)}
    for name, part in prints.items():
        bb = part.val().BoundingBox()
        print(f"출력 {name:12s} {bb.xlen:5.0f} × {bb.ylen:5.0f} × 높이 {bb.zlen:4.0f} mm")
        cq.exporters.export(part, f"{OUT}/{name}_print.stl", tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(part, f"{OUT}/{name}_print.step")
    # 한 파일 통합: 부품 3 + 카메라 더미 + 볼트·너트·와셔 (torso_link 좌표, mm)
    asm = cq.Assembly(name="h2_cam_bracket")
    asm.add(YL, name="yoke_L", color=cq.Color(0.9, 0.35, 0.2)).add(YR, name="yoke_R", color=cq.Color(0.9, 0.35, 0.2))
    asm.add(BARP, name="camera_bar", color=cq.Color(0.2, 0.5, 0.9)).add(CAMBOX, name="D435i_dummy", color=cq.Color(0.15, 0.15, 0.15))
    hw = hardware(B0)
    for name, part, color in hw:
        asm.add(part, name=name, color=color)
    asm.export(f"{OUT}/h2_cam_bracket_all.step")
    print("하드웨어", len(hw), "개:", ", ".join(n for n, _, _ in hw))
    print("ok")
