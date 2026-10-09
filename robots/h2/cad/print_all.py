#!/usr/bin/env python3
"""
H2 출력 부품 전부를 STEP 하나로 (출력 방향 그대로, x 로 10 mm 간격 나란히 — 슬라이서에서 배치만 다시).
  camera_bracket/print/h2_cam_bracket_print_all.step (yoke_L, yoke_R, camera_bar) + back_handle/print/handle_print.step

  pip install cadquery
  python print_all.py            # → h2_print_all.step (이 폴더)
부품을 다시 만든 뒤(bracket.py out / handle.py out → print/ 에 복사) 이것도 다시 실행.
"""
import os
import cadquery as cq

D = os.path.dirname(os.path.abspath(__file__))
SRC = [os.path.join(D, "camera_bracket/print/h2_cam_bracket_print_all.step"),
       os.path.join(D, "back_handle/print/handle_print.step")]
NAMES = ["yoke_L", "yoke_R", "camera_bar", "back_handle"]

solids = []
for p in SRC:
    solids += sorted(cq.importers.importStep(p).solids().vals(), key=lambda v: v.BoundingBox().xmin)
assert len(solids) == len(NAMES), f"부품 수 {len(solids)} (기대 {len(NAMES)})"
asm, x = cq.Assembly(name="h2_print_all"), 0.0
for name, v in zip(NAMES, solids):
    bb = v.BoundingBox()
    asm.add(cq.Workplane().add(v).translate((x - bb.xmin, -bb.ymin, -bb.zmin)), name=name)
    print(f"{name:12s} {bb.xlen:5.0f} × {bb.ylen:5.0f} × 높이 {bb.zlen:4.0f} mm  부피 {v.Volume() / 1000:6.1f} cm3")
    x += bb.xlen + 10.0
asm.export(os.path.join(D, "h2_print_all.step"))
print("ok → h2_print_all.step")
