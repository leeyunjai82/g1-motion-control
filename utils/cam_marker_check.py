#!/usr/bin/env python3
"""
cam_marker_check.py — ArUco 마커로 카메라 장착값(robot.yaml camera) 확인 (카메라만 사용, 로봇 명령 없음)

  마커: DICT_4X4_50, 한 변 45 mm (detect_marker.py 와 같음 — --size 로 변경 가능, 검은 테두리 바깥 기준이 아닌 검은 사각형 한 변)
  rs_stream.py 등 카메라를 쓰는 프로그램은 먼저 끌 것.

  ROBOT=h2 source activate_tv.sh
  ROBOT=h2 python utils/cam_marker_check.py              # 3초 중앙값, 1회
  ROBOT=h2 python utils/cam_marker_check.py --watch      # 계속 출력

  마커를 '수평인 테이블 위'에 평평하게 놓고 실행하면 세 가지를 출력한다.
   1) 렌즈 기준 수평 좌표 — 줄자로 바로 비교할 값
        앞 : 렌즈 바로 아래 지점에서 마커 중심까지 앞쪽 수평 거리
        좌 : 왼쪽 + (로봇 기준)
        아래: 렌즈 높이 − 마커 높이 (= 바닥→렌즈 − 바닥→테이블 윗면)
      robot.yaml pitch_deg 를 써서 계산 → 줄자와 다르면 pitch 또는 거리 오차
   2) 마커 면으로 추정한 카메라 숙임각 / 좌우 기울기 (테이블이 수평일 때) — robot.yaml pitch_deg 와 비교
   3) torso_link / IK(pelvis) 좌표 — 잡기에서 실제로 쓰는 값 (robot_server camera_to_torso 와 같은 식)
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT 미지정이면 종료 — robot.yaml camera / frames 사용

import cv2
import numpy as np
import pyrealsense2 as rs

CX, CY, CZ, CP = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH
P2T = np.array(robot_env.CFG["frames"]["pelvis_to_torso"], dtype=float)


def camera_to_torso(c):
    """robot_server.py camera_to_torso 와 같은 식 (카메라 광학 좌표 x 오른쪽, y 아래, z 앞)."""
    cx, cy, cz = c
    cos_p, sin_p = math.cos(CP), math.sin(CP)
    cy_r = cy * cos_p + cz * sin_p
    cz_r = -cy * sin_p + cz * cos_p
    return np.array([cz_r + CX, -cx + CY, -cy_r + CZ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=0.045, help="마커 검은 사각형 한 변 [m] (기본 0.045)")
    ap.add_argument("--id", type=int, default=None, help="이 ID 만 사용 (기본: 처음 보이는 것)")
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--watch", action="store_true")
    a = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    dist = np.array(intr.coeffs[:5], dtype=np.float64)
    print(f"[marker] 카메라 K: fx {intr.fx:.1f} fy {intr.fy:.1f} ppx {intr.ppx:.1f} ppy {intr.ppy:.1f} "
          f"(detect_box.py 고정값 606.8 / 606.6 / 316.7 / 259.0)")
    print(f"[marker] robot.yaml camera: x {CX} y {CY} z {CZ} pitch {math.degrees(CP):.1f}°  "
          f"pelvis_to_torso {P2T.tolist()}  마커 {a.size * 1000:.0f} mm")

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    h = a.size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    try:
        while True:
            ts, ns = [], []
            t_end = time.time() + (1.0 if a.watch else a.sec)
            mid = None
            while time.time() < t_end:
                f = pipe.wait_for_frames(1000).get_color_frame()
                if not f:
                    continue
                img = np.asanyarray(f.get_data())
                corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
                if ids is None:
                    continue
                for c, i in zip(corners, ids.flatten()):
                    if a.id is not None and i != a.id:
                        continue
                    ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, dist,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        R, _ = cv2.Rodrigues(rvec)
                        ts.append(tvec.ravel())
                        ns.append(R[:, 2])
                        mid = int(i)
                    break
            if not ts:
                print("  마커 안 보임")
                if not a.watch:
                    break
                continue
            t = np.median(np.array(ts), axis=0)
            n = np.median(np.array(ns), axis=0)
            n /= np.linalg.norm(n)
            if n[1] > 0:                      # 마커 면 법선을 '위쪽'(카메라 y 는 아래)으로
                n = -n

            # 1) 렌즈 기준 수평 좌표 (robot.yaml pitch 사용)
            cos_p, sin_p = math.cos(CP), math.sin(CP)
            fwd = t[2] * cos_p - t[1] * sin_p
            down = t[2] * sin_p + t[1] * cos_p
            left = -t[0]
            # 2) 마커 면(수평 가정)으로 추정한 카메라 숙임각 / 좌우 기울기
            pitch_est = math.degrees(math.atan2(-n[2], -n[1]))
            roll_est = math.degrees(math.asin(max(-1.0, min(1.0, n[0]))))
            # 3) torso / IK(pelvis) 좌표
            tor = camera_to_torso(t)
            ik = tor + P2T

            print(f"  ID {mid}  n={len(ts)}  거리 {np.linalg.norm(t):.3f} m")
            print(f"    1) 렌즈 기준 (pitch {math.degrees(CP):.1f}° 사용): 앞 {fwd:.3f}  좌 {left:+.3f}  아래 {down:.3f} m")
            print(f"    2) 마커 면으로 본 카메라 숙임각 {pitch_est:.1f}° (yaml {math.degrees(CP):.1f}°), 좌우 기울기 {roll_est:+.1f}°")
            print(f"    3) torso  x {tor[0]:.3f} y {tor[1]:+.3f} z {tor[2]:+.3f}   |   IK(pelvis) x {ik[0]:.3f} y {ik[1]:+.3f} z {ik[2]:+.3f}")
            if not a.watch:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
