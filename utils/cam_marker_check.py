#!/usr/bin/env python3
"""
cam_marker_check.py — ArUco 마커로 카메라 장착값(robot.yaml camera) 확인 (카메라만 사용, 로봇 명령 없음)

  마커: DICT_4X4_50, 한 변 45 mm (detect_marker.py 와 같음 — --size 로 변경 가능, 검은 테두리 바깥 기준이 아닌 검은 사각형 한 변)
  rs_stream.py 등 카메라를 쓰는 프로그램은 먼저 끌 것.

  ROBOT=h2 source activate_tv.sh
  ROBOT=h2 python utils/cam_marker_check.py              # 3초 중앙값, 1회
  ROBOT=h2 python utils/cam_marker_check.py --watch      # 계속 출력
  ROBOT=h2 python utils/cam_marker_check.py --points 0.35,0.45   # 2점 보정: 같은 테이블 면, 몸통 중심에서 앞 거리 [m]

  마커를 '수평인 테이블 위'에 평평하게 놓고 실행하면 세 가지를 출력한다.
   1) 렌즈 기준 수평 좌표 — 줄자로 바로 비교할 값
        앞 : 렌즈 바로 아래 지점에서 마커 중심까지 앞쪽 수평 거리
        좌 : 왼쪽 + (로봇 기준)
        아래: 렌즈 높이 − 마커 높이 (= 바닥→렌즈 − 바닥→테이블 윗면)
      robot.yaml pitch_deg 를 써서 계산 → 줄자와 다르면 pitch 또는 거리 오차
   2) 마커 면으로 추정한 카메라 숙임각 / 좌우 기울기 (테이블이 수평일 때) — robot.yaml pitch_deg 와 비교
   3) torso_link / IK(pelvis) 좌표 — 잡기에서 실제로 쓰는 값 (robot_server camera_to_torso 와 같은 식)

  --points: 같은 테이블 면에서 몸통 중심(torso_link x 0)으로부터 앞 거리를 아는 위치 2곳 이상에 차례로 마커를 놓고 측정
    → '모든 점의 높이가 같다' 조건으로 숙임각, '앞 거리가 맞다' 조건으로 camera.x 를 계산 (테이블 높이는 몰라도 됨)
    → 같은 순간 IMU 숙임각을 썼을 때의 결과도 함께 출력
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
    ap.add_argument("--id", type=int, default=None, help="이 ID 만 사용 (기본: 화면에서 가장 큰 마커)")
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--points", default=None, help="2점 보정: 몸통 중심에서 마커까지 앞 거리 목록 [m], 예: 0.35,0.45")
    a = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.accel)          # D435i IMU — 같은 순간의 숙임각 (cam_tilt.py 와 같은 계산)
    try:
        prof = pipe.start(cfg)
    except RuntimeError as e:
        # IMU(HID/iio) 권한 없음 등 — 컬러만으로 계속 (IMU 줄은 생략). 해결: sudo 실행 또는 RealSense udev 규칙
        print(f"[marker] ⚠️ IMU 스트림 열기 실패 → 컬러만 사용 ({e})")
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

    seen = set()

    def pick(corners, ids):
        """--id 가 있으면 그 ID, 없으면 화면에서 가장 큰(= 가장 가까운) 마커. 처음 보는 ID 는 알려 준다."""
        best, best_a = (None, None), -1.0
        for c, i in zip(corners, ids.flatten()):
            if int(i) not in seen:
                seen.add(int(i))
                print(f"  (화면에 마커 ID {int(i)} 보임)")
            if a.id is not None and i != a.id:
                continue
            area = cv2.contourArea(c.reshape(4, 2).astype(np.float32))
            if area > best_a:
                best, best_a = (c, i), area
        return best

    def measure(sec):
        """sec 동안 마커 tvec·법선 중앙값 + IMU 숙임각 평균 → (t, n, imu_pitch_rad|None, id, 개수)."""
        ts, ns, acc = [], [], []
        t_end = time.time() + sec
        mid = None
        while time.time() < t_end:
            fs = pipe.wait_for_frames(1000)
            af = fs.first_or_default(rs.stream.accel)
            if af:
                d_ = af.as_motion_frame().get_motion_data()
                acc.append((d_.x, d_.y, d_.z))
            f = fs.get_color_frame()
            if not f:
                continue
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(np.asanyarray(f.get_data()), cv2.COLOR_BGR2GRAY))
            if ids is None:
                continue
            c, i = pick(corners, ids)
            if c is None:
                continue
            ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                ts.append(tvec.ravel())
                ns.append(cv2.Rodrigues(rvec)[0][:, 2])
                mid = int(i)
        if not ts:
            return None
        t = np.median(np.array(ts), axis=0)
        n = np.median(np.array(ns), axis=0)
        n /= np.linalg.norm(n)
        if n[1] > 0:
            n = -n
        ip = None
        if acc:
            g = np.mean(np.array(acc), axis=0)
            ip = math.asin(min(1.0, abs(g[2]) / float(np.linalg.norm(g))))
        return t, n, ip, mid, len(ts)

    def fwd_down(t, p):
        return t[2] * math.cos(p) - t[1] * math.sin(p), t[2] * math.sin(p) + t[1] * math.cos(p)

    if a.points:
        xs = [float(v) for v in a.points.split(",")]
        meas = []
        try:
            for i, xt in enumerate(xs):
                input(f"\n[{i + 1}/{len(xs)}] 마커를 몸통 중심에서 앞 {xt * 100:.0f} cm (같은 테이블 면) 에 놓고 Enter ")
                r = measure(a.sec)
                if r is None:
                    print("  마커 안 보임 — 중단")
                    return
                t, n, ip, mid, cnt = r
                f0, d0 = fwd_down(t, CP)
                print(f"  ID {mid} n={cnt}  yaml pitch: torso x {f0 + CX:.3f} z {CZ - d0:+.3f} y {-t[0] + CY:+.3f}"
                      f"   IMU 숙임각 {math.degrees(ip) if ip else float('nan'):.1f}°")
                meas.append((xt, t, ip))
        finally:
            pipe.stop()
        ts_ = [m[1] for m in meas]
        # 숙임각: 모든 점의 '아래' 가 같아지는 값 (최소제곱)
        grid = np.radians(np.arange(10.0, 70.0, 0.05))
        spread = [np.std([fwd_down(t, p)[1] for t in ts_]) for p in grid]
        p_fit = float(grid[int(np.argmin(spread))])
        print("\n== 2점 보정 결과 ==")
        for label, p in [("높이 일치 조건으로 구한 숙임각", p_fit),
                         ("IMU 숙임각(평균)", float(np.mean([m[2] for m in meas if m[2] is not None])) if all(m[2] for m in meas) else None),
                         ("robot.yaml 숙임각", CP)]:
            if p is None:
                continue
            fd = [fwd_down(t, p) for t in ts_]
            cx = [xt - f for (xt, _, _), (f, _) in zip(meas, fd)]
            downs = [d for _, d in fd]
            print(f"  {label} {math.degrees(p):5.1f}°: camera.x 필요값 " + " / ".join(f"{v:.3f}" for v in cx) +
                  f"  (평균 {np.mean(cx):.3f}),  렌즈→테이블 아래 " + " / ".join(f"{v:.3f}" for v in downs) +
                  f"  (차이 {max(downs) - min(downs):.3f})")
        print("  · 같은 숙임각에서 camera.x 필요값이 점마다 같고, 아래 차이가 1 cm 안이면 그 숙임각이 맞음")
        print("  · camera.z 는 렌즈→테이블 아래 값 + (바닥→테이블) 과 (바닥→torso 원점) 실측이 있어야 정해짐")
        return

    try:
        while True:
            ts, ns, acc = [], [], []
            t_end = time.time() + (1.0 if a.watch else a.sec)
            mid = None
            while time.time() < t_end:
                fs = pipe.wait_for_frames(1000)
                af = fs.first_or_default(rs.stream.accel)
                if af:
                    d_ = af.as_motion_frame().get_motion_data()
                    acc.append((d_.x, d_.y, d_.z))
                f = fs.get_color_frame()
                if not f:
                    continue
                img = np.asanyarray(f.get_data())
                corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
                if ids is None:
                    continue
                c, i = pick(corners, ids)
                if c is not None:
                    ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, dist,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        R, _ = cv2.Rodrigues(rvec)
                        ts.append(tvec.ravel())
                        ns.append(R[:, 2])
                        mid = int(i)
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
            # 1') 같은 순간 D435i IMU 숙임각 (중력 기준) 으로 계산한 렌즈 기준 좌표
            imu_line = "    IMU: 가속도 없음"
            if acc:
                g = np.mean(np.array(acc), axis=0)
                ip = math.asin(min(1.0, abs(g[2]) / float(np.linalg.norm(g))))
                f2 = t[2] * math.cos(ip) - t[1] * math.sin(ip)
                d2 = t[2] * math.sin(ip) + t[1] * math.cos(ip)
                imu_line = (f"    1') IMU 숙임각 {math.degrees(ip):.1f}° 로 계산: 앞 {f2:.3f}  아래 {d2:.3f} m "
                            f"→ torso x {f2 + CX:.3f} z {CZ - d2:+.3f}")
            # 3) torso / IK(pelvis) 좌표
            tor = camera_to_torso(t)
            ik = tor + P2T

            print(f"  ID {mid}  n={len(ts)}  거리 {np.linalg.norm(t):.3f} m")
            print(f"    1) 렌즈 기준 (pitch {math.degrees(CP):.1f}° 사용): 앞 {fwd:.3f}  좌 {left:+.3f}  아래 {down:.3f} m")
            print(imu_line)
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
