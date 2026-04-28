#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
pipeline.start(config)

print("D435i raw accel (카메라 고정 상태에서 확인)")
print(f"{'ax':>8} {'ay':>8} {'az':>8}  | 각 축 pitch 계산")
print("-" * 60)

while True:
    frames = pipeline.wait_for_frames()
    accel_frame = frames.first_or_default(rs.stream.accel)
    if not accel_frame:
        continue
    a = accel_frame.as_motion_frame().get_motion_data()
    ax, ay, az = a.x, a.y, a.z

    p1 = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
    p2 = np.degrees(np.arctan2( ay, np.sqrt(ax**2 + az**2)))
    p3 = np.degrees(np.arctan2( az, np.sqrt(ax**2 + ay**2)))
    p4 = np.degrees(np.arctan2(-az, np.sqrt(ax**2 + ay**2)))

    print(f"{ax:>8.3f} {ay:>8.3f} {az:>8.3f}  | "
          f"p1={p1:>7.2f} p2={p2:>7.2f} p3={p3:>7.2f} p4={p4:>7.2f}")

    import time; time.sleep(0.3)
