#!/usr/bin/env python3
"""
RealSense → HTTP MJPEG 서버
Endpoints:
  /video_feed  : color stream (BGR)
  /depth_feed  : depth visualization (colormap)

원격 PC에서 RealSense를 붙여 실행하고, ik_box.py에서 video_feed URL을 사용.
depth_feed는 디버깅/시각화용.
"""

import threading
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware


# ==========================================
# 전역
# ==========================================
pipeline       = None
align          = None
camera_started = False

latest_color   = None
latest_depth   = None      # raw uint16 (mm)
color_lock     = threading.Lock()
depth_lock     = threading.Lock()


# ==========================================
# 초기화 / 캡처 스레드
# ==========================================
def init_camera():
    global pipeline, align, camera_started

    if camera_started:
        return

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    for _ in range(15):
        pipeline.wait_for_frames()

    camera_started = True
    print("[RS] 카메라 시작됨 (color + depth 640x480 @30fps)")


def capture_loop():
    """RealSense → latest_color/depth"""
    global latest_color, latest_depth
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception:
            continue

        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())  # uint16 mm

        with color_lock:
            latest_color = color_img
        with depth_lock:
            latest_depth = depth_img


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    init_camera()
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    yield
    if pipeline:
        try:
            pipeline.stop()
        except Exception:
            pass


app = FastAPI(title="RealSense MJPEG Stream", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def generate_color_mjpeg():
    while True:
        with color_lock:
            frame = latest_color
        if frame is None:
            time.sleep(0.01)
            continue

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')


def generate_depth_mjpeg():
    while True:
        with depth_lock:
            d = latest_depth
        if d is None:
            time.sleep(0.01)
            continue

        # 0~3m 범위로 정규화 후 JET 컬러맵 적용
        d_clip = np.clip(d, 0, 3000).astype(np.float32)  # 3m 이상은 잘림
        d_norm = (d_clip * (255.0 / 3000.0)).astype(np.uint8)
        d_color = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)
        # 측정 안 된 영역(0)은 검은색으로
        d_color[d == 0] = (0, 0, 0)

        _, buf = cv2.imencode('.jpg', d_color, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')


@app.get("/")
async def index():
    return HTMLResponse("""
        <html><body style='background:#1a1a1a;color:#fff;font-family:monospace;padding:20px'>
        <h2 style='color:#4CAF50'>RealSense MJPEG Stream</h2>
        <p>Endpoints:
          <a href='/video_feed' style='color:#4CAF50'>/video_feed</a> (color),
          <a href='/depth_feed' style='color:#FF9800'>/depth_feed</a> (depth)
        </p>
        <div style='display:flex; gap:20px;'>
            <div>
                <div>Color</div>
                <img src='/video_feed' width='640' height='480' style='border:2px solid #4CAF50'>
            </div>
            <div>
                <div>Depth (0~3m JET)</div>
                <img src='/depth_feed' width='640' height='480' style='border:2px solid #FF9800'>
            </div>
        </div>
        </body></html>
    """)


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_color_mjpeg(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_feed")
async def depth_feed():
    return StreamingResponse(generate_depth_mjpeg(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50001, timeout_graceful_shutdown=2)
