#!/usr/bin/env python3
"""
RealSense color stream → HTTP MJPEG 서버
Endpoint: http://<host>:8002/video_feed

원격 PC에서 RealSense를 붙여 실행하고, ik_box.py에서 이 URL을 읽어 사용.
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
camera_started = False
latest_frame   = None
frame_lock     = threading.Lock()


# ==========================================
# 초기화 / 캡처 스레드
# ==========================================
def init_camera():
    global pipeline, camera_started

    if camera_started:
        return

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    for _ in range(15):
        pipeline.wait_for_frames()

    camera_started = True
    print("[RS] 카메라 시작됨 (color 640x480 @30fps)")


def capture_loop():
    """RealSense → latest_frame (별도 스레드)"""
    global latest_frame
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception:
            continue

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        with frame_lock:
            latest_frame = img


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


def generate_mjpeg():
    while True:
        with frame_lock:
            frame = latest_frame
        if frame is None:
            time.sleep(0.01)
            continue

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')


@app.get("/")
async def index():
    return HTMLResponse("""
        <html><body style='background:#1a1a1a;color:#fff;font-family:monospace;padding:20px'>
        <h2 style='color:#4CAF50'>RealSense MJPEG Stream</h2>
        <p>Endpoint: <a href='/video_feed' style='color:#4CAF50'>/video_feed</a></p>
        <img src='/video_feed' width='640' height='480' style='border:2px solid #4CAF50'>
        </body></html>
    """)


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_mjpeg(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8002)
