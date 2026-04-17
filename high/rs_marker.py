import cv2
import uvicorn
import time
import threading
import numpy as np
import math
import pyrealsense2 as rs  # [변경] 리얼센스 라이브러리 추가
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
from contextlib import asynccontextmanager
import os

# ---------------------------------------------------------
# [설정] 튜닝값 및 마커 크기
# ---------------------------------------------------------
MARKER_SIZE = 0.08       # 8cm
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# RealSense 해상도 및 FPS 설정
WIDTH = 640
HEIGHT = 480
FPS = 30

# ---------------------------------------------------------
# 전역 변수
# ---------------------------------------------------------
pipeline = None          # [변경] cv2.VideoCapture 대신 pipeline 사용
config = None
align = None             # Depth와 Color 정렬용 (필요시 사용)
camera_matrix = None     # [변경] 리얼센스 내부 파라미터로 자동 설정됨
dist_coeffs = np.zeros((5, 1)) # 리얼센스 이미지는 보통 보정되어 들어오므로 0으로 시작

aruco_dict = None
aruco_params = None
detector = None
lock = threading.Lock()

# USB 권한 설정 (기존 유지)
os.system('sudo chown unitree:unitree /dev/ttyACM0')

def init_resources():
    global pipeline, config, align, camera_matrix, aruco_dict, aruco_params, detector

    # 1. ArUco 리소스 초기화
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        detector = None

    # 2. RealSense 파이프라인 초기화
    print("Opening RealSense Camera...")
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        
        # RGB 스트림 활성화
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        
        # 스트리밍 시작
        profile = pipeline.start(config)
        
        # [핵심] 리얼센스 하드웨어로부터 직접 Intrinsic(카메라 매트릭스) 추출
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        # RealSense Intrinsics -> OpenCV Camera Matrix 변환
        # [[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]]
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # 왜곡 계수 (RealSense RGB는 보통 보정된 이미지를 주지만, 필요시 intrinsics.coeffs 사용 가능)
        # 여기서는 기본 0으로 설정합니다.
        global dist_coeffs
        dist_coeffs = np.array(intrinsics.coeffs) if len(intrinsics.coeffs) == 5 else np.zeros((5, 1))

        print(f"✅ RealSense Ready. Intrinsics Loaded: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
        
    except Exception as e:
        print(f"❌ RealSense Open Failed: {e}")
        pipeline = None

def release_resources():
    global pipeline
    if pipeline:
        pipeline.stop()
        print("RealSense Pipeline Stopped.")

@asynccontextmanager
async def lifespan(app: FastAPI):
    init_resources()
    yield
    release_resources()

app = FastAPI(lifespan=lifespan)

# ---------------------------------------------------------
# [핵심] 정면 판별 및 그리기 로직
# ---------------------------------------------------------
def is_facing_camera(rvec):
    """
    마커가 카메라를 정면으로 보고 있는지 각도를 계산합니다.
    Return: (정면여부 Bool, 각도 Degree)
    """
    R, _ = cv2.Rodrigues(rvec)
    marker_z_axis = R[:, 2]
    camera_optical_axis = np.array([0, 0, 1])
    
    dot_prod = np.dot(marker_z_axis, camera_optical_axis)
    dot_prod = max(-1.0, min(1.0, dot_prod))
    
    angle_rad = np.arccos(-dot_prod)
    angle_deg = np.degrees(angle_rad)

    is_front = angle_deg < 10.0
    return is_front, angle_deg

def process_frame(frame):
    # [변경] 기존에는 frame[:, :w//2]로 잘랐으나, RealSense는 단일 RGB 이미지이므로 그대로 사용
    process_image = frame.copy() 
    gray = cv2.cvtColor(process_image, cv2.COLOR_BGR2GRAY)

    if detector:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None and camera_matrix is not None:
        # Pose Estimation
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # 1. 정면 판별
            is_front, angle = is_facing_camera(rvec)

            # 2. 시각화 (축 & 마커)
            cv2.drawFrameAxes(process_image, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.aruco.drawDetectedMarkers(process_image, corners, ids)

            # 3. 좌표 및 상태 정보 텍스트
            x, y, z = tvec[0], tvec[1], tvec[2]

            status_color = (0, 255, 0) if is_front else (0, 0, 255)
            status_text = "FRONT" if is_front else f"ANGLE: {angle:.1f}"

            c = corners[i][0]
            cx_marker = int(np.mean(c[:, 0]))
            cy_marker = int(np.mean(c[:, 1]))

            coord_str = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"
            cv2.putText(process_image, coord_str, (cx_marker - 60, cy_marker - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.putText(process_image, status_text, (cx_marker - 60, cy_marker - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

    return process_image

def generate_frames():
    global pipeline
    while True:
        with lock:
            if pipeline is None:
                time.sleep(0.1)
                continue
            
            # [변경] RealSense 프레임 대기 및 획득
            try:
                frames = pipeline.wait_for_frames(timeout_ms=2000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                # RealSense 이미지를 numpy 배열로 변환
                frame = np.asanyarray(color_frame.get_data())
                
            except RuntimeError:
                # 프레임 획득 실패 시 재시도
                continue

        # 이미지 처리
        final_image = process_frame(frame)

        ret, buffer = cv2.imencode('.jpg', final_image)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/")
async def index():
    html_content = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>ArUco Pose (RealSense)</title>
        <style>
            body { background: #111; color: #fff; text-align: center; font-family: sans-serif; }
            h1 { color: #00d2ff; }
            img { border: 2px solid #555; width: 80%; max-width: 800px; }
            .desc { margin: 10px; color: #aaa; }
        </style>
    </head>
    <body>
        <h1>ArUco Tracking (Intel RealSense)</h1>
        <div class="desc">
            Detects: 8cm Marker<br>
            Using: RealSense RGB Stream & Hardware Intrinsics
        </div>
        <img src="/video_feed">
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=5000)
