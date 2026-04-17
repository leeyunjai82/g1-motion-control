#!/usr/bin/env python3
"""
Unitree G1 + RealSense D435i 웹 스트리밍 + 클릭 → IK 로봇 제어 + ArUco 마커 감지
수정: 마커는 2D 중심점만 추출하고, 실제 거리/좌표는 RealSense Depth 사용
수정: UI에서 왼손/오른손 RPY 설정 분리 유지
"""

import os
import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse

# ==========================================
# 로봇 라이브러리 로드
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

try:
    from lib.arm_controller_wrapper import ArmControllerWrapper
    ROBOT_AVAILABLE = True
    print("[시스템] 로봇 라이브러리 로드 성공")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"[경고] 로봇 라이브러리 없음: {e}")
    print("[경고] 시뮬레이션 모드로 실행")

app = FastAPI(title="G1 RealSense IK & Marker")

# ==========================================
# ArUco & 카메라 설정
# ==========================================
MARKER_SIZE = 0.035        # 8cm (시각화용)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# 전역 변수
pipeline = None
align = None
intrinsics = None      # RealSense Intrinsic (Deprojection용)
camera_matrix = None   # ArUco용 Matrix
dist_coeffs = None     # ArUco용 왜곡계수

latest_depth = None
latest_markers = []    # [{'id':, 'corners':, 'cx':, 'cy':}, ...] (2D 좌표 저장)
lock = threading.Lock()
camera_started = False
arm = None

# ArUco 탐지기
aruco_dict = None
aruco_params = None
aruco_detector = None

# ============================================
# URDF 기반 카메라 설정 (torso_link 기준)
# ============================================
CAMERA_X = 0.0576235
CAMERA_Y = 0.01753
CAMERA_Z = 0.42987
CAMERA_PITCH = 0.8307767239493009 # 약 47.6도

# 홈 위치
HOME_LEFT = [0.2, 0.2, 0.2]
HOME_RIGHT = [0.2, -0.2, 0.2]

# IK 기본 설정
DEFAULT_DURATION = 3.0


def init_camera():
    global pipeline, align, intrinsics, camera_started
    global camera_matrix, dist_coeffs, aruco_dict, aruco_params, aruco_detector

    if camera_started:
        return

    # 1. ArUco 초기화
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_detector = None

    # 2. RealSense 파이프라인 시작
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    # 3. Intrinsics 추출
    color_stream = profile.get_stream(rs.stream.color)
    intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

    # RealSense Intrinsics -> OpenCV Camera Matrix 변환 (시각화용)
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ], dtype=np.float32)

    dist_coeffs = np.array(intrinsics.coeffs) if len(intrinsics.coeffs) == 5 else np.zeros((5, 1))

    # 워밍업
    for _ in range(30):
        pipeline.wait_for_frames()

    camera_started = True
    print("=" * 50)
    print("카메라 시작됨")
    print(f"카메라 위치 (torso): [{CAMERA_X:.4f}, {CAMERA_Y:.4f}, {CAMERA_Z:.4f}]")
    print("=" * 50)


def init_robot():
    global arm
    if not ROBOT_AVAILABLE:
        print("[로봇] 시뮬레이션 모드")
        return
    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("[로봇] 초기화 완료")
    except Exception as e:
        print(f"[로봇] 초기화 실패: {e}")


def pixel_to_torso_coords(u, v, depth_mm):
    """
    2D 픽셀(u, v) + Depth -> 3D Camera Coords -> 3D Robot Torso Coords
    RealSense Intrinsics를 사용하여 정확한 3D 복원 수행
    """
    depth_m = depth_mm / 1000.0

    # 1. Deprojection (Pixel -> 3D Camera Point)
    # 수동 계산 (intrinsics 사용)
    cam_x = (u - intrinsics.ppx) * depth_m / intrinsics.fx
    cam_y = (v - intrinsics.ppy) * depth_m / intrinsics.fy
    cam_z = depth_m

    # 2. Transform (Camera -> Torso)
    cos_p = np.cos(CAMERA_PITCH)
    sin_p = np.sin(CAMERA_PITCH)

    # Pitch 회전 (카메라 좌표계 기준 X축 회전)
    cam_x_rot = cam_x
    cam_y_rot = cam_y * cos_p + cam_z * sin_p
    cam_z_rot = -cam_y * sin_p + cam_z * cos_p

    # 축 변환 및 오프셋 적용
    # Cam Z(전방) -> Robot X
    # Cam X(우측) -> Robot -Y
    # Cam Y(하방) -> Robot -Z
    torso_x = cam_z_rot + CAMERA_X
    torso_y = -cam_x_rot + CAMERA_Y
    torso_z = -cam_y_rot + CAMERA_Z

    return torso_x, torso_y, torso_z


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    import pinocchio as pin
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return pin.Quaternion(w, x, y, z).normalized()


# ==========================================
# ArUco 감지 로직
# ==========================================
def is_facing_camera(rvec):
    R, _ = cv2.Rodrigues(rvec)
    marker_z_axis = R[:, 2]
    camera_optical_axis = np.array([0, 0, 1])
    dot_prod = np.dot(marker_z_axis, camera_optical_axis)
    dot_prod = max(-1.0, min(1.0, dot_prod))
    angle_rad = np.arccos(-dot_prod)
    return np.degrees(angle_rad) < 10.0


def detect_and_draw_aruco(image):
    """
    마커를 찾아 2D 중심점(cx, cy)을 저장하고 화면에 시각화만 수행
    실제 좌표 계산은 여기서 하지 않음
    """
    global latest_markers

    if camera_matrix is None:
        return image

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if aruco_detector:
        corners, ids, rejected = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    current_markers = []

    if ids is not None:
        # 시각화를 위한 Pose Estimation (좌표 계산에는 사용 안 함)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            # 중심점 계산 (2D Pixel)
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            # 마커 정보 저장 (중심점 포함)
            current_markers.append({
                'id': int(ids[i][0]),
                'corners': corners[i][0],
                'cx': cx,
                'cy': cy
            })

            # 시각화
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            # 정면 여부 확인 (시각화용)
            is_front = is_facing_camera(rvec)
            status_color = (0, 255, 0) if is_front else (0, 0, 255)

            cv2.putText(image, f"ID:{ids[i][0]}", (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    with lock:
        latest_markers = current_markers

    return image


def generate_frames():
    global latest_depth

    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception:
            continue

        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        with lock:
            latest_depth = np.asanyarray(depth_frame.get_data())

        color_image = np.asanyarray(color_frame.get_data())

        # ArUco 마커 그리기
        color_image = detect_and_draw_aruco(color_image)

        _, buffer = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>G1 RealSense IK & Marker</title>
    <style>
        body { font-family: monospace; background: #1a1a1a; color: #fff; margin: 20px; }
        h1 { color: #4CAF50; margin-bottom: 5px; }
        .subtitle { color: #888; margin-bottom: 20px; }
        #container { display: flex; gap: 20px; }
        #video-wrapper { position: relative; cursor: crosshair; }
        #stream { border: 2px solid #4CAF50; }
        #coords { background: #2a2a2a; padding: 20px; border-radius: 8px; min-width: 400px; }
        .section-title { color: #4CAF50; border-bottom: 1px solid #444; padding-bottom: 5px; margin-bottom: 15px; }
        .coord-row { margin: 12px 0; font-size: 16px; }
        .label { color: #888; display: inline-block; width: 150px; }
        .value { color: #4CAF50; font-weight: bold; font-size: 18px; }
        .unit { color: #666; font-size: 14px; }
        #click-marker {
            position: absolute; width: 20px; height: 20px;
            border: 2px solid #ff0; border-radius: 50%;
            pointer-events: none; display: none;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 10px rgba(255,255,0,0.5);
        }
        .btn {
            border: none; color: white;
            padding: 12px 24px; border-radius: 4px; cursor: pointer;
            margin: 5px; font-size: 14px; font-weight: bold;
        }
        .btn-move { background: #2196F3; }
        .btn-move:hover { background: #1976D2; }
        .btn-move:disabled { background: #666; cursor: not-allowed; }
        .btn-home { background: #FF9800; }
        .btn-home:hover { background: #F57C00; }
        #ik-output {
            background: #000; padding: 12px; border-radius: 4px;
            margin-top: 15px; font-size: 15px; color: #0f0;
        }
        #status {
            background: #333; padding: 10px; border-radius: 4px;
            margin-top: 10px; font-size: 14px;
        }
        .status-ready { color: #4CAF50; }
        .status-moving { color: #FF9800; }
        .status-error { color: #f44336; }
        #history { margin-top: 20px; max-height: 150px; overflow-y: auto; }
        .history-item {
            background: #333; padding: 6px 10px; margin: 4px 0;
            border-radius: 4px; font-size: 12px; border-left: 3px solid #4CAF50;
        }
        .param-input {
            background: #333; border: 1px solid #555; color: #fff;
            padding: 8px; border-radius: 4px; width: 60px; text-align: center;
        }
        .param-row { margin: 10px 0; }
        .param-label { color: #888; display: inline-block; width: 150px; }
        .hand-config { border-bottom: 1px solid #444; padding-bottom: 10px; margin-bottom: 10px; }
    </style>
</head>
<body>
    <h1>G1 RealSense IK & Marker Control</h1>
    <div class="subtitle">마커 클릭 -> RealSense 깊이로 좌표 계산 -> 로봇 이동</div>

    <div id="container">
        <div id="video-wrapper">
            <img id="stream" src="/video_feed" width="640" height="480" onclick="getCoords(event)">
            <div id="click-marker"></div>
        </div>

        <div id="coords">
            <h3 class="section-title">torso_link 기준 좌표</h3>
            <div class="coord-row">
                <span class="label">X (전방):</span>
                <span class="value" id="torso-x">-</span>
                <span class="unit">m</span>
            </div>
            <div class="coord-row">
                <span class="label">Y (좌측):</span>
                <span class="value" id="torso-y">-</span>
                <span class="unit">m</span>
            </div>
            <div class="coord-row">
                <span class="label">Z (위):</span>
                <span class="value" id="torso-z">-</span>
                <span class="unit">m</span>
            </div>
            <div class="coord-row">
                <span class="label">RealSense 깊이:</span>
                <span class="value" id="depth">-</span>
                <span class="unit">m</span>
            </div>

            <h3 class="section-title" style="margin-top:25px;">로봇 제어 설정</h3>
            <div id="ik-output">Target: -</div>
            <div style="margin-top:10px; font-size:18px;">
                <span class="label">감지된 위치:</span>
                <span class="value" id="arm-side">-</span>
            </div>

            <div class="param-row">
                <span class="param-label">Duration (초):</span>
                <input type="number" id="duration" class="param-input" value="3.0" min="0.5" max="10" step="0.5">
            </div>

            <div class="param-row">
                <span class="param-label">Offset X,Y,Z (m):</span>
                <input type="number" id="offset-x" class="param-input" value="0" step="0.01">
                <input type="number" id="offset-y" class="param-input" value="0" step="0.01">
                <input type="number" id="offset-z" class="param-input" value="0" step="0.01">
            </div>

            <!-- 왼손 설정 -->
            <div class="hand-config">
                <div class="param-row">
                    <span class="param-label" style="color:#2196F3">왼손 R,P,Y (도):</span>
                    <input type="number" id="left-roll" class="param-input" value="0" step="5">
                    <input type="number" id="left-pitch" class="param-input" value="0" step="5">
                    <input type="number" id="left-yaw" class="param-input" value="0" step="5">
                </div>
            </div>

            <!-- 오른손 설정 -->
            <div class="hand-config">
                <div class="param-row">
                    <span class="param-label" style="color:#FF9800">오른손 R,P,Y (도):</span>
                    <input type="number" id="right-roll" class="param-input" value="0" step="5">
                    <input type="number" id="right-pitch" class="param-input" value="0" step="5">
                    <input type="number" id="right-yaw" class="param-input" value="0" step="5">
                </div>
            </div>

            <div style="margin-top:15px;">
                <button class="btn btn-move" id="btn-move" onclick="moveRobot()" disabled>이동</button>
                <button class="btn btn-home" onclick="goHome()">홈</button>
            </div>

            <div id="status" class="status-ready">상태: 대기 중</div>

            <div id="history">
                <h3 class="section-title">이동 기록</h3>
                <div id="history-list"></div>
            </div>
        </div>
    </div>

    <script>
        let lastCoords = null;

        function getCoords(event) {
            const img = document.getElementById('stream');
            const rect = img.getBoundingClientRect();
            const u = Math.round(event.clientX - rect.left);
            const v = Math.round(event.clientY - rect.top);

            const marker = document.getElementById('click-marker');
            marker.style.left = u + 'px';
            marker.style.top = v + 'px';
            marker.style.display = 'block';

            fetch('/get_coords?u=' + u + '&v=' + v)
                .then(r => r.json())
                .then(data => {
                    if (data.error) {
                        setStatus(data.error, 'error');
                        return;
                    }

                    lastCoords = data;

                    document.getElementById('torso-x').textContent = data.torso_x.toFixed(4);
                    document.getElementById('torso-y').textContent = data.torso_y.toFixed(4);
                    document.getElementById('torso-z').textContent = data.torso_z.toFixed(4);
                    document.getElementById('depth').textContent = data.depth_m.toFixed(3);

                    let label = data.marker_id !== null ? 'Marker ID: ' + data.marker_id : 'Click Point';
                    document.getElementById('ik-output').textContent =
                        label + ' [' + data.torso_x.toFixed(3) + ', ' +
                        data.torso_y.toFixed(3) + ', ' + data.torso_z.toFixed(3) + ']';

                    const armSide = data.torso_y >= 0 ? '왼쪽 영역' : '오른쪽 영역';
                    document.getElementById('arm-side').textContent = armSide;

                    document.getElementById('btn-move').disabled = false;
                    setStatus('좌표 준비됨 (' + label + ')', 'ready');
                });
        }

        function moveRobot() {
            if (!lastCoords) {
                setStatus('먼저 화면을 클릭하세요', 'error');
                return;
            }

            const duration = parseFloat(document.getElementById('duration').value) || 3.0;
            const offsetX = parseFloat(document.getElementById('offset-x').value) || 0;
            const offsetY = parseFloat(document.getElementById('offset-y').value) || 0;
            const offsetZ = parseFloat(document.getElementById('offset-z').value) || 0;

            // 왼손/오른손 구분하여 RPY 값 적용
            const isLeft = lastCoords.torso_y >= 0;
            let roll, pitch, yaw;

            if (isLeft) {
                roll = parseFloat(document.getElementById('left-roll').value) || 0;
                pitch = parseFloat(document.getElementById('left-pitch').value) || 0;
                yaw = parseFloat(document.getElementById('left-yaw').value) || 0;
            } else {
                roll = parseFloat(document.getElementById('right-roll').value) || 0;
                pitch = parseFloat(document.getElementById('right-pitch').value) || 0;
                yaw = parseFloat(document.getElementById('right-yaw').value) || 0;
            }

            document.getElementById('btn-move').disabled = true;
            setStatus('이동 중... (' + duration + '초)', 'moving');

            const url = '/move_to?x=' + lastCoords.torso_x +
                  '&y=' + lastCoords.torso_y +
                  '&z=' + lastCoords.torso_z +
                  '&offset_x=' + offsetX +
                  '&offset_y=' + offsetY +
                  '&offset_z=' + offsetZ +
                  '&roll=' + roll +
                  '&pitch=' + pitch +
                  '&yaw=' + yaw +
                  '&duration=' + duration;

            fetch(url)
                .then(r => r.json())
                .then(data => {
                    document.getElementById('btn-move').disabled = false;
                    if (data.success) {
                        setStatus('이동 완료!', 'ready');
                        addHistory(lastCoords);
                    } else {
                        setStatus('이동 실패: ' + data.error, 'error');
                    }
                })
                .catch(err => {
                    document.getElementById('btn-move').disabled = false;
                    setStatus('오류: ' + err, 'error');
                });
        }

        function goHome() {
            setStatus('홈 위치로 이동 중...', 'moving');
            fetch('/go_home')
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        setStatus('홈 위치 완료', 'ready');
                    } else {
                        setStatus('홈 이동 실패', 'error');
                    }
                });
        }

        function setStatus(msg, type) {
            const el = document.getElementById('status');
            el.textContent = '상태: ' + msg;
            el.className = 'status-' + type;
        }

        function addHistory(coords) {
            const list = document.getElementById('history-list');
            const item = document.createElement('div');
            item.className = 'history-item';
            let label = coords.marker_id !== null ? 'ID:'+coords.marker_id : 'Click';
            item.textContent = label + ' [' + coords.torso_x.toFixed(3) + ', ' +
                               coords.torso_y.toFixed(3) + ', ' + coords.torso_z.toFixed(3) + ']';
            list.insertBefore(item, list.firstChild);
            while (list.children.length > 5) list.removeChild(list.lastChild);
        }
    </script>
</body>
</html>
"""


@app.on_event("startup")
async def startup():
    init_camera()
    init_robot()


@app.on_event("shutdown")
async def shutdown():
    global arm
    if arm:
        print("[로봇] 홈 위치로 이동 후 종료...")
        arm.go_home()


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/get_coords")
async def get_coords(u: int = 0, v: int = 0):
    with lock:
        # 최신 마커 리스트 복사 및 뎁스 이미지 가져오기
        current_markers = list(latest_markers)
        if latest_depth is None:
            return JSONResponse({"error": "깊이 데이터 없음"})
        depth_image = latest_depth.copy()

    # 1. 클릭한 위치에 마커가 있는지 확인
    found_marker = None
    for m in current_markers:
        # pointPolygonTest: 점이 다각형 내부면 >= 0
        if cv2.pointPolygonTest(m['corners'], (u, v), False) >= 0:
            found_marker = m
            break

    target_u, target_v = u, v
    marker_id = None

    # 2. 마커가 발견되면 마커의 중심 픽셀을 사용
    if found_marker:
        target_u = found_marker['cx']
        target_v = found_marker['cy']
        marker_id = found_marker['id']
        print(f"[감지] 마커 클릭 (ID: {marker_id}) -> 중심점 ({target_u}, {target_v}) 사용")
    else:
        print(f"[감지] 일반 클릭 ({u}, {v})")

    # 3. RealSense Depth Map에서 해당 픽셀의 깊이 가져오기
    h, w = depth_image.shape
    target_u = int(np.clip(target_u, 0, w - 1))
    target_v = int(np.clip(target_v, 0, h - 1))

    # 노이즈 제거를 위해 주변 5x5 영역의 중간값 사용
    u_min = max(0, target_u - 2)
    u_max = min(w, target_u + 3)
    v_min = max(0, target_v - 2)
    v_max = min(h, target_v + 3)

    region = depth_image[v_min:v_max, u_min:u_max]
    valid = region[region > 0]

    if len(valid) == 0:
        return JSONResponse({"error": "깊이 측정 실패 (유효값 없음)"})

    depth_mm = float(np.median(valid))

    # 4. Pixel(u,v) + Depth -> Torso(x,y,z) 변환
    torso_x, torso_y, torso_z = pixel_to_torso_coords(target_u, target_v, depth_mm)

    print(f"  Pixel: ({target_u}, {target_v}), Depth: {depth_mm/1000.0:.3f}m")
    print(f"  Torso: [{torso_x:.4f}, {torso_y:.4f}, {torso_z:.4f}]")

    return {
        "marker_id": marker_id, # 마커 없으면 null
        "pixel_u": int(target_u),
        "pixel_v": int(target_v),
        "depth_m": float(depth_mm / 1000.0),
        "torso_x": float(torso_x),
        "torso_y": float(torso_y),
        "torso_z": float(torso_z)
    }


@app.get("/move_to")
async def move_to(x: float, y: float, z: float,
                  offset_x: float = 0.0, offset_y: float = 0.0, offset_z: float = 0.0,
                  roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                  duration: float = 3.0):
    global arm

    # offset 적용
    target_x = x + offset_x
    target_y = y + offset_y
    target_z = z + offset_z

    # RPY 설정
    has_rotation = roll != 0 or pitch != 0 or yaw != 0
    rotation = rpy_to_quaternion(roll, pitch, yaw) if has_rotation else None

    # y 부호로 팔 선택: 양수=왼팔, 음수=오른팔
    if target_y >= 0:
        arm_side = "left"
        left_xyz = [target_x, abs(target_y), target_z]
        right_xyz = HOME_RIGHT
        left_rot = rotation
        right_rot = None
    else:
        arm_side = "right"
        left_xyz = HOME_LEFT
        right_xyz = [target_x, -abs(target_y), target_z]
        left_rot = None
        right_rot = rotation

    print(f"[IK] {arm_side} 팔 이동")
    print(f"  원본: [{x:.4f}, {y:.4f}, {z:.4f}]")
    print(f"  offset: [{offset_x:.4f}, {offset_y:.4f}, {offset_z:.4f}]")
    print(f"  최종: [{target_x:.4f}, {target_y:.4f}, {target_z:.4f}]")
    print(f"  RPY: [{roll:.1f}, {pitch:.1f}, {yaw:.1f}]도")
    print(f"  duration: {duration}초")

    if not ROBOT_AVAILABLE or arm is None:
        print(f"[시뮬레이션] {arm_side} 팔 이동")
        import time
        time.sleep(duration)
        return {"success": True, "message": f"시뮬레이션 완료 ({arm_side})"}

    try:
        arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, 100)

        print(f"[IK] {arm_side} 팔 이동 완료")
        return {"success": True, "message": f"{arm_side} 팔 이동 완료", "arm": arm_side}

    except Exception as e:
        print(f"[IK] 오류: {e}")
        return {"success": False, "error": str(e)}


@app.get("/go_home")
async def go_home():
    global arm

    print("[IK] 홈 위치 요청")

    if not ROBOT_AVAILABLE or arm is None:
        print("[시뮬레이션] 홈 위치")
        return {"success": True, "message": "시뮬레이션"}

    try:
        #arm.go_home()
        arm.move_hands([0.2, 0.2, 0.2], [0.2, -0.2, 0.2], None, None, 3.0, 100)
        print("[IK] 홈 위치 완료")
        return {"success": True}
    except Exception as e:
        print(f"[IK] 홈 오류: {e}")
        return {"success": False, "error": str(e)}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=5000)
