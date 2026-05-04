#!/usr/bin/env python3
# Version: 1.43
# Changes from 1.42:
#   - HOME z: 0.2 → 0.15
"""
Unitree G1 + 원격/로컬 MJPEG 스트림
ArUco 마커 3D 박스 오버레이 + Start / Release / Home
"""

import os
import sys
import threading
import time
import urllib.request
import numpy as np
import cv2
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# ==========================================
# 로봇 라이브러리 로드
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir  = os.path.dirname(current_dir)
sys.path.append(parent_dir)

try:
    from ctrl.arm_controller_wrapper import ArmControllerWrapper
    ROBOT_AVAILABLE = True
    print("[시스템] 로봇 라이브러리 로드 성공")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"[경고] 시뮬레이션 모드: {e}")

# ==========================================
# 설정
# ==========================================
RS_STREAM_URL = os.environ.get("RS_STREAM_URL", "http://localhost:50002/video_feed")

# 카메라 캘리브레이션 (camera_calib.py 실행해서 출력된 값을 여기에 붙여넣기)
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FX     = 615.000000
CAM_FY     = 615.000000
CAM_PPX    = 320.000000
CAM_PPY    = 240.000000
CAM_DIST   = [0.0, 0.0, 0.0, 0.0, 0.0]

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
MARKER_SIZE     = 0.045

CAMERA_X          = 0.0576235
CAMERA_Y          = 0.01753
CAMERA_Z          = 0.42987
CAMERA_PITCH_URDF = 0.8307767239493009  # 47.6도

HOME_LEFT  = [0.2,  0.2, 0.15]
HOME_RIGHT = [0.2, -0.2, 0.15]

HALF_W     = 0.27 / 2
HALF_D     = 0.09 / 2
HEIGHT_BOX = 0.09
GRIP_EXTRA = -0.015
APPROACH_EXTRA = 0.10
GRAB_Z_OFFSET = 0.05
GRAB_X_OFFSET = -0.15

HANDOVER_X = 0.40

MARKER_OBJ_PTS = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
], dtype=np.float32)

BOX_CORNERS_3D = np.array([
    [-HALF_W, -HALF_D,       0],
    [ HALF_W, -HALF_D,       0],
    [ HALF_W,  HALF_D,       0],
    [-HALF_W,  HALF_D,       0],
    [-HALF_W, -HALF_D, -HEIGHT_BOX],
    [ HALF_W, -HALF_D, -HEIGHT_BOX],
    [ HALF_W,  HALF_D, -HEIGHT_BOX],
    [-HALF_W,  HALF_D, -HEIGHT_BOX],
], dtype=np.float32)

BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# ==========================================
# 전역 변수
# ==========================================
camera_matrix = None
dist_coeffs   = None
image_width   = 640
image_height  = 480

latest_image       = None
latest_markers     = []
latest_marker_pose = None
marker_last_seen_time = 0.0

lock        = threading.Lock()
pose_lock   = threading.Lock()
image_lock  = threading.Lock()

stream_started = False
arm            = None
aruco_dict     = None
aruco_params   = None
aruco_detector = None

grab_state = {'active': False, 'lifted_left': None, 'lifted_right': None, 'busy': False}

# 자동 모드 설정
AUTO_DEFAULT = {
    "enabled": False,
    "x_min": 0.30, "x_max": 0.40,
    "y_min": -0.15, "y_max": 0.15,
    "z_min": -0.10, "z_max": 0.20,
    "dwell_sec": 2.0,
}
auto_mode = dict(AUTO_DEFAULT)
auto_state = {"in_zone_since": None}  # 영역 진입 시각 (None이면 영역 밖)

wrist_params = {
    'left':  {'roll': 0.0, 'pitch': -15.0, 'yaw': -10.0},
    'right': {'roll': 0.0, 'pitch': -15.0, 'yaw':  10.0},
}


# ==========================================
# 카메라 매트릭스 구성 (상단 상수로부터)
# ==========================================
def setup_camera_matrix():
    global camera_matrix, dist_coeffs, image_width, image_height
    image_width  = CAM_WIDTH
    image_height = CAM_HEIGHT
    camera_matrix = np.array([
        [CAM_FX, 0,      CAM_PPX],
        [0,      CAM_FY, CAM_PPY],
        [0,      0,      1      ]
    ], dtype=np.float32)
    if len(CAM_DIST) == 5:
        dist_coeffs = np.array(CAM_DIST, dtype=np.float32)
    else:
        dist_coeffs = np.zeros((5,), dtype=np.float32)
    print(f"[CALIB] {image_width}x{image_height}, fx={CAM_FX:.1f}, fy={CAM_FY:.1f}, "
          f"ppx={CAM_PPX:.1f}, ppy={CAM_PPY:.1f}")


# ==========================================
# ArUco 초기화
# ==========================================
def init_aruco():
    global aruco_dict, aruco_params, aruco_detector
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_detector = None


# ==========================================
# MJPEG 스트림 수신 (별도 스레드)
# ==========================================
def stream_reader_loop():
    """RS_STREAM_URL에서 MJPEG 받아 latest_image 갱신. 끊기면 재연결."""
    global latest_image, stream_started

    while True:
        print(f"[STREAM] 연결 시도: {RS_STREAM_URL}")
        try:
            req = urllib.request.urlopen(RS_STREAM_URL, timeout=5)
            stream_started = True
            print("[STREAM] 연결 성공")

            buf = b""
            while True:
                chunk = req.read(4096)
                if not chunk:
                    break
                buf += chunk

                # JPEG SOI/EOI 마커로 프레임 추출
                while True:
                    soi = buf.find(b'\xff\xd8')
                    eoi = buf.find(b'\xff\xd9', soi + 2) if soi >= 0 else -1
                    if soi < 0 or eoi < 0:
                        break
                    jpg = buf[soi:eoi+2]
                    buf = buf[eoi+2:]

                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        with image_lock:
                            latest_image = img

        except Exception as e:
            print(f"[STREAM] 오류: {e}")
            stream_started = False
            time.sleep(2.0)


# ==========================================
# 로봇 초기화
# ==========================================
def init_robot():
    global arm
    if not ROBOT_AVAILABLE:
        return
    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("[로봇] 초기화 완료")
        print("[로봇] waist 0으로 리셋")
        arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=2.0)
        time.sleep(2.0)

        print("[로봇] HOME 자세로 이동")
        try:
            arm.move_hands(HOME_LEFT, HOME_RIGHT, None, None, 2.0, 100)
            time.sleep(0.5)
        except Exception as e:
            print(f"[로봇] HOME 이동 실패: {e}")
    except Exception as e:
        print(f"[로봇] 초기화 실패: {e}")


def auto_monitor_loop():
    """마커 torso 좌표가 영역 안에 dwell_sec 머물면 자동 잡기 트리거."""
    while True:
        time.sleep(0.1)

        if not auto_mode["enabled"]:
            auto_state["in_zone_since"] = None
            continue
        if grab_state.get('busy'):
            auto_state["in_zone_since"] = None
            continue

        with pose_lock:
            pose = latest_marker_pose
        if pose is None or not is_marker_visible(threshold_sec=0.3):
            auto_state["in_zone_since"] = None
            continue

        tvec = pose['tvec']
        mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])

        in_zone = (
            auto_mode["x_min"] <= mx <= auto_mode["x_max"] and
            auto_mode["y_min"] <= my <= auto_mode["y_max"] and
            auto_mode["z_min"] <= mz <= auto_mode["z_max"]
        )

        if not in_zone:
            auto_state["in_zone_since"] = None
            continue

        # 영역 안
        if auto_state["in_zone_since"] is None:
            auto_state["in_zone_since"] = time.time()
            print(f"[AUTO] 영역 진입: torso=[{mx:.3f},{my:.3f},{mz:.3f}]")
            continue

        elapsed = time.time() - auto_state["in_zone_since"]
        if elapsed >= auto_mode["dwell_sec"]:
            print(f"[AUTO] {auto_mode['dwell_sec']}초 머무름 → 자동 잡기 트리거")
            launch_grab(tvec)


@asynccontextmanager
async def lifespan(app: FastAPI):
    setup_camera_matrix()
    init_aruco()

    # 스트림 reader 스레드 시작
    t = threading.Thread(target=stream_reader_loop, daemon=True)
    t.start()

    # 첫 프레임 도착 대기 (최대 10초)
    print("[STREAM] 첫 프레임 대기...")
    for _ in range(100):
        with image_lock:
            if latest_image is not None:
                break
        time.sleep(0.1)
    if latest_image is None:
        print("[STREAM] 프레임 미도착 — 계속 진행")
    else:
        print("[STREAM] 프레임 수신 시작")

    init_robot()

    # 자동 모니터 스레드 시작
    threading.Thread(target=auto_monitor_loop, daemon=True).start()
    print("[AUTO] 자동 모니터 스레드 시작 (기본 OFF)")

    yield
    if arm:
        try:
            arm.go_home()
        except Exception:
            pass


app = FastAPI(title="G1 Box Grab", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================================
# 좌표 변환 (카메라 pitch 47.6° 정적)
# ==========================================
def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cx_r  =  cx
    cy_r  =  cy * cos_p + cz * sin_p
    cz_r  = -cy * sin_p + cz * cos_p
    return float(cz_r + CAMERA_X), float(-cx_r + CAMERA_Y), float(-cy_r + CAMERA_Z)


def camera_dir_to_torso(dx, dy, dz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    dx_r  =  dx
    dy_r  =  dy * cos_p + dz * sin_p
    dz_r  = -dy * sin_p + dz * cos_p
    return float(dz_r), float(-dx_r), float(-dy_r)


def get_marker_x_axis_in_torso(rvec):
    R, _ = cv2.Rodrigues(rvec)
    x_cam = R[:, 0]
    tx, ty, tz = camera_dir_to_torso(x_cam[0], x_cam[1], x_cam[2])
    v = np.array([tx, ty, tz])
    if np.linalg.norm(v) < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    v[2] = 0.0
    norm_xy = np.linalg.norm(v)
    if norm_xy < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    return v / norm_xy


# ==========================================
# 손목 회전 변환
# ==========================================
def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    import pinocchio as pin
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yq= cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, yq, z).normalized()


# ==========================================
# 3D 박스 그리기
# ==========================================
def draw_box_3d(frame, rvec, tvec):
    """박스 와이어프레임. 윗면은 녹색, 나머지(측면+밑면)는 노랑."""
    pts, _ = cv2.projectPoints(BOX_CORNERS_3D, rvec, tvec, camera_matrix, dist_coeffs)
    pts = pts.reshape(-1, 2).astype(int)

    # 윗면 반투명 녹색 채움
    overlay = frame.copy()
    cv2.fillPoly(overlay, [pts[:4].reshape((-1,1,2))], (0, 255, 0))
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)

    # edge 색 분리: 윗면(0-1, 1-2, 2-3, 3-0)은 녹색, 나머지는 노랑
    TOP_EDGES = {(0,1),(1,2),(2,3),(3,0)}
    for i, j in BOX_EDGES:
        if (i,j) in TOP_EDGES or (j,i) in TOP_EDGES:
            color = (0, 255, 0)      # 윗면 = 녹색
            thickness = 3
        else:
            color = (0, 255, 255)    # 측면/밑면 = 노랑
            thickness = 2
        cv2.line(frame, tuple(pts[i]), tuple(pts[j]), color, thickness)

    # 윗면 코너 점 (강조용, 녹색)
    for pt in pts[:4]:
        cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)


# ==========================================
# ArUco 감지 (이미지만 입력 — 카메라 의존 없음)
# ==========================================
def detect_and_draw_aruco(image):
    global latest_markers, latest_marker_pose, marker_last_seen_time

    if camera_matrix is None or image is None:
        return image

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if aruco_detector:
        corners, ids, _ = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    current_markers = []
    best_pose       = None

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c  = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            ok, rvec, tvec = cv2.solvePnP(
                MARKER_OBJ_PTS, c, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            rvec = rvec.flatten()
            tvec = tvec.flatten()

            draw_box_3d(image, rvec, tvec)

            current_markers.append({'id': int(marker_id), 'corners': c, 'cx': cx, 'cy': cy})
            if best_pose is None:
                best_pose = {'id': int(marker_id), 'rvec': rvec, 'tvec': tvec}

    with lock:
        latest_markers = current_markers
    with pose_lock:
        if best_pose is not None:
            latest_marker_pose = best_pose
            marker_last_seen_time = time.time()

    return image


def is_marker_visible(threshold_sec=0.5):
    return (time.time() - marker_last_seen_time) < threshold_sec


# ==========================================
# 비디오 출력 (latest_image 처리해서 클라에 송출)
# ==========================================
def generate_frames():
    while True:
        with image_lock:
            img = None if latest_image is None else latest_image.copy()

        if img is None:
            time.sleep(0.05)
            continue

        img = detect_and_draw_aruco(img)

        _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


# ==========================================
# 로봇 동작
# ==========================================
def robot_move(left_xyz, right_xyz, duration, msg="", left_rot=None, right_rot=None):
    print(f"[IK] {msg}  L:{[f'{v:.3f}' for v in left_xyz]}  R:{[f'{v:.3f}' for v in right_xyz]}")
    if not ROBOT_AVAILABLE or arm is None:
        time.sleep(duration)
        return True
    try:
        arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, 100)
        return True
    except Exception as e:
        print(f"[IK] 오류: {e}")
        return False


def reset_waist():
    print("[WAIST] 0으로 리셋")
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=1.0)
    time.sleep(0.5)


def align_waist_yaw(tvec):
    yaw_deg = float(np.degrees(-np.arctan2(tvec[0], tvec[2])))
    print(f"[WAIST] yaw: {yaw_deg:.1f}도")
    if abs(yaw_deg) < 1.5:
        return
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=yaw_deg, roll=0.0, pitch=0.0, duration=1.0)
    else:
        time.sleep(1.0)
    time.sleep(0.5)


def grab_sequence(tvec_orig):
    print("[GRAB] ① waist 리셋")
    reset_waist()

    align_waist_yaw(tvec_orig)

    with pose_lock:
        pose = latest_marker_pose
    if pose is None or not is_marker_visible(threshold_sec=0.5):
        print("[GRAB] 재감지 실패 (마커 안 보임)")
        return False, None, None

    tvec = pose['tvec']
    rvec = pose['rvec']
    mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])

    box_x_axis = get_marker_x_axis_in_torso(rvec)
    if box_x_axis[1] < 0:
        box_x_axis = -box_x_axis
    print(f"[GRAB] torso: [{mx:.3f}, {my:.3f}, {mz:.3f}], box_x_axis (torso XY): "
          f"[{box_x_axis[0]:+.3f}, {box_x_axis[1]:+.3f}]")

    grab_x_base = mx + GRAB_X_OFFSET
    grab_z = mz - HEIGHT_BOX / 2 + GRAB_Z_OFFSET
    above_z = mz + 0.10
    lift_z = mz + 0.22
    app_off = HALF_W + GRIP_EXTRA + APPROACH_EXTRA
    grp_off = HALF_W + GRIP_EXTRA

    grip_dir = box_x_axis

    def offset_point(base_x, base_y, z, offset):
        lx = base_x + grip_dir[0] * offset
        ly = base_y + grip_dir[1] * offset
        rx = base_x - grip_dir[0] * offset
        ry = base_y - grip_dir[1] * offset
        return [lx, ly, z], [rx, ry, z]

    lp = wrist_params['left']
    rp = wrist_params['right']
    l_rot = rpy_to_quat(lp['roll'], lp['pitch'], lp['yaw'])
    r_rot = rpy_to_quat(rp['roll'], rp['pitch'], rp['yaw'])

    L, R = offset_point(mx, my, above_z, app_off)
    if not robot_move(L, R, 1.5, "④ 위쪽 접근", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    L, R = offset_point(mx, my, grab_z, app_off)
    if not robot_move(L, R, 1.0, "⑤ 측면 하강", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    L, R = offset_point(grab_x_base, my, grab_z, grp_off)
    if not robot_move(L, R, 2.5, "⑥ 잡기", l_rot, r_rot): return False, None, None
    time.sleep(1.0)

    sym_L = [grab_x_base, +grp_off, grab_z]
    sym_R = [grab_x_base, -grp_off, grab_z]
    if not robot_move(sym_L, sym_R, 1.5, "⑥' 좌우 대칭 정렬", l_rot, r_rot): return False, None, None
    time.sleep(0.3)

    ll = [grab_x_base, +grp_off, lift_z]
    rl = [grab_x_base, -grp_off, lift_z]
    if not robot_move(ll, rl, 1.5, "⑦ 들기", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    print("[GRAB] ⑦' 허리 0 복귀")
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=1.5)
        time.sleep(0.5)

    hl = [HANDOVER_X, +grp_off, lift_z]
    hr = [HANDOVER_X, -grp_off, lift_z]
    if not robot_move(hl, hr, 1.5, "⑧ 건네기", l_rot, r_rot): return False, None, None
    time.sleep(0.3)

    print("[HANDOVER] 10초 대기 (마커 가려지면 받음, 안 가려지면 내려놓음)")
    start = time.time()
    received = False
    while time.time() - start < 10.0:
        if not is_marker_visible(threshold_sec=0.5):
            received = True
            print(f"[HANDOVER] 마커 가려짐 → 받음 감지 ({time.time()-start:.1f}초)")
            break
        time.sleep(0.1)

    if received:
        robot_move([HANDOVER_X, +grp_off+0.10, lift_z],
                   [HANDOVER_X, -grp_off-0.10, lift_z],
                   1.0, "⑩ 손 벌림 (수령)", l_rot, r_rot)
    else:
        print("[HANDOVER] 타임아웃 → 박스 내려놓기")
        robot_move([HANDOVER_X, +grp_off, grab_z],
                   [HANDOVER_X, -grp_off, grab_z],
                   1.5, "⑩a 내려놓기", l_rot, r_rot)
        robot_move([HANDOVER_X, +grp_off+0.10, grab_z],
                   [HANDOVER_X, -grp_off-0.10, grab_z],
                   1.0, "⑩b 손 벌림", l_rot, r_rot)
        robot_move([HANDOVER_X, +grp_off+0.10, lift_z],
                   [HANDOVER_X, -grp_off-0.10, lift_z],
                   1.0, "⑩c 위로 후퇴", l_rot, r_rot)

    print("[HANDOVER] 홈 복귀")
    reset_waist()
    robot_move(HOME_LEFT, HOME_RIGHT, 2.0, "⑪ Home")

    return False, None, None


# ==========================================
# HTML
# ==========================================
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
    <title>G1 Box Grab</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: monospace; background: #1a1a1a; color: #fff; padding: 24px; }
        #wrap { max-width: 1000px; margin: 0 auto; }
        h1   { color: #4CAF50; margin-bottom: 4px; font-size: 22px; }
        .sub { color: #666; font-size: 13px; margin-bottom: 24px; }
        #layout { display: flex; gap: 24px; align-items: flex-start; justify-content: center; }
        #stream  { border: 2px solid #4CAF50; display: block; }

        #panel {
            background: #242424; border-radius: 10px; padding: 20px;
            width: 300px; display: flex; flex-direction: column; gap: 16px;
        }

        .card { background: #1e1e1e; border-radius: 8px; padding: 14px; }
        .card-title { color: #4CAF50; font-size: 12px; text-transform: uppercase;
                      letter-spacing: 1px; margin-bottom: 10px; }

        #marker-status { font-size: 14px; color: #555; }
        #marker-status.found { color: #4CAF50; }

        .info-row { display: flex; justify-content: space-between; font-size: 13px; margin: 4px 0; }
        .info-key { color: #555; }
        .info-val { color: #ccc; }

        .btn-group { display: flex; gap: 8px; }
        .btn {
            flex: 1; padding: 13px 0; border: none; border-radius: 6px;
            font-size: 14px; font-weight: bold; cursor: pointer; transition: opacity .15s;
        }
        .btn:disabled { opacity: 0.35; cursor: not-allowed; }
        .btn-start   { background: #4CAF50; color: #000; }
        .btn-release { background: #2196F3; color: #fff; }
        .btn-home    { background: #FF9800; color: #000; }

        #status-bar {
            border-radius: 6px; padding: 10px 14px; font-size: 13px;
            background: #2a2a2a; color: #666;
        }
        .s-running { background: #1a2a1a !important; color: #4CAF50 !important; }
        .s-holding { background: #1a1a2a !important; color: #64B5F6 !important; }
        .s-error   { background: #2a1a1a !important; color: #ef5350 !important; }
        .rpy-row   { display:flex; align-items:center; margin:3px 0; }
        .rpy-label { color:#555; font-size:12px; width:16px; }
        .rpy-input { background:#333; border:1px solid #444; color:#fff; padding:4px 6px;
                     border-radius:4px; width:70px; font-size:13px; }
        .btn-apply { background:#555; color:#fff; border:none; border-radius:4px; cursor:pointer; font-size:13px; }
        .btn-apply:hover { background:#666; }
    </style>
</head>
<body>
    <div id="wrap">
    <h1>G1 Box Grab</h1>
    <p class="sub">원격 MJPEG → 마커 감지 → 가로면 잡기 → 정면 대칭 → 건네기</p>

    <div id="layout">
        <img id="stream" src="/video_feed" width="640" height="480">

        <div id="panel">
            <div class="card">
                <div class="card-title">마커 감지</div>
                <div id="marker-status">대기 중...</div>
            </div>

            <div class="card">
                <div class="card-title">박스 위치 (torso)</div>
                <div class="info-row"><span class="info-key">X 전방</span><span class="info-val" id="tx">-</span></div>
                <div class="info-row"><span class="info-key">Y 좌우</span><span class="info-val" id="ty">-</span></div>
                <div class="info-row"><span class="info-key">Z 높이</span><span class="info-val" id="tz">-</span></div>
                <div class="info-row"><span class="info-key">Yaw 목표</span><span class="info-val" id="yaw">-</span></div>
            </div>

            <div class="card">
                <div class="card-title">손목 RPY (도)</div>
                <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px;">
                    <div>
                        <div style="color:#4CAF50; font-size:11px; margin-bottom:4px;">왼손 L</div>
                        <div class="rpy-row"><span class="rpy-label">R</span><input class="rpy-input" id="l-roll"  type="number" value="0" step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">P</span><input class="rpy-input" id="l-pitch" type="number" value="-15"   step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">Y</span><input class="rpy-input" id="l-yaw"   type="number" value="-10" step="5"></div>
                    </div>
                    <div>
                        <div style="color:#FF9800; font-size:11px; margin-bottom:4px;">오른손 R</div>
                        <div class="rpy-row"><span class="rpy-label">R</span><input class="rpy-input" id="r-roll"  type="number" value="0" step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">P</span><input class="rpy-input" id="r-pitch" type="number" value="-15"  step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">Y</span><input class="rpy-input" id="r-yaw"   type="number" value="10" step="5"></div>
                    </div>
                </div>
                <button class="btn btn-apply" onclick="applyWrist()" style="margin-top:10px; width:100%; padding:8px;">적용</button>
                <div id="wrist-msg" style="font-size:11px; color:#666; margin-top:6px;"></div>
            </div>

            <div class="card">
                <div class="card-title">자동 모드</div>
                <label style="display:flex; align-items:center; gap:8px; margin-bottom:10px;">
                    <input type="checkbox" id="auto-enabled" onchange="toggleAuto()" style="width:18px; height:18px;">
                    <span id="auto-label" style="color:#888;">OFF</span>
                </label>
                <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; font-size:11px; color:#666;">
                    <div>X min<input class="rpy-input" id="ax-min" type="number" value="0.30" step="0.05" style="width:100%"></div>
                    <div>X max<input class="rpy-input" id="ax-max" type="number" value="0.40" step="0.05" style="width:100%"></div>
                    <div>Y min<input class="rpy-input" id="ay-min" type="number" value="-0.15" step="0.05" style="width:100%"></div>
                    <div>Y max<input class="rpy-input" id="ay-max" type="number" value="0.15" step="0.05" style="width:100%"></div>
                    <div>Z min<input class="rpy-input" id="az-min" type="number" value="-0.10" step="0.05" style="width:100%"></div>
                    <div>Z max<input class="rpy-input" id="az-max" type="number" value="0.20" step="0.05" style="width:100%"></div>
                </div>
                <button class="btn btn-apply" onclick="applyAutoZone()" style="margin-top:8px; width:100%; padding:6px; font-size:11px;">영역 적용</button>
                <div id="auto-progress" style="margin-top:8px; height:6px; background:#333; border-radius:3px; overflow:hidden;">
                    <div id="auto-bar" style="height:100%; width:0%; background:#4CAF50; transition: width .15s;"></div>
                </div>
                <div id="auto-msg" style="font-size:11px; color:#666; margin-top:6px;">대기</div>
            </div>

            <div id="status-bar">대기 중</div>

            <div class="btn-group">
                <button class="btn btn-start"   id="btn-start"   onclick="startGrab()">▶ Start</button>
                <button class="btn btn-release" id="btn-release" onclick="doRelease()" disabled>↔ Release</button>
                <button class="btn btn-home"    id="btn-home"    onclick="goHome()">⌂ Home</button>
            </div>
        </div>
    </div>

    <script>
        function poll() {
            fetch('/status').then(r => r.json()).then(d => {
                const ms = document.getElementById('marker-status');
                ms.className   = d.marker_found ? 'found' : '';
                ms.textContent = d.marker_found
                    ? `ID: ${d.marker_id}  감지됨 ✓`
                    : '대기 중...';

                if (d.torso) {
                    document.getElementById('tx').textContent  = d.torso.x.toFixed(3) + ' m';
                    document.getElementById('ty').textContent  = d.torso.y.toFixed(3) + ' m';
                    document.getElementById('tz').textContent  = d.torso.z.toFixed(3) + ' m';
                    document.getElementById('yaw').textContent = d.yaw_deg.toFixed(1) + '°';
                }

                document.getElementById('btn-start').disabled   = d.busy;
                document.getElementById('btn-home').disabled    = d.busy;
                document.getElementById('btn-release').disabled = d.busy || !d.grab_active;

                if (d.busy) setSt('동작 중...', 'running');
                else if (d.grab_active) setSt('박스 들고 있음', 'holding');
                else setSt('대기 중', '');

                // 자동 모드 표시
                const cb = document.getElementById('auto-enabled');
                if (cb.checked !== d.auto_enabled) cb.checked = d.auto_enabled;
                document.getElementById('auto-label').textContent = d.auto_enabled ? 'ON' : 'OFF';
                document.getElementById('auto-label').style.color = d.auto_enabled ? '#4CAF50' : '#888';

                const pct = d.auto_dwell > 0
                    ? Math.min(100, (d.auto_elapsed / d.auto_dwell) * 100)
                    : 0;
                document.getElementById('auto-bar').style.width = pct + '%';

                if (!d.auto_enabled) {
                    document.getElementById('auto-msg').textContent = '대기 (OFF)';
                } else if (d.busy) {
                    document.getElementById('auto-msg').textContent = '잡기 동작 중';
                } else if (d.auto_in_zone) {
                    document.getElementById('auto-msg').textContent =
                        `영역 안 ${d.auto_elapsed.toFixed(1)}/${d.auto_dwell.toFixed(1)}초`;
                } else {
                    document.getElementById('auto-msg').textContent = '마커 영역 밖';
                }
            }).catch(() => {});
        }
        setInterval(poll, 500);

        function toggleAuto() {
            const enabled = document.getElementById('auto-enabled').checked;
            fetch('/set_auto_mode?enabled=' + enabled).then(r => r.json());
        }
        function applyAutoZone() {
            const q = [
                'x_min=' + document.getElementById('ax-min').value,
                'x_max=' + document.getElementById('ax-max').value,
                'y_min=' + document.getElementById('ay-min').value,
                'y_max=' + document.getElementById('ay-max').value,
                'z_min=' + document.getElementById('az-min').value,
                'z_max=' + document.getElementById('az-max').value,
            ].join('&');
            fetch('/set_auto_mode?' + q).then(r => r.json()).then(d => {
                document.getElementById('auto-msg').textContent = '영역 적용됨';
            });
        }

        function startGrab() {
            setBtns(true);
            fetch('/start_grab').then(r => r.json()).then(d => {
                if (!d.success) setSt('실패: ' + d.error, 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function doRelease() {
            setBtns(true);
            fetch('/release').then(r => r.json()).then(d => {
                if (!d.success) setSt('실패: ' + d.error, 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function goHome() {
            setBtns(true);
            fetch('/go_home').then(r => r.json()).then(d => {
                if (!d.success) setSt('홈 실패', 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function applyWrist() {
            const params = {
                l_roll:  parseFloat(document.getElementById('l-roll').value)  || 0,
                l_pitch: parseFloat(document.getElementById('l-pitch').value) || 0,
                l_yaw:   parseFloat(document.getElementById('l-yaw').value)   || 0,
                r_roll:  parseFloat(document.getElementById('r-roll').value)  || 0,
                r_pitch: parseFloat(document.getElementById('r-pitch').value) || 0,
                r_yaw:   parseFloat(document.getElementById('r-yaw').value)   || 0,
            };
            const q = Object.entries(params).map(([k,v]) => `${k}=${v}`).join('&');
            fetch('/set_wrist?' + q).then(r => r.json()).then(d => {
                document.getElementById('wrist-msg').textContent = d.success ? '적용됨 ✓' : '실패';
                document.getElementById('wrist-msg').style.color = d.success ? '#4CAF50' : '#ef5350';
            });
        }
        function setSt(msg, cls) {
            const el = document.getElementById('status-bar');
            el.textContent = msg;
            el.className   = cls ? `s-${cls}` : '';
        }
        function setBtns(disabled) {
            document.getElementById('btn-start').disabled = disabled;
            document.getElementById('btn-home').disabled  = disabled;
        }
    </script>
</body>
</html>"""


# ==========================================
# 엔드포인트
# ==========================================
@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_frames(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/status")
async def status():
    with pose_lock:
        pose = latest_marker_pose

    visible = pose is not None and is_marker_visible(threshold_sec=0.5)

    torso = None
    yaw_deg = 0.0
    if visible:
        tvec = pose['tvec']
        mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])
        torso   = {"x": round(mx,3), "y": round(my,3), "z": round(mz,3)}
        yaw_deg = float(np.degrees(-np.arctan2(tvec[0], tvec[2])))

    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time() - in_zone_since) if in_zone_since else 0.0

    return {
        "grab_active":    grab_state['active'],
        "busy":           grab_state.get('busy', False),
        "marker_found":   visible,
        "marker_id":      pose['id'] if visible else None,
        "torso":          torso,
        "yaw_deg":        round(yaw_deg, 1),
        "stream_started": stream_started,
        "auto_enabled":   auto_mode["enabled"],
        "auto_in_zone":   in_zone_since is not None,
        "auto_elapsed":   round(elapsed, 2),
        "auto_dwell":     auto_mode["dwell_sec"],
    }


def launch_grab(tvec):
    """grab_sequence를 백그라운드 스레드로 시작. busy 중이면 무시."""
    if grab_state.get('busy'):
        return False

    def run():
        global grab_state
        grab_state['busy'] = True
        # 자동 모드 타이머 리셋 (잡기 중 트리거 안 되게)
        auto_state['in_zone_since'] = None
        try:
            ok, ll, rl = grab_sequence(tvec)
            grab_state['active']      = ok
            grab_state['lifted_left'] = ll
            grab_state['lifted_right']= rl
            if ok:
                print("[GRAB] 완료 (들고 있음)")
            elif ll is None and rl is None:
                print("[GRAB] 자동 완료")
            else:
                print("[GRAB] 실패")
        finally:
            grab_state['busy'] = False
            # 잡기 끝나면 영역 타이머 리셋
            auto_state['in_zone_since'] = None

    threading.Thread(target=run, daemon=True).start()
    return True


@app.get("/start_grab")
async def start_grab():
    # 3초 동안 마커 찾기
    print("[START] 마커 탐색 (최대 3초)...")
    deadline = time.time() + 3.0
    pose = None
    while time.time() < deadline:
        with pose_lock:
            p = latest_marker_pose
        if p is not None and is_marker_visible(threshold_sec=0.3):
            pose = p
            break
        time.sleep(0.1)

    if pose is None:
        print("[START] 마커 미발견 (타임아웃)")
        return JSONResponse({"success": False, "error": "마커가 감지되지 않았습니다 (3초 대기)"})

    print(f"[START] 마커 발견: ID {pose['id']}")
    if not launch_grab(pose['tvec']):
        return JSONResponse({"success": False, "error": "이미 동작 중입니다"})
    return JSONResponse({"success": True, "marker_id": pose['id']})


@app.get("/release")
async def release():
    global grab_state

    if not grab_state['active']:
        return JSONResponse({"success": False, "error": "박스를 잡고 있지 않습니다"})

    ll = grab_state['lifted_left']
    rl = grab_state['lifted_right']

    def run():
        global grab_state
        grab_state['busy'] = True
        try:
            robot_move([ll[0], ll[1]+0.10, ll[2]],
                       [rl[0], rl[1]-0.10, rl[2]],
                       1.5, "Release")
            grab_state['active'] = False
        finally:
            grab_state['busy'] = False

    threading.Thread(target=run, daemon=True).start()
    return JSONResponse({"success": True})


@app.get("/go_home")
async def go_home():
    global grab_state

    def run():
        global grab_state
        grab_state['busy'] = True
        try:
            reset_waist()
            robot_move(HOME_LEFT, HOME_RIGHT, 2.0, "Home")
            grab_state['active'] = False
        finally:
            grab_state['busy'] = False

    threading.Thread(target=run, daemon=True).start()
    return JSONResponse({"success": True})


@app.get("/set_wrist")
async def set_wrist(
    l_roll: float=0, l_pitch: float=0, l_yaw: float=0,
    r_roll: float=0, r_pitch: float=0, r_yaw: float=0
):
    global wrist_params
    wrist_params = {
        'left':  {'roll': l_roll, 'pitch': l_pitch, 'yaw': l_yaw},
        'right': {'roll': r_roll, 'pitch': r_pitch, 'yaw': r_yaw},
    }
    print(f"[WRIST] L: R={l_roll} P={l_pitch} Y={l_yaw}  R: R={r_roll} P={r_pitch} Y={r_yaw}")
    return {"success": True, "wrist_params": wrist_params}


@app.get("/auto_mode")
async def get_auto_mode():
    """현재 자동 모드 설정/상태 반환"""
    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time() - in_zone_since) if in_zone_since else 0.0
    return {
        "config": auto_mode,
        "in_zone": in_zone_since is not None,
        "elapsed_sec": round(elapsed, 2),
    }


@app.get("/set_auto_mode")
async def set_auto_mode(
    enabled: bool = None,
    x_min: float = None, x_max: float = None,
    y_min: float = None, y_max: float = None,
    z_min: float = None, z_max: float = None,
    dwell_sec: float = None,
):
    """자동 모드 설정. None은 변경 안 함"""
    global auto_mode
    for k, v in [("enabled", enabled),
                 ("x_min", x_min), ("x_max", x_max),
                 ("y_min", y_min), ("y_max", y_max),
                 ("z_min", z_min), ("z_max", z_max),
                 ("dwell_sec", dwell_sec)]:
        if v is not None:
            auto_mode[k] = v
    # 상태 변경 시 타이머 리셋
    auto_state["in_zone_since"] = None
    print(f"[AUTO] 설정 변경: {auto_mode}")
    return {"success": True, "config": auto_mode}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50000)
