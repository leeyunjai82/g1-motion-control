import cv2
import numpy as np
import uvicorn
import time
from collections import deque
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import openvino as ov
import pyrealsense2 as rs

app = FastAPI()

# 1. OpenVINO 및 모델 초기화
core = ov.Core()
print(f"Available devices: {core.available_devices}")
core.set_property({"CACHE_DIR": "./ov_cache"})

DEVICE = "NPU" if "NPU" in core.available_devices else "CPU"
print(f"Using device: {DEVICE}")

# 모델 경로 + 공개 스펙 (Open Model Zoo 기준 GFLOPs)
MODELS = {
    "det":   {"path": "ov_models/detection/face-detection-retail-0004.xml", "gflops": 1.067},
    "ag":    {"path": "ov_models/age-gender/age-gender-recognition-retail-0013.xml", "gflops": 0.094},
    "emo":   {"path": "ov_models/emotion/emotions-recognition-retail-0003.xml", "gflops": 0.126},
}
EMOTION_LABELS = ['neutral', 'happy', 'sad', 'surprise', 'anger']


def compile_with_fallback(model_path, device):
    model = core.read_model(model_path)
    if device == "NPU":
        try:
            return core.compile_model(model, "NPU"), "NPU"
        except Exception as e:
            print(f"  NPU 컴파일 실패 ({model_path}): {e} → CPU")
            return core.compile_model(model, "CPU"), "CPU"
    return core.compile_model(model, device), device


try:
    print("모델 컴파일 중...")
    compiled_det, dev_det = compile_with_fallback(MODELS["det"]["path"], DEVICE)
    input_det, output_det = compiled_det.input(0), compiled_det.output(0)
    _, _, H_det, W_det = input_det.shape

    compiled_ag, dev_ag = compile_with_fallback(MODELS["ag"]["path"], DEVICE)
    out_age, out_gender = compiled_ag.output("age_conv3"), compiled_ag.output("prob")
    _, _, H_ag, W_ag = compiled_ag.input(0).shape

    compiled_emo, dev_emo = compile_with_fallback(MODELS["emo"]["path"], DEVICE)
    output_emo = compiled_emo.output(0)
    _, _, H_emo, W_emo = compiled_emo.input(0).shape
    print("모든 모델 컴파일 완료")
except Exception as e:
    print(f"모델 로드 오류: {e}"); exit(1)

# --- 2. RealSense 파이프라인 ---
pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.yuyv, 60)
pipeline.start(config)

# --- 3. 성능 측정용 버퍼 (최근 30프레임 평균) ---
fps_buf = deque(maxlen=30)
det_lat_buf = deque(maxlen=30)   # ms
ag_lat_buf = deque(maxlen=30)
emo_lat_buf = deque(maxlen=30)
frame_gflops_buf = deque(maxlen=30)  # 프레임당 총 GFLOPs 누적


def generate_frames():
    prev_t = time.perf_counter()
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            h, w, _ = frame.shape
            frame_gflops = 0.0

            # Face detection (+latency)
            resized_det = cv2.resize(frame, (W_det, H_det))
            input_data_det = np.expand_dims(resized_det.transpose(2, 0, 1), 0).astype(np.float32)
            t0 = time.perf_counter()
            det_results = compiled_det([input_data_det])[output_det]
            det_lat = (time.perf_counter() - t0) * 1000
            det_lat_buf.append(det_lat)
            frame_gflops += MODELS["det"]["gflops"]

            ag_lats, emo_lats = [], []

            for detection in det_results[0][0]:
                confidence = detection[2]
                if confidence > 0.5:
                    xmin, ymin = max(0, int(detection[3] * w)), max(0, int(detection[4] * h))
                    xmax, ymax = min(w, int(detection[5] * w)), min(h, int(detection[6] * h))
                    face_roi = frame[ymin:ymax, xmin:xmax]
                    if face_roi.size == 0: continue

                    # Age-Gender
                    resized_ag = cv2.resize(face_roi, (W_ag, H_ag))
                    t0 = time.perf_counter()
                    ag_results = compiled_ag([np.expand_dims(resized_ag.transpose(2, 0, 1), 0).astype(np.float32)])
                    ag_lats.append((time.perf_counter() - t0) * 1000)
                    frame_gflops += MODELS["ag"]["gflops"]
                    age = int(ag_results[out_age][0][0][0][0] * 100)
                    gender = "Female" if ag_results[out_gender][0][0] > ag_results[out_gender][0][1] else "Male"

                    # Emotion
                    resized_emo = cv2.resize(face_roi, (W_emo, H_emo))
                    t0 = time.perf_counter()
                    emo_results = compiled_emo([np.expand_dims(resized_emo.transpose(2, 0, 1), 0).astype(np.float32)])[output_emo]
                    emo_lats.append((time.perf_counter() - t0) * 1000)
                    frame_gflops += MODELS["emo"]["gflops"]
                    emotion = EMOTION_LABELS[np.argmax(emo_results[0])]

                    label = f"{gender}, {age}s, {emotion}"
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if ag_lats: ag_lat_buf.append(np.mean(ag_lats))
            if emo_lats: emo_lat_buf.append(np.mean(emo_lats))
            frame_gflops_buf.append(frame_gflops)

            # FPS
            now = time.perf_counter()
            fps_buf.append(1.0 / max(now - prev_t, 1e-6))
            prev_t = now

            # 평균값
            avg_fps = np.mean(fps_buf)
            avg_det = np.mean(det_lat_buf) if det_lat_buf else 0
            avg_ag  = np.mean(ag_lat_buf) if ag_lat_buf else 0
            avg_emo = np.mean(emo_lat_buf) if emo_lat_buf else 0
            avg_gflops_per_frame = np.mean(frame_gflops_buf)
            # 달성 연산량 = 프레임당 GFLOPs × FPS → GOPS
            achieved_gops = avg_gflops_per_frame * avg_fps
            achieved_tops = achieved_gops / 1000.0

            # --- HUD ---
            hud = [
                f"Device: det={dev_det} ag={dev_ag} emo={dev_emo}",
                f"FPS: {avg_fps:5.1f}",
                f"Latency (ms)  det:{avg_det:5.1f}  ag:{avg_ag:5.1f}  emo:{avg_emo:5.1f}",
                f"Compute: {achieved_gops:6.1f} GOPS  ({achieved_tops:.3f} TOPS)",
                f"Model FLOPs/frame: {avg_gflops_per_frame:.3f} GFLOPs",
            ]
            y = 25
            for line in hud:
                cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 1, cv2.LINE_AA)
                y += 22

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret: continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        pipeline.stop()


@app.get('/')
def index():
    return {"message": f"Running on {DEVICE}"}


@app.get('/stats')
def stats():
    return {
        "device": {"det": dev_det, "ag": dev_ag, "emo": dev_emo},
        "fps": float(np.mean(fps_buf)) if fps_buf else 0,
        "latency_ms": {
            "detection": float(np.mean(det_lat_buf)) if det_lat_buf else 0,
            "age_gender": float(np.mean(ag_lat_buf)) if ag_lat_buf else 0,
            "emotion": float(np.mean(emo_lat_buf)) if emo_lat_buf else 0,
        },
        "gflops_per_frame": float(np.mean(frame_gflops_buf)) if frame_gflops_buf else 0,
        "achieved_gops": float(np.mean(frame_gflops_buf) * np.mean(fps_buf)) if fps_buf else 0,
    }


@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(), media_type='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
