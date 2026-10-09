#!/usr/bin/env python3
"""
head_track.py — H2 머리 카메라 인식 서버 (포트 50013)
  왼눈  : 얼굴 (없으면 사람) — OpenVINO Open Model Zoo face-detection-retail-0004 / person-detection-0201 (NPU)
  오른눈: 사물 (COCO 80종, 기본 person 제외) — YOLO11s OpenVINO (common/models/yolo11s_openvino_model, NPU)
  스트립: 두 눈에서 인식된 것만 잘라 한 줄로 (제어 화면 robot_web.html 의 Head Vision 카드)
  (가슴 D435i 는 박스 인식 — detect_box, 이 서버와 무관)

  ROBOT=h2 python common/head_track.py            # 보기만 → http://<pc-ip>:50013/   (start_robot.sh 가 H2 에서 띄움)
  ROBOT=h2 python common/head_track.py --drive    # 머리 명령 준비 — 웹 '추종 켜기' 를 눌러야 움직임 (arm_server :50022 필요)
  ROBOT=h2 python common/head_track.py --no-objects   # 오른눈 사물 인식 끔

  ⚠️ 실기 2026-10-09: H2 703 에서 rt/arm_sdk 머리 29/30 명령이 반영 안 됨 (토크 0, robots/h2/FACTS.md) — 펌웨어가 머리를
     arm_sdk 에 넘겨줄 때까지 --drive 는 효과 없음. 인식·스트리밍은 그대로 동작
  준비: 앱에서 video_hub 끔 + 'Stereo patch PC1' 켬, RGB 수신 IP = 이 PC
        (python utils/check_head_cam.py --set-ip → 앱에서 서비스 재시작). gstreamer 설치는 common/ctrl/head_cam.py 참고.
        check_head_cam.py 와 동시에 켜지 말 것 (같은 UDP 포트)
  설정: robot.yaml head_track (fps 5 — 눈별 인식 횟수, device, objects, strip, 추종 값들)

  머리 추종 ('느낌만' — 사람 쪽으로 조금만 돌림. 카메라가 머리와 같이 도니 내부 파라미터·카메라 위치 보정 필요 없음)
    e = (대상 점 − 화면 중심) / (화면 반폭, 반높이)   ∈ [−1, 1]
    대상 방향 ≈ 프레임 받을 때 실측 머리각 + (pitch: +e_y · vfov/2, yaw: −e_x · hfov/2)   (pitch + = 숙임, yaw + = 왼쪽)
      → smooth 로 고르게 (지수 평균)
    원하는 각 = home + follow_ratio · (대상 방향 − home)  → track_range_deg 안으로 자름
    지금 명령과 deadband_deg 안이면 그 축은 그대로. 변화 속도 ≤ max_speed_deg
    대상 점: 얼굴 = 박스 중심, 사람 = 박스 위에서 person_aim 비율 아래 (≈ 머리)
    대상 고르기: 가장 큰 것. 단 지금 대상(가장 가까운 박스)보다 switch_ratio 배 이상 클 때만 바꿈
    새 대상은 confirm_frames 번 연속 보여야 따라감 (한 번 오검출에 머리가 튀지 않게)
    대상이 lost_s 동안 없으면 home_deg 로 home_speed_deg 속도로

  API
    GET  /             화면 (왼눈·오른눈 영상 + 스트립 + 상태 + 추종 켜기/끄기)
    GET  /strip        MJPEG — 인식된 것만 잘라 한 줄 (왼눈 초록·주황, 오른눈 파랑), /strip.jpg = 한 장
    GET  /video_feed   MJPEG 왼눈 (얼굴·사람 박스, 대상, 머리각)
    GET  /video_right  MJPEG 오른눈 (사물 박스)
    GET  /detections   JSON 지금 인식 목록
    GET  /status       JSON
    POST /track        {"enable": true|false}  끄면 머리는 그 자리에 둠
    POST /look         {"pitch": deg, "yaw": deg}  추종 끄고 이 각도로
    POST /home         추종 끄고 home 으로
"""
import argparse
import json
import os
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_env   # noqa: E402  ROBOT 필수
from ctrl.head_cam import PC1, RGB_PORTS, RgbRx, my_ip, web_urls   # noqa: E402
from ctrl.ov_detect import OvSSD   # noqa: E402

import uvicorn   # noqa: E402
from fastapi import FastAPI, HTTPException   # noqa: E402
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse   # noqa: E402
from pydantic import BaseModel   # noqa: E402

C = dict(robot_env.CFG.get("head_track") or {})
ARM_SERVER = "http://localhost:50022"
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FRESH_S = 1.0                      # 스트립에 쓸 인식 결과의 최대 나이 [s]


def cfg(key, default):
    v = C.get(key, default)
    return default if v is None else v


def arm_head(pitch=None, yaw=None, timeout=0.3):
    """arm_server /head — pitch·yaw [deg] 주면 POST, 아니면 GET. 반환 (dict 또는 None, 오류 문자열)."""
    try:
        if pitch is None:
            req = urllib.request.Request(ARM_SERVER + "/head")
        else:
            req = urllib.request.Request(ARM_SERVER + "/head", data=json.dumps({"pitch": float(pitch), "yaw": float(yaw)}).encode(),
                                         headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read()), ""
    except Exception as e:      # noqa: BLE001 — 연결 실패·409 등은 상태로만 표시
        return None, f"arm_server /head: {e}"


def wait_rgb(port, who):
    """머리 카메라 RGB 가 올 때까지 기다렸다가 수신기 반환 (서비스가 나중에 켜져도 됨). who.err 에 상태."""
    while True:
        rx = RgbRx(port)
        if rx.probe(6.0):
            rx.start()
            who.err = ""
            print(f"[head_track] 머리 카메라 RGB {rx.size[0]}×{rx.size[1]} (UDP {port})")
            return rx
        who.err = (f"머리 카메라 RGB 없음 (UDP {port}) — 앱 video_hub 끔 · Stereo patch PC1 켬 · "
                   f"수신 IP = {my_ip(PC1)} 인지 확인")


def jpeg(img, q=80):
    ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return j.tobytes() if ok else None


class EyeLoop(threading.Thread):
    """한 눈의 수신 → (fps 만큼) 인식 → 그림 반복. 하위 클래스가 detect / draw 를 채움."""

    def __init__(self, port, name):
        super().__init__(daemon=True)
        self.port, self.name = port, name
        self.lock = threading.Lock()
        self.jpeg = None
        self.last = None                   # (BGR, 인식 목록, 받은 시각)
        self.fps = 0.0
        self.det_ms = 0.0
        self.err = "머리 카메라 RGB 기다리는 중"

    def process(self, img, now, dt):
        raise NotImplementedError

    def run(self):
        rx = wait_rgb(self.port, self)
        period = 1.0 / max(0.5, float(cfg("fps", 5.0)))
        last_t, last_proc, t_fps, n_fps = 0.0, 0.0, time.time(), 0
        while True:
            f = rx.latest()
            now = time.time()
            if f is None or f[1] == last_t or now - last_proc < period:
                if last_t and now - last_t > 1.0:
                    self.err, self.fps = f"머리 카메라 RGB 끊김 (UDP {self.port})", 0.0
                time.sleep(0.005)
                continue
            if self.err.startswith("머리 카메라 RGB 끊김"):
                self.err = ""
            img, t = f
            dt = float(np.clip(t - last_proc, 0.05, 0.5)) if last_proc else period
            last_t, last_proc = t, now
            t0 = time.time()
            dets, view = self.process(img, now, dt)
            self.det_ms = (time.time() - t0) * 1000
            j = jpeg(view)
            with self.lock:
                self.last = (img, dets, now)
                if j is not None:
                    self.jpeg = j
            n_fps += 1
            if now - t_fps >= 2.0:
                self.fps, t_fps, n_fps = n_fps / (now - t_fps), now, 0


class Tracker(EyeLoop):
    """왼눈: 얼굴 (없으면 사람) + 머리 추종."""

    def __init__(self, port, drive, mode, device, track_on=False):
        super().__init__(port, "left")
        self.drive, self.mode = drive, mode
        self.face = OvSSD("face", device, cfg("face_conf", 0.6)) if mode in ("face", "auto") else None
        self.person = OvSSD("person", device, cfg("person_conf", 0.5)) if mode in ("person", "auto") else None
        self.enabled = drive and track_on   # --drive 만이면 꺼진 채 시작 → 웹 '추종 켜기'
        self.cmd = None                    # 마지막으로 보낸 머리 목표 [pitch, yaw] deg
        self.meas = None                   # arm_server 실측 [pitch, yaw] deg
        self.look = None                   # /look·/home 요청 (추종 끈 상태에서 보낼 각)
        self.tgt = None                    # 지금 대상 {"box", "kind", "pt", "area"}
        self.dir = None                    # 대상 방향 [pitch, yaw] deg (지수 평균)
        self.last_seen = 0.0
        self.last_face = 0.0
        self.streak = 0                    # 후보가 연속으로 보인 횟수
        self.active = False                # 확인된 대상을 따라가는 중 (lost_s 동안 유지)
        self.cands = []
        self.arm_err = ""

    # ---- 대상 ----
    def _pick(self, cands):
        if not cands:
            return None
        big = max(cands, key=lambda c: c["area"])
        if self.tgt is None:
            return big
        px, py = self.tgt["pt"]
        near = min(cands, key=lambda c: (c["pt"][0] - px) ** 2 + (c["pt"][1] - py) ** 2)
        if big is not near and big["area"] >= float(cfg("switch_ratio", 1.3)) * near["area"]:
            return big
        return near

    @staticmethod
    def _cands(faces, persons):
        out = [{"box": b, "kind": "face", "pt": ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2),
                "area": (b[2] - b[0]) * (b[3] - b[1])} for b in faces]
        if out:
            return out
        a = float(cfg("person_aim", 0.12))
        return [{"box": b, "kind": "person", "pt": ((b[0] + b[2]) / 2, b[1] + a * (b[3] - b[1])),
                 "area": (b[2] - b[0]) * (b[3] - b[1])} for b in persons]

    # ---- 머리 명령 ----
    def _command(self, img_shape, dt, now):
        hs, self.arm_err = arm_head()
        if hs is None:
            return
        self.meas = np.array(hs["meas"], float)
        if self.cmd is None:
            self.cmd = np.array(hs["target"], float)
        h = cfg("home_deg", {"pitch": 10.0, "yaw": 0.0})
        home = np.array([h["pitch"], h["yaw"]], float)
        if self.look is not None:                                  # /look, /home
            self.dir = None
            want, speed = np.array(self.look[:2], float), float(self.look[2])
        elif not self.enabled:
            self.dir = None
            return
        elif self.tgt is not None:
            H, W = img_shape[:2]
            ex = (self.tgt["pt"][0] - W / 2) / (W / 2)
            ey = (self.tgt["pt"][1] - H / 2) / (H / 2)
            d = np.array([self.meas[0] + ey * float(cfg("vfov_deg", 75.0)) / 2,
                          self.meas[1] - ex * float(cfg("hfov_deg", 90.0)) / 2])
            a = float(cfg("smooth", 0.3))
            self.dir = d if self.dir is None else (1 - a) * self.dir + a * d
            want = home + float(cfg("follow_ratio", 0.4)) * (self.dir - home)
            r = cfg("track_range_deg", {"pitch": [-20.0, 20.0], "yaw": [-30.0, 30.0]})
            want = np.clip(want, [r["pitch"][0], r["yaw"][0]], [r["pitch"][1], r["yaw"][1]])
            db = float(cfg("deadband_deg", 3.0))
            want = np.where(np.abs(want - self.cmd) < db, self.cmd, want)
            speed = float(cfg("max_speed_deg", 25.0))
        elif now - self.last_seen > float(cfg("lost_s", 2.5)):
            self.dir = None
            want, speed = home, float(cfg("home_speed_deg", 15.0))
        else:
            return                                                 # 잠깐 놓침 — 그 자리 유지
        step = speed * dt
        new = self.cmd + np.clip(want - self.cmd, -step, step)
        if np.allclose(new, self.cmd, atol=0.05):
            if self.look is not None and np.allclose(new, self.look[:2], atol=0.1):
                self.look = None if self.enabled else self.look    # 도착
            return
        r, self.arm_err = arm_head(new[0], new[1])
        if r is not None:
            self.cmd = np.array(r["target"], float)                # robot.yaml head_range_deg 로 잘린 값

    def process(self, img, now, dt):
        faces = self.face(img) if self.face is not None else []
        if faces:
            self.last_face = now
        persons = []
        if self.person is not None and (self.mode == "person" or
                                        (not faces and now - self.last_face > float(cfg("person_after_s", 1.0)))):
            persons = self.person(img)
        cands = self._cands(faces, persons)
        self.streak = self.streak + 1 if cands else 0
        with self.lock:
            self.cands = cands
            pick = self._pick(cands)
            if pick is not None and (self.active or self.streak >= int(cfg("confirm_frames", 3))):
                self.tgt, self.last_seen, self.active = pick, now, True
            else:
                self.tgt = None
                if now - self.last_seen > float(cfg("lost_s", 2.5)):
                    self.active = False
        if self.drive:
            self._command(img.shape, dt, now)
        dets = [dict(label="face", conf=b[4], box=b[:4]) for b in faces] + \
               [dict(label="person", conf=b[4], box=b[:4]) for b in persons]
        return dets, self._draw(img, faces, persons)

    def _draw(self, img, faces, persons):
        v = img.copy()
        H, W = v.shape[:2]
        for b in persons:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 160, 255), 1)
        for b in faces:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 220, 120), 1)
        for c in self.cands:                                       # 확인 전 후보 (노랑)
            if self.tgt is None:
                cv2.circle(v, (int(c["pt"][0]), int(c["pt"][1])), 4, (0, 255, 255), -1)
        if self.tgt is not None:
            b = self.tgt["box"]
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
            cv2.circle(v, (int(self.tgt["pt"][0]), int(self.tgt["pt"][1])), 4, (0, 255, 0), -1)
        cv2.drawMarker(v, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        head = "-" if self.meas is None else f"pitch {self.meas[0]:+.1f}  yaw {self.meas[1]:+.1f}"
        state = "TRACK" if (self.enabled and self.drive) else ("VIEW" if not self.drive else "OFF")
        cv2.putText(v, f"L {state}  {self.fps:.1f} fps  det {self.det_ms:.0f} ms  head {head}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return v

    def status(self):
        with self.lock:
            t = self.tgt
        return {"drive": self.drive, "enabled": self.enabled, "mode": self.mode,
                "fps": round(self.fps, 1), "det_ms": round(self.det_ms, 1),
                "target": None if t is None else {"kind": t["kind"], "pt": [round(v) for v in t["pt"]],
                                                  "box": [round(v) for v in t["box"][:4]]},
                "head_meas": None if self.meas is None else [round(float(v), 1) for v in self.meas],
                "head_cmd": None if self.cmd is None else [round(float(v), 1) for v in self.cmd],
                "models": [m.name + "@" + m.device for m in (self.face, self.person) if m is not None],
                "error": self.err or self.arm_err}


class Objects(EyeLoop):
    """오른눈: COCO 사물 (ultralytics YOLO, OpenVINO). device 는 ultralytics 형식 (intel:npu / intel:gpu / intel:cpu)."""

    def __init__(self, port, model, device, conf, imgsz, exclude):
        super().__init__(port, "right")
        from ultralytics import YOLO
        self.model = YOLO(model, task="detect")
        self.model_name = os.path.basename(model.rstrip("/"))
        self.device, self.conf, self.imgsz = device, conf, imgsz
        self.device_shown = device
        try:                                    # ultralytics 는 장치가 없으면 조용히 AUTO 로 감 → 상태에 실제 장치 표시
            import openvino as ov
            want = device.split(":")[-1].upper()
            if device.startswith("intel:") and want not in {d.split(".")[0] for d in ov.Core().available_devices}:
                self.device_shown = f"{device}→AUTO"
                print(f"[head_track] 오른눈 {want} 없음 → AUTO ({', '.join(ov.Core().available_devices)})")
        except Exception:                       # noqa: BLE001
            pass
        self.exclude = set(exclude)
        names = self.model.names
        self.keep = None if not self.exclude else [i for i, n in names.items() if n not in self.exclude]

    def process(self, img, now, dt):
        r = self.model(img, conf=self.conf, imgsz=self.imgsz, device=self.device, classes=self.keep, verbose=False)[0]
        dets = []
        if r.boxes is not None and len(r.boxes):
            for (x0, y0, x1, y1), c, k in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(), r.boxes.cls.cpu().numpy()):
                dets.append(dict(label=r.names[int(k)], conf=float(c), box=(float(x0), float(y0), float(x1), float(y1))))
        v = img.copy()
        for d in dets:
            x0, y0, x1, y1 = (int(t) for t in d["box"])
            cv2.rectangle(v, (x0, y0), (x1, y1), (255, 150, 40), 2)
            cv2.putText(v, f"{d['label']} {d['conf']:.2f}", (x0 + 2, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 150, 40), 1, cv2.LINE_AA)
        cv2.putText(v, f"R objects  {self.fps:.1f} fps  det {self.det_ms:.0f} ms", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return dets, v

    def status(self):
        return {"fps": round(self.fps, 1), "det_ms": round(self.det_ms, 1), "model": f"{self.model_name}@{self.device_shown}",
                "error": self.err}


def crop_tile(img, box, label, color, h):
    """인식 박스를 10 % 넓혀 잘라 높이 h 로 + 아래 이름표."""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = box
    mx, my = 0.1 * (x1 - x0), 0.1 * (y1 - y0)
    x0, y0 = int(max(0, x0 - mx)), int(max(0, y0 - my))
    x1, y1 = int(min(W, x1 + mx)), int(min(H, y1 + my))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    c = img[y0:y1, x0:x1]
    cw = int(np.clip(round(c.shape[1] * h / c.shape[0]), 0.6 * h, 1.8 * h))
    c = cv2.resize(c, (cw, h))
    w = max(cw, cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0] + 6)   # 이름표가 잘리지 않을 폭
    if w > cw:                                                                     # 좁은 그림은 가운데 두고 양옆 여백
        pad = np.full((h, w, 3), 24, np.uint8)
        pad[:, (w - cw) // 2:(w - cw) // 2 + cw] = c
        c = pad
    bar = np.full((18, w, 3), 24, np.uint8)
    bar[:3] = color
    cv2.putText(bar, label, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1, cv2.LINE_AA)
    return np.vstack([c, bar])


def build_strip(left, right):
    """두 눈에서 지금 인식된 것만 잘라 한 줄 (왼눈 → 오른눈). 없으면 안내 한 줄."""
    s = cfg("strip", {}) or {}
    h, gap = int(s.get("height", 72)), 4
    tiles, now, seen = [], time.time(), False
    for eye, maxn in ((left, int(s.get("max_left", 3))), (right, int((cfg("objects", {}) or {}).get("max", 6)))):
        if eye is None:
            continue
        with eye.lock:
            last = eye.last
        if last is None or now - last[2] > FRESH_S:
            continue
        seen = True
        img, dets, _ = last
        for d in sorted(dets, key=lambda d: -d["conf"])[:maxn]:
            color = {"face": (0, 220, 120), "person": (0, 160, 255)}.get(d["label"], (255, 150, 40))
            t = crop_tile(img, d["box"], f"{d['label']} {d['conf']:.2f}", color, h)
            if t is not None:
                tiles.append(t)
    if not tiles:                       # 영상은 오는데 인식 없음 / 영상 자체가 안 옴 (앱 video_hub 끔·Stereo patch PC1 켬 확인)
        msg = ["no detection"] if seen else ["no head-cam stream", "app: video_hub off, Stereo patch PC1 on"]
        out = np.full((h + 18, 330, 3), 18, np.uint8)
        y = (h + 18) // 2 + 5 - 10 * (len(msg) - 1)
        for i, m in enumerate(msg):
            cv2.putText(out, m, (10, y + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55 if i == 0 else 0.42,
                        (130, 130, 130) if seen else (90, 150, 230), 1, cv2.LINE_AA)
        return out
    sep = np.full((h + 18, gap, 3), 14, np.uint8)
    parts = []
    for t in tiles:
        parts += [t, sep]
    return np.hstack(parts[:-1])


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H2 머리 카메라 인식</title><style>body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
img{max-width:100%;border:1px solid #444}.row{display:flex;gap:8px;flex-wrap:wrap}.row img{width:calc(50% - 6px);min-width:280px}
button{font-size:16px;margin:4px;padding:6px 14px}pre{background:#222;padding:8px;white-space:pre-wrap}</style></head><body>
<h3>H2 머리 카메라 — 왼눈 얼굴·사람 / 오른눈 사물</h3>
<div><img src="/strip" style="height:96px;width:auto"></div>
<div class="row"><img src="/video_feed"><img src="/video_right"></div><div>
<button onclick="post('/track',{enable:true})">추종 켜기</button><button onclick="post('/track',{enable:false})">추종 끄기</button>
<button onclick="post('/home',{})">정면(home)</button></div><pre id="s"></pre>
<script>function post(u,b){fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)}).then(r=>r.json()).then(j=>{if(j.detail)alert(j.detail)})}
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('s').textContent=JSON.stringify(j,null,1)}),1000)</script></body></html>"""


class TrackReq(BaseModel):
    enable: bool


class LookReq(BaseModel):
    pitch: float = 0.0
    yaw: float = 0.0
    speed_deg: float = 30.0


def mjpeg(get, period=0.05):
    def gen():
        last = None
        while True:
            j = get()
            if j is not None and j is not last:
                last = j
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n"
            time.sleep(period)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


def make_app(tr, ob, strip):
    app = FastAPI(title="H2 head vision")

    def latest(eye):
        if eye is None:
            return None
        with eye.lock:
            return eye.jpeg

    @app.get("/", response_class=HTMLResponse)
    def index():
        return PAGE

    @app.get("/status")
    def status():
        return {"left": tr.status(), "right": ob.status() if ob else None}

    @app.get("/detections")
    def detections():
        out, now = [], time.time()
        for eye in (tr, ob):
            if eye is None:
                continue
            with eye.lock:
                last = eye.last
            if last is not None and now - last[2] <= FRESH_S:
                out += [dict(eye=eye.name, label=d["label"], conf=round(d["conf"], 3), box=[round(v) for v in d["box"]])
                        for d in last[1]]
        return JSONResponse(out, headers={"Access-Control-Allow-Origin": "*"})

    @app.get("/strip")
    def strip_feed():
        return mjpeg(lambda: strip["jpeg"], 0.1)

    @app.get("/strip.jpg")
    def strip_jpg():
        j = strip["jpeg"]
        if j is None:
            raise HTTPException(503, "아직 없음")
        return Response(j, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/video_feed")
    def video_feed():
        return mjpeg(lambda: latest(tr))

    @app.get("/video_right")
    def video_right():
        return mjpeg(lambda: latest(ob))

    @app.post("/track")
    def track(req: TrackReq):
        if req.enable and not tr.drive:
            raise HTTPException(409, "보기 전용으로 실행됨 — --drive 로 다시 실행")
        tr.look = None
        tr.enabled = req.enable
        return tr.status()

    @app.post("/look")
    def look(req: LookReq):
        if not tr.drive:
            raise HTTPException(409, "보기 전용으로 실행됨 — --drive 로 다시 실행")
        tr.enabled = False
        tr.look = (req.pitch, req.yaw, max(1.0, min(req.speed_deg, 60.0)))
        return tr.status()

    @app.post("/home")
    def home():
        h = cfg("home_deg", {"pitch": 10.0, "yaw": 0.0})
        return look(LookReq(pitch=h["pitch"], yaw=h["yaw"], speed_deg=float(cfg("home_speed_deg", 15.0))))

    return app


def main():
    ap = argparse.ArgumentParser(description="H2 머리 카메라 인식 (왼눈 얼굴·사람, 오른눈 사물)")
    ap.add_argument("--drive", action="store_true", help="arm_server /head 로 머리 명령 가능 (없으면 보기만)")
    ap.add_argument("--track-on", action="store_true", help="--drive 와 같이: 시작하자마자 추종 (기본은 웹에서 '추종 켜기')")
    ap.add_argument("--mode", choices=("face", "person", "auto"), default=cfg("mode", "auto"))
    ap.add_argument("--device", default=cfg("device", "CPU"), help="얼굴·사람 OpenVINO 장치 CPU / GPU / NPU / AUTO")
    ap.add_argument("--no-objects", action="store_true", help="오른눈 사물 인식 끔")
    ap.add_argument("--port", type=int, default=int(cfg("port", 50013)))
    ap.add_argument("--face-conf", type=float, default=None, help="robot.yaml head_track.face_conf 대신")
    ap.add_argument("--person-conf", type=float, default=None, help="robot.yaml head_track.person_conf 대신")
    a = ap.parse_args()
    if a.face_conf is not None:
        C["face_conf"] = a.face_conf
    if a.person_conf is not None:
        C["person_conf"] = a.person_conf

    tr = Tracker(RGB_PORTS["left"], a.drive, a.mode, a.device, a.track_on)
    ob = None
    oc = cfg("objects", {}) or {}
    if not a.no_objects and oc.get("enabled", True):
        path = os.path.join(MODEL_DIR, oc.get("model", "yolo11s_openvino_model"))
        if os.path.exists(path):
            ob = Objects(RGB_PORTS["right"], path, oc.get("device", "intel:cpu"), float(oc.get("conf", 0.45)),
                         int(oc.get("imgsz", 448)), oc.get("exclude", ["person"]))
        else:
            print(f"[head_track] ⚠️ 사물 모델 없음 ({path}) — 오른눈 인식 끔")
    if a.drive:
        hs, err = arm_head()
        print(f"[head_track] arm_server 머리: {hs if hs else err}")
    print(f"[head_track] 왼눈 {[m.name + '@' + m.device for m in (tr.face, tr.person) if m]}"
          + (f", 오른눈 {ob.model_name}@{ob.device_shown}" if ob else ", 오른눈 끔")
          + f", {cfg('fps', 5.0)} fps, {'머리 명령' if a.drive else '보기만'} →\n    {web_urls(a.port, PC1)}")

    strip = {"jpeg": None}

    def strip_loop():
        while True:
            strip["jpeg"] = jpeg(build_strip(tr, ob), 85)
            time.sleep(0.2)
    tr.start()
    if ob is not None:
        ob.start()
    threading.Thread(target=strip_loop, daemon=True).start()
    uvicorn.run(make_app(tr, ob, strip), host="0.0.0.0", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
