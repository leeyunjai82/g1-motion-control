#!/usr/bin/env python3
"""
head_track.py — H2 머리 추종 서버 (포트 50013): 머리 카메라 왼눈 영상에서 가장 큰 얼굴(없으면 사람)을 화면 가운데로

  ROBOT=h2 python common/head_track.py            # 보기만 (머리 명령 안 함) → http://<pc-ip>:50013/
  ROBOT=h2 python common/head_track.py --drive    # 머리 명령 준비 — 웹 '추종 켜기' 를 눌러야 움직임 (arm_server :50022 필요)
  ROBOT=h2 python common/head_track.py --drive --track-on   # 시작하자마자 추종

  준비: 앱에서 video_hub 끔 + 'Stereo patch PC1' 켬, RGB 수신 IP = 이 PC
        (python utils/check_head_cam.py --set-ip → 앱에서 서비스 재시작). gstreamer 설치는 common/ctrl/head_cam.py 참고
  검출: OpenVINO Open Model Zoo face-detection-retail-0004 (300×300) / person-detection-0201 (384×384)
        → common/models/omz. 설정은 robot.yaml head_track (mode, gain, deadband, max_speed_deg, hfov_deg …)

  제어 (카메라가 머리와 같이 돌므로 화면 오차만 0 으로 — 내부 파라미터·카메라 위치 보정 필요 없음)
    e = (대상 점 − 화면 중심) / (화면 반폭, 반높이)   ∈ [−1, 1]
    원하는 각 = 프레임 받을 때의 실측 머리각 + gain · (pitch: +e_y · vfov/2, yaw: −e_x · hfov/2)   (pitch + = 숙임, yaw + = 왼쪽)
    |e| < deadband 인 축은 지금 명령 유지. 프레임마다 변화량 ≤ max_speed_deg · dt
    대상 점: 얼굴 = 박스 중심, 사람 = 박스 위에서 person_aim 비율 아래 (≈ 머리)
    대상 고르기: 가장 큰 것. 단 지금 대상(가장 가까운 박스)보다 switch_ratio 배 이상 클 때만 바꿈
    새 대상은 confirm_frames 프레임 연속 보여야 따라감 (한 프레임 오검출에 머리가 튀지 않게)
    대상이 lost_s 동안 없으면 home_deg 로 home_speed_deg 속도로

  API
    GET  /             화면 (영상 + 상태 + 추종 켜기/끄기)
    GET  /video_feed   MJPEG (검출 박스, 대상, 머리각)
    GET  /status       JSON
    POST /track        {"enable": true|false}  끄면 머리는 그 자리에 둠
    POST /look         {"pitch": deg, "yaw": deg}  추종 끄고 이 각도로 (잡기 시퀀스에서 박스 보기 등)
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
from fastapi.responses import HTMLResponse, StreamingResponse   # noqa: E402
from pydantic import BaseModel   # noqa: E402

C = dict(robot_env.CFG.get("head_track") or {})
ARM_SERVER = "http://localhost:50022"


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


class Tracker(threading.Thread):
    def __init__(self, port, drive, mode, device, track_on=False):
        super().__init__(daemon=True)
        self.port, self.drive, self.mode = port, drive, mode
        self.rx = None
        self.face = OvSSD("face", device, cfg("face_conf", 0.6)) if mode in ("face", "auto") else None
        self.person = OvSSD("person", device, cfg("person_conf", 0.5)) if mode in ("person", "auto") else None
        self.enabled = drive and track_on   # --drive 만이면 꺼진 채 시작 → 웹 '추종 켜기'
        self.lock = threading.Lock()
        self.jpeg = None
        self.cmd = None                    # 마지막으로 보낸 머리 목표 [pitch, yaw] deg
        self.meas = None                   # arm_server 실측 [pitch, yaw] deg
        self.look = None                   # /look·/home 요청 (추종 끈 상태에서 보낼 각)
        self.tgt = None                    # 지금 대상 {"box", "kind", "pt", "area"}
        self.last_seen = 0.0
        self.last_face = 0.0
        self.streak = 0                    # 후보가 연속으로 보인 프레임 수
        self.active = False                # 확인된 대상을 따라가는 중 (lost_s 동안 유지)
        self.cands = []
        self.fps = 0.0
        self.det_ms = 0.0
        self.err = "머리 카메라 RGB 기다리는 중"
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

    def _cands(self, faces, persons):
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
        if self.look is not None:                                  # /look, /home
            want, speed = np.array(self.look[:2], float), float(self.look[2])
        elif not self.enabled:
            return
        elif self.tgt is not None:
            H, W = img_shape[:2]
            ex = (self.tgt["pt"][0] - W / 2) / (W / 2)
            ey = (self.tgt["pt"][1] - H / 2) / (H / 2)
            g, db = float(cfg("gain", 0.5)), float(cfg("deadband", 0.06))
            want = self.cmd.copy()
            if abs(ey) > db:
                want[0] = self.meas[0] + g * ey * float(cfg("vfov_deg", 75.0)) / 2
            if abs(ex) > db:
                want[1] = self.meas[1] - g * ex * float(cfg("hfov_deg", 90.0)) / 2
            speed = float(cfg("max_speed_deg", 40.0))
        elif now - self.last_seen > float(cfg("lost_s", 2.5)):
            h = cfg("home_deg", {"pitch": 10.0, "yaw": 0.0})
            want, speed = np.array([h["pitch"], h["yaw"]], float), float(cfg("home_speed_deg", 15.0))
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

    # ---- 화면 ----
    def _draw(self, img, faces, persons):
        v = img.copy()
        H, W = v.shape[:2]
        for b in persons:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (255, 160, 0), 1)
        for b in faces:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 200, 255), 1)
        for c in self.cands:                                       # 확인 전 후보 (노랑)
            if self.tgt is None:
                cv2.circle(v, (int(c["pt"][0]), int(c["pt"][1])), 4, (0, 255, 255), -1)
        if self.tgt is not None:
            b = self.tgt["box"]
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
            cv2.circle(v, (int(self.tgt["pt"][0]), int(self.tgt["pt"][1])), 4, (0, 255, 0), -1)
        db = float(cfg("deadband", 0.06))
        cv2.rectangle(v, (int(W / 2 * (1 - db)), int(H / 2 * (1 - db))), (int(W / 2 * (1 + db)), int(H / 2 * (1 + db))),
                      (255, 255, 255), 1)
        head = "-" if self.meas is None else f"pitch {self.meas[0]:+.1f}  yaw {self.meas[1]:+.1f}"
        state = "TRACK" if (self.enabled and self.drive) else ("VIEW" if not self.drive else "OFF")
        cv2.putText(v, f"{state}  {self.fps:.1f} fps  det {self.det_ms:.0f} ms  head {head}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return v

    def run(self):
        while self.rx is None:                                     # 머리 카메라 RGB 기다림 (서비스가 나중에 켜져도 됨)
            rx = RgbRx(self.port)
            if rx.probe(6.0):
                rx.start()
                self.rx = rx
                self.err = ""
                print(f"[head_track] 머리 카메라 RGB {rx.size[0]}×{rx.size[1]} (UDP {self.port})")
            else:
                self.err = (f"머리 카메라 RGB 없음 (UDP {self.port}) — 앱 video_hub 끔 · Stereo patch PC1 켬 · "
                            f"수신 IP = {my_ip(PC1)} 인지 확인")
        last_t, t_fps, n_fps = 0.0, time.time(), 0
        while True:
            f = self.rx.latest()
            if f is None or f[1] == last_t:
                if last_t and time.time() - last_t > 1.0:
                    self.err, self.fps = f"머리 카메라 RGB 끊김 (UDP {self.port})", 0.0
                time.sleep(0.005)
                continue
            if self.err.startswith("머리 카메라 RGB 끊김"):
                self.err = ""
            img, t = f
            dt = float(np.clip(t - last_t, 0.02, 0.3)) if last_t else 0.1
            last_t = t
            now = time.time()
            t0 = time.time()
            faces = self.face(img) if self.face is not None else []
            if faces:
                self.last_face = now
            persons = []
            if self.person is not None and (self.mode == "person" or
                                            (not faces and now - self.last_face > float(cfg("person_after_s", 1.0)))):
                persons = self.person(img)
            self.det_ms = (time.time() - t0) * 1000
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
            ok, jpg = cv2.imencode(".jpg", self._draw(img, faces, persons), [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.lock:
                    self.jpeg = jpg.tobytes()
            n_fps += 1
            if now - t_fps >= 1.0:
                self.fps, t_fps, n_fps = n_fps / (now - t_fps), now, 0

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


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H2 머리 추종</title><style>body{font-family:sans-serif;margin:12px;background:#111;color:#eee}img{max-width:100%;border:1px solid #444}
button{font-size:16px;margin:4px;padding:6px 14px}pre{background:#222;padding:8px;white-space:pre-wrap}</style></head><body>
<h3>H2 머리 추종 (얼굴 → 사람)</h3><img src="/video_feed"><div>
<button onclick="post('/track',{enable:true})">추종 켜기</button><button onclick="post('/track',{enable:false})">추종 끄기</button>
<button onclick="post('/home',{})">정면(home)</button></div><pre id="s"></pre>
<script>function post(u,b){fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)}).then(r=>r.json()).then(j=>{if(j.detail)alert(j.detail)})}
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('s').textContent=JSON.stringify(j,null,1)}),500)</script></body></html>"""


class TrackReq(BaseModel):
    enable: bool


class LookReq(BaseModel):
    pitch: float = 0.0
    yaw: float = 0.0
    speed_deg: float = 30.0


def make_app(tr):
    app = FastAPI(title="H2 head track")

    @app.get("/", response_class=HTMLResponse)
    def index():
        return PAGE

    @app.get("/status")
    def status():
        return tr.status()

    @app.get("/video_feed")
    def video_feed():
        def gen():
            last = None
            while True:
                with tr.lock:
                    j = tr.jpeg
                if j is not None and j is not last:
                    last = j
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n"
                time.sleep(0.03)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

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
    ap = argparse.ArgumentParser(description="H2 머리 추종 (얼굴 → 사람)")
    ap.add_argument("--drive", action="store_true", help="arm_server /head 로 머리 명령 가능 (없으면 보기만)")
    ap.add_argument("--track-on", action="store_true", help="--drive 와 같이: 시작하자마자 추종 (기본은 웹에서 '추종 켜기')")
    ap.add_argument("--mode", choices=("face", "person", "auto"), default=cfg("mode", "auto"))
    ap.add_argument("--device", default=cfg("device", "CPU"), help="OpenVINO 장치 CPU / GPU / NPU / AUTO")
    ap.add_argument("--rgb", choices=("left", "right"), default="left")
    ap.add_argument("--port", type=int, default=int(cfg("port", 50013)))
    ap.add_argument("--face-conf", type=float, default=None, help="robot.yaml head_track.face_conf 대신")
    ap.add_argument("--person-conf", type=float, default=None, help="robot.yaml head_track.person_conf 대신")
    a = ap.parse_args()
    if a.face_conf is not None:
        C["face_conf"] = a.face_conf
    if a.person_conf is not None:
        C["person_conf"] = a.person_conf
    tr = Tracker(RGB_PORTS[a.rgb], a.drive, a.mode, a.device, a.track_on)
    if a.drive:
        hs, err = arm_head()
        print(f"[head_track] arm_server 머리: {hs if hs else err}")
    print(f"[head_track] 모델 {[m.name + '@' + m.device for m in (tr.face, tr.person) if m]}, "
          f"{'머리 명령' if a.drive else '보기만'} →\n    {web_urls(a.port, PC1)}")
    tr.start()
    uvicorn.run(make_app(tr), host="0.0.0.0", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
