#!/usr/bin/env python3
"""
check_head_cam.py — H2 머리 쌍안 카메라 수신 확인: 왼눈(또는 오른눈·원본) RGB + 깊이 (로봇 명령 없음)

  공식 문서 'Bilateral Data Stream Acquisition Interface' (support.unitree.com H2_developer, 2026-08-11 판) 기준
    RGB  : PC1 이 RTP/H264 UDP 유니캐스트로 '수신 IP' 에 보냄 (기본 192.168.123.170)
           5002 원본 쌍안 1920×1080 15 fps (왜곡 보정 전) / 5004 왼눈 544×448 10 fps / 5006 오른눈 544×448 10 fps
    깊이 : PC1(192.168.123.161) TCP 5000 에 접속해서 받음 — Y16 544×448 10 fps, 단위 mm (0·65535 = 무효)
           프레임 = 헤더 36 B (<IQQIIII: magic 'Y16 ' 0x59313620, seq, 보낸 쪽 시각 us, w, h, format, data_size) + w·h·2 B
           (공식 unitree-dep-img 1.0.0 deb 의 dep_img_client.c 와 같은 형식 → deb·v4l2loopback 없이 직접 읽음)

  준비
    로봇: 앱에서 video_hub 끄기 (충돌) → 'Stereo patch PC1' 서비스 켜기 (기본 자동 시작 아님)
          RGB 수신 IP 를 이 PC 로: --set-ip (= curl "http://192.168.123.161:9080/set?ip=<이 PC IP>") → 앱에서 서비스 껐다 켜기
    이 PC: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                            gstreamer1.0-plugins-ugly gstreamer1.0-libav
           방화벽: sudo ufw allow from 192.168.123.0/24
    공식 unitree-dep-img 와 동시에 켜지 말 것 (깊이 서버 동시 접속 가능 여부 확인 필요)

  python utils/check_head_cam.py                       # 창: RGB | 깊이 | 겹쳐 보기 (q 종료, s 저장, 클릭 = 그 점 깊이)
  python utils/check_head_cam.py --sec 10              # 창 없이 10 초 통계 (해상도, fps, 깊이 유효율·가운데 값)
  python utils/check_head_cam.py --save logs/headcam   # 1 초마다 RGB png + 깊이 npy(uint16 mm) 저장 (내부 파라미터·정렬 확인용)
  python utils/check_head_cam.py --rgb raw             # 5002 원본 1920×1080 쌍안
  python utils/check_head_cam.py --set-ip              # RGB 수신 IP 를 이 PC 로 바꾸고 끝 (그다음 앱에서 서비스 재시작)

확인 필요 (문서에 없음): 내부 파라미터, 깊이가 왼눈 544×448 과 픽셀 정렬인지, 머리 링크 기준 카메라 위치
                        (URDF 에 카메라 프레임 없음 — 머리 pitch 29 / yaw 30 관절 따라 움직임).
"""
import argparse
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import threading
import time
import urllib.request

import cv2
import numpy as np

PC1 = "192.168.123.161"
RGB_PORTS = {"left": 5004, "right": 5006, "raw": 5002}
RTP_CAPS = "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
MAGIC = 0x59313620                       # 'Y16 '
HDR = struct.Struct("<IQQIIII")          # 36 B, packed
MAGIC_BYTES = struct.pack("<I", MAGIC)
WIN = "H2 head camera (q quit, s save, click = depth)"


def my_ip(peer):
    """peer 로 나가는 이 PC 의 IP (패킷은 보내지 않음)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((peer, 9))
        return s.getsockname()[0]
    except OSError:
        return "?"
    finally:
        s.close()


class DepthRx(threading.Thread):
    """PC1 TCP 깊이 수신 (끊기면 1 초 뒤 다시 접속)."""

    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.frame = None            # (uint16 h×w [mm], seq, 보낸 시각 us, 받은 시각 s)
        self.count = 0
        self.seq_gaps = 0
        self.err = "아직 접속 안 됨"
        self.stop = False

    @staticmethod
    def _read(sock, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("서버가 연결을 끊음")
            buf += chunk
        return bytes(buf)

    def run(self):
        last_seq = None
        while not self.stop:
            try:
                with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
                    sock.settimeout(3.0)
                    self.err = ""
                    buf = b""
                    while not self.stop:
                        buf += self._read(sock, HDR.size - len(buf))
                        magic, seq, ts, w, h, _fmt, size = HDR.unpack(buf)
                        if magic != MAGIC:                       # 동기 다시 맞추기
                            k = buf.find(MAGIC_BYTES, 1)
                            buf = buf[k:] if k >= 0 else buf[-3:]
                            continue
                        if not (0 < w <= 4096 and 0 < h <= 4096) or size != w * h * 2:
                            raise ValueError(f"헤더 이상: {w}x{h}, data_size {size}")
                        data = self._read(sock, size)
                        buf = b""
                        d = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
                        with self.lock:
                            if last_seq is not None and seq != last_seq + 1:
                                self.seq_gaps += 1
                            last_seq = seq
                            self.frame = (d, seq, ts, time.time())
                            self.count += 1
            except (OSError, ConnectionError, ValueError) as e:
                self.err = f"{self.host}:{self.port} {e}"
                last_seq = None
                time.sleep(1.0)

    def latest(self):
        with self.lock:
            return self.frame


class RgbRx(threading.Thread):
    """gst-launch-1.0 로 RTP/H264 → BGR 원시 프레임을 stdout 으로 받아 읽음 (pip opencv 는 GStreamer 없음)."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.lock = threading.Lock()
        self.frame = None            # (BGR h×w×3, 받은 시각 s)
        self.size = None             # (w, h)
        self.caps_fps = "?"
        self.count = 0
        self.err = "아직 수신 없음"
        self.stop = False
        self.proc = None

    def probe(self, timeout):
        """공식 문서 방식 (fakesink -v 의 caps) 으로 해상도·fps 확인. 성공하면 True."""
        cmd = ["gst-launch-1.0", "-v", "udpsrc", f"port={self.port}", "buffer-size=2097152", "!", RTP_CAPS, "!",
               "rtph264depay", "!", "avdec_h264", "!", "fakesink"]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        found = []

        def reader():
            for line in p.stdout:
                m = re.search(r"video/x-raw.*?width=\(int\)(\d+).*?height=\(int\)(\d+)", line)
                if m:
                    fr = re.search(r"framerate=\(fraction\)(\d+/\d+)", line)
                    found.append((int(m.group(1)), int(m.group(2)), fr.group(1) if fr else "?"))
                    return
        t = threading.Thread(target=reader, daemon=True)
        t.start()
        t.join(timeout)
        p.kill()
        p.wait()
        if not found:
            return False
        w, h, self.caps_fps = found[0]
        self.size = (w, h)
        return True

    def run(self):
        w, h = self.size
        stride = (w * 3 + 3) // 4 * 4                   # GStreamer BGR 줄 정렬 4 B
        n = stride * h
        cmd = ["gst-launch-1.0", "-q", "udpsrc", f"port={self.port}", "buffer-size=2097152", "!", RTP_CAPS, "!",
               "rtph264depay", "!", "avdec_h264", "!", "videoconvert", "!", f"video/x-raw,format=BGR,width={w},height={h}",
               "!", "fdsink", "fd=1", "sync=false"]
        while not self.stop:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            self.err = ""
            try:
                while not self.stop:
                    buf = bytearray()
                    while len(buf) < n:
                        chunk = self.proc.stdout.read(n - len(buf))
                        if not chunk:
                            raise ConnectionError("gst-launch 종료")
                        buf += chunk
                    img = np.frombuffer(bytes(buf), np.uint8).reshape(h, stride)[:, :w * 3].reshape(h, w, 3)
                    with self.lock:
                        self.frame = (img, time.time())
                        self.count += 1
            except ConnectionError as e:
                self.err = str(e)
                time.sleep(1.0)
            finally:
                self.kill()

    def kill(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()

    def latest(self):
        with self.lock:
            return self.frame


def colorize(d, rng):
    """uint16 mm → 컬러 (무효 = 진회색)."""
    lo, hi = rng
    m = d.astype(np.float32) / 1000.0
    bad = (d == 0) | (d == 65535)
    u8 = (np.clip((m - lo) / (hi - lo), 0, 1) * 255).astype(np.uint8)
    c = cv2.applyColorMap(255 - u8, cv2.COLORMAP_JET)       # 가까울수록 빨강
    c[bad] = (40, 40, 40)
    return c


def depth_at(d, u, v, r=1):
    """(u, v) 둘레 (2r+1)² 의 유효 깊이 중앙값 [mm], 없으면 0."""
    p = d[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].ravel()
    p = p[(p > 0) & (p < 65535)]
    return int(np.median(p)) if len(p) else 0


def depth_stats(d):
    valid = (d > 0) & (d < 65535)
    h, w = d.shape
    c = depth_at(d, w // 2, h // 2, 10)
    if valid.any():
        v = d[valid]
        lo, hi = np.percentile(v, 1), np.percentile(v, 99)
    else:
        lo = hi = 0
    return valid.mean() * 100, c, lo, hi


def save_pair(out, idx, rgb, dep, rng):
    os.makedirs(out, exist_ok=True)
    meta = {"i": idx, "t": time.time()}
    if rgb is not None:
        cv2.imwrite(f"{out}/rgb_{idx:04d}.png", rgb[0])
        meta.update(rgb_t=rgb[1], rgb_size=list(rgb[0].shape[1::-1]))
    if dep is not None:
        np.save(f"{out}/depth_{idx:04d}.npy", dep[0])
        cv2.imwrite(f"{out}/depth_{idx:04d}.png", colorize(dep[0], rng))
        meta.update(depth_t=dep[3], depth_seq=dep[1], depth_send_us=dep[2], depth_size=list(dep[0].shape[::-1]))
    if rgb is not None and dep is not None:
        meta["rgb_minus_depth_ms"] = round((rgb[1] - dep[3]) * 1000, 1)
    with open(f"{out}/meta.jsonl", "a") as f:
        f.write(json.dumps(meta) + "\n")
    print(f"[저장] {out} #{idx:04d}" + (f"  RGB−깊이 받은 시각 차 {meta['rgb_minus_depth_ms']} ms" if "rgb_minus_depth_ms" in meta else ""))


def report(rgb_rx, dep_rx, sec):
    print(f"\n=== {sec:.0f} 초 결과 ===")
    if rgb_rx is not None:
        if rgb_rx.count:
            w, h = rgb_rx.size
            print(f"  RGB  (UDP {rgb_rx.port}) {w}×{h}, caps {rgb_rx.caps_fps} fps, 받은 {rgb_rx.count / sec:.1f} fps")
        else:
            print(f"  RGB  (UDP {rgb_rx.port}) ❌ 수신 없음 — {rgb_rx.err}")
    if dep_rx is not None:
        f = dep_rx.latest()
        if f is not None:
            d, seq, ts, tr = f
            ok, c, lo, hi = depth_stats(d)
            print(f"  깊이 (TCP {dep_rx.host}:{dep_rx.port}) {d.shape[1]}×{d.shape[0]}, 받은 {dep_rx.count / sec:.1f} fps, "
                  f"seq 끊김 {dep_rx.seq_gaps}회, 마지막 seq {seq}")
            print(f"        유효 {ok:.0f} %, 가운데 21×21 {c / 1000:.3f} m, 유효 1–99 % {lo / 1000:.3f}–{hi / 1000:.3f} m")
            print(f"        (참고) 받은 시각 − 보낸 시각 {(tr - ts / 1e6) * 1000:+.0f} ms — 로봇·PC 시계가 맞아야 의미 있음")
        else:
            print(f"  깊이 (TCP {dep_rx.host}:{dep_rx.port}) ❌ 수신 없음 — {dep_rx.err}")


def main():
    ap = argparse.ArgumentParser(description="H2 머리 쌍안 카메라 RGB + 깊이 수신 확인")
    ap.add_argument("--host", default=PC1, help="PC1 IP (깊이 서버·수신 IP 설정 서버)")
    ap.add_argument("--depth-port", type=int, default=5000)
    ap.add_argument("--rgb", choices=("left", "right", "raw", "none"), default="left")
    ap.add_argument("--rgb-port", type=int, default=None, help="RTP 포트 직접 지정 (기본 left 5004 / right 5006 / raw 5002)")
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--sec", type=float, default=0.0, help="창 없이 이 시간 받은 뒤 통계 출력 (0 = 창 띄움)")
    ap.add_argument("--save", default="", help="저장 폴더 (RGB png, 깊이 npy uint16 mm + png, meta.jsonl)")
    ap.add_argument("--save-every", type=float, default=1.0, help="자동 저장 간격 [s] (0 = s 키로만)")
    ap.add_argument("--range", default="0.3,3.0", help="깊이 색 범위 [m]")
    ap.add_argument("--probe-sec", type=float, default=6.0, help="RGB 스트림 기다리는 시간 [s]")
    ap.add_argument("--set-ip", action="store_true", help="RGB 수신 IP 를 이 PC 로 설정하고 끝 (그다음 앱에서 서비스 재시작)")
    a = ap.parse_args()
    rng = tuple(float(v) for v in a.range.split(","))
    me = my_ip(a.host)
    print(f"[head_cam] 이 PC IP {me} (PC1 {a.host} 방향) — RGB 는 로봇에 설정된 수신 IP 로만 옴 (기본 192.168.123.170)")

    if a.set_ip:
        url = f"http://{a.host}:9080/set?ip={me}"
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                body = r.read(300).decode(errors="replace")
            print(f"[head_cam] {url} → HTTP {r.status} {body.strip()[:200]}")
            print("[head_cam] 앱에서 'Stereo patch PC1' 서비스를 껐다 켜야 적용됩니다 (공식 문서).")
        except OSError as e:
            print(f"[head_cam] ❌ {url} 실패: {e} — 앱에서 서비스가 켜져 있는지 확인")
        return

    dep_rx = None
    if not a.no_depth:
        dep_rx = DepthRx(a.host, a.depth_port)
        dep_rx.start()

    rgb_rx = None
    if a.rgb != "none":
        if shutil.which("gst-launch-1.0") is None:
            print("[head_cam] ❌ gst-launch-1.0 없음 → sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good "
                  "gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav  (RGB 없이 깊이만 진행)")
        else:
            rgb_rx = RgbRx(a.rgb_port or RGB_PORTS[a.rgb])
            print(f"[head_cam] RGB UDP {rgb_rx.port} 기다리는 중 ({a.probe_sec:.0f} 초)...")
            if rgb_rx.probe(a.probe_sec):
                print(f"[head_cam] RGB {rgb_rx.size[0]}×{rgb_rx.size[1]}, caps {rgb_rx.caps_fps} fps")
                rgb_rx.start()
            else:
                print(f"[head_cam] ❌ RGB UDP {rgb_rx.port} 수신 없음 — 확인: 앱 video_hub 끔 · Stereo patch PC1 켬 · "
                      f"수신 IP = {me} (--set-ip 후 서비스 재시작) · ufw. 깊이만 진행")
                rgb_rx = None
    if rgb_rx is None and dep_rx is None:
        return

    for rx in (rgb_rx, dep_rx):              # fps 는 여기서부터 (RGB 기다리는 동안 받은 깊이 빼기)
        if rx is not None:
            with rx.lock:
                rx.count = 0
                if rx is dep_rx:
                    rx.seq_gaps = 0
    t0 = time.time()
    t_save = 0.0
    idx = 0
    gui = False
    try:
        if a.sec > 0:
            while time.time() - t0 < a.sec:
                if a.save and a.save_every > 0 and time.time() - t_save >= a.save_every:
                    rgb, dep = (rgb_rx.latest() if rgb_rx else None), (dep_rx.latest() if dep_rx else None)
                    if rgb is not None or dep is not None:
                        save_pair(a.save, idx, rgb, dep, rng); idx += 1; t_save = time.time()
                time.sleep(0.05)
            report(rgb_rx, dep_rx, time.time() - t0)
            return

        click = {}
        layout = {}

        def on_mouse(ev, x, y, _flags, _p):
            if ev != cv2.EVENT_LBUTTONDOWN or "panels" not in layout:
                return
            for name, (x0, pw, sx, sy) in layout["panels"].items():
                if x0 <= x < x0 + pw:
                    click["uv"] = (int((x - x0) * sx), int(y * sy), name)
        try:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WIN, on_mouse)
        except cv2.error:
            print("[head_cam] ❌ 창을 띄울 수 없음 (화면 없음 / opencv-python-headless) → --sec 10 으로 실행")
            return
        gui = True
        last_status = 0.0
        while True:
            rgb = rgb_rx.latest() if rgb_rx else None
            dep = dep_rx.latest() if dep_rx else None
            H = dep[0].shape[0] if dep is not None else (min(rgb[0].shape[0], 540) if rgb is not None else 448)
            panels, layout["panels"], x0 = [], {}, 0
            if rgb is not None:
                img = rgb[0]
                pw = int(round(img.shape[1] * H / img.shape[0]))
                panels.append(cv2.resize(img, (pw, H)))
                layout["panels"]["rgb"] = (x0, pw, img.shape[1] / pw, img.shape[0] / H); x0 += pw
            if dep is not None:
                d = dep[0]
                dh, dw = d.shape
                c = colorize(d, rng)
                cd = depth_at(d, dw // 2, dh // 2, 10)
                cv2.drawMarker(c, (dw // 2, dh // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(c, f"center {cd / 1000:.3f} m", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                pw = int(round(dw * H / dh))
                panels.append(cv2.resize(c, (pw, H), interpolation=cv2.INTER_NEAREST))
                layout["panels"]["depth"] = (x0, pw, dw / pw, dh / H); x0 += pw
                if rgb is not None and rgb[0].shape[:2] == d.shape:          # 같은 크기면 겹쳐서 정렬 확인
                    ov = cv2.addWeighted(rgb[0], 0.5, colorize(d, rng), 0.5, 0)
                    panels.append(cv2.resize(ov, (pw, H)))
                    layout["panels"]["overlay"] = (x0, pw, dw / pw, dh / H); x0 += pw
            if panels:
                cv2.imshow(WIN, np.hstack(panels))
            if "uv" in click:
                u, v, name = click.pop("uv")
                if dep is not None and (name != "rgb" or rgb[0].shape[:2] == dep[0].shape):
                    print(f"[클릭] {name} ({u}, {v}) 깊이 {depth_at(dep[0], u, v) / 1000:.3f} m (3×3 중앙값)")
                else:
                    print(f"[클릭] {name} ({u}, {v}) — RGB 와 깊이 크기가 달라 깊이 픽셀 대응 없음")
            now = time.time()
            if a.save and a.save_every > 0 and now - t_save >= a.save_every and (rgb is not None or dep is not None):
                save_pair(a.save, idx, rgb, dep, rng); idx += 1; t_save = now
            if now - last_status > 5.0:
                s = []
                if rgb_rx:
                    s.append(f"RGB {rgb_rx.count / (now - t0):.1f} fps" + (f" ({rgb_rx.err})" if rgb_rx.err else ""))
                if dep_rx:
                    s.append(f"깊이 {dep_rx.count / (now - t0):.1f} fps" + (f" ({dep_rx.err})" if dep_rx.err else ""))
                print("[head_cam] " + " | ".join(s))
                last_status = now
            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s") and (rgb is not None or dep is not None):
                save_pair(a.save or "logs/headcam", idx, rgb, dep, rng); idx += 1
        report(rgb_rx, dep_rx, time.time() - t0)
    except KeyboardInterrupt:
        report(rgb_rx, dep_rx, time.time() - t0)
    finally:
        if rgb_rx:
            rgb_rx.stop = True
            rgb_rx.kill()
        if dep_rx:
            dep_rx.stop = True
        if gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
