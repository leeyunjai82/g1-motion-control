"""
head_cam.py — H2 머리 쌍안 카메라 수신 (공식 'Bilateral Data Stream Acquisition Interface', 2026-08-11 판)

  RGB  : PC1 이 RTP/H264 UDP 유니캐스트로 '수신 IP' 에 보냄 (기본 192.168.123.170 → http://192.168.123.161:9080/set?ip=…)
         5002 원본 쌍안 1920×1080 15 fps (왜곡 보정 전) / 5004 왼눈 544×448 10 fps / 5006 오른눈 544×448 10 fps
         → gst-launch-1.0 이 디코딩해 BGR 원시 프레임을 stdout 으로 넘김 (pip opencv 는 GStreamer 없음)
  깊이 : PC1(192.168.123.161) TCP 5000 — Y16 544×448 10 fps, mm (0·65535 = 무효)
         헤더 36 B <IQQIIII (magic 'Y16 ' 0x59313620, seq, 보낸 쪽 시각 us, w, h, format, data_size) + w·h·2 B
         (공식 deb unitree-dep-img 1.0.0 의 dep_img_client.c 와 같은 형식)
  로봇: 앱에서 video_hub 끄고 'Stereo patch PC1' 서비스 켬 (기본 자동 시작 아님).
  이 PC: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
쓰는 곳: utils/check_head_cam.py (수신 확인), common/head_track.py (얼굴·사람 따라 머리 돌리기)
"""
import re
import shutil
import socket
import struct
import subprocess
import threading
import time

import numpy as np

PC1 = "192.168.123.161"
RGB_PORTS = {"left": 5004, "right": 5006, "raw": 5002}
RTP_CAPS = "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
MAGIC = 0x59313620                       # 'Y16 '
HDR = struct.Struct("<IQQIIII")          # 36 B, packed
MAGIC_BYTES = struct.pack("<I", MAGIC)


def _pdeathsig_ok():
    """setpriv --pdeathsig (util-linux 2.33+) 가 있는지."""
    if not shutil.which("setpriv"):
        return False
    try:
        return "pdeathsig" in subprocess.run(["setpriv", "--help"], capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return False


_PDEATH = None


def gst_cmd(args):
    """gst-launch-1.0 명령줄. 가능하면 setpriv --pdeathsig 로 감싸 부모(이 파이썬)가 SIGKILL 로 죽어도 같이 죽게 —
    안 그러면 수신 gst 가 UDP 포트를 잡은 채 남아 다음 실행이 영상을 못 받음."""
    global _PDEATH
    if _PDEATH is None:
        _PDEATH = _pdeathsig_ok()
    return (["setpriv", "--pdeathsig", "KILL", "--"] if _PDEATH else []) + ["gst-launch-1.0"] + list(args)


def all_ips():
    """이 PC 의 IPv4 주소 전부 (127.x 제외) — 웹 보기 주소 안내용 (로봇 대역 말고 사무실 망으로도 열 수 있게)."""
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
    except (OSError, subprocess.SubprocessError):
        out = []
    return [ip for ip in out if ip.count(".") == 3 and not ip.startswith("127.")]


def web_urls(port, peer):
    """웹 보기 주소 안내 문자열 (모든 IP + SSH 터널)."""
    ips = all_ips() or [my_ip(peer)]
    lines = [f"http://{ip}:{port}/" for ip in ips]
    lines.append(f"(SSH 로만 들어올 수 있으면: 내 PC 에서 ssh -L {port}:localhost:{port} <계정>@<이 PC 주소> → http://localhost:{port}/)")
    return "\n    ".join(lines)


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
        cmd = gst_cmd(["-v", "udpsrc", f"port={self.port}", "buffer-size=2097152", "!", RTP_CAPS, "!",
                       "rtph264depay", "!", "avdec_h264", "!", "fakesink"])
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
        cmd = gst_cmd(["-q", "udpsrc", f"port={self.port}", "buffer-size=2097152", "!", RTP_CAPS, "!",
                       "rtph264depay", "!", "avdec_h264", "!", "videoconvert", "!", f"video/x-raw,format=BGR,width={w},height={h}",
                       "!", "fdsink", "fd=1", "sync=false"])
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
