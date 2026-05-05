"""
streams_server.py
-----------------
실행: python streams_server.py
접속: http://<서버IP>:50003

구조:
  /                     -> 통합 대시보드 (위: 3D viewer, 아래: video|depth)
  /robot-only           -> 192.168.68.116:50001/robot-only 프록시 (HTML)
  /api/*                -> 192.168.68.116:50001/api/* 프록시 (urdf/mesh/SSE 등)
  /stream/video-feed    -> localhost:50000/video_feed 프록시 (MJPEG)
  /stream/depth-feed    -> localhost:50002/depth_feed 프록시 (MJPEG)
"""
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse, Response
import httpx
import uvicorn

# ===== 설정 =====
PUBLIC_PORT = 50003
ROBOT_VIEWER = "http://192.168.68.116:50001"   # 3D 뷰어 (HTML + /api/*)
VIDEO_FEED   = "http://localhost:50000/video_feed"
DEPTH_FEED   = "http://localhost:50002/depth_feed"
# ================

app = FastAPI()

HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>Robot Streams</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #111; color: #eee;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    min-height: 100vh; padding: 16px;
  }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 65vh 45vh;
    gap: 16px;
    max-width: 1400px;
    margin: 0 auto;
  }
  .panel {
    background: #1c1c1c; border: 1px solid #2a2a2a;
    border-radius: 8px; overflow: hidden;
    display: flex; flex-direction: column;
  }
  .panel.full { grid-column: 1 / -1; }
  .panel h2 {
    font-size: 14px; font-weight: 500;
    padding: 10px 14px; background: #232323;
    border-bottom: 1px solid #2a2a2a; color: #9cf;
    flex: 0 0 auto;
  }
  .panel iframe, .panel img {
    flex: 1 1 auto;
    width: 100%;
    border: 0;
    background: #000;
    display: block;
  }
  .panel img { object-fit: contain; }
</style>
</head>
<body>
  <div class="grid">
    <div class="panel full">
      <h2>Robot Only (3D Viewer)</h2>
      <iframe src="/robot-only"></iframe>
    </div>
    <div class="panel">
      <h2>Video Feed</h2>
      <img src="/stream/video-feed" alt="video_feed">
    </div>
    <div class="panel">
      <h2>Depth Feed</h2>
      <img src="/stream/depth-feed" alt="depth_feed">
    </div>
  </div>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index():
    return HTML


# ---------- 공용 프록시 헬퍼 ----------
async def _stream_proxy(upstream_url: str):
    """스트림(MJPEG/SSE) 또는 일반 응답을 그대로 흘려보냄."""
    print(f"[proxy] -> {upstream_url}")
    client = httpx.AsyncClient(timeout=None)
    try:
        req = client.build_request("GET", upstream_url)
        resp = await client.send(req, stream=True)
    except Exception as e:
        await client.aclose()
        print(f"[proxy] connect FAIL: {e!r}")
        return Response(content=str(e), status_code=502)

    ctype = resp.headers.get("content-type", "application/octet-stream")
    print(f"[proxy]    status={resp.status_code} ctype={ctype}")

    async def iterator():
        try:
            async for chunk in resp.aiter_raw():
                yield chunk
        except Exception as e:
            print(f"[proxy] stream error: {e!r}")
        finally:
            await resp.aclose()
            await client.aclose()

    return StreamingResponse(
        iterator(),
        status_code=resp.status_code,
        media_type=ctype,
    )


# ---------- 3D 뷰어 (HTML + /api/*) ----------
@app.get("/robot-only")
async def robot_only():
    return await _stream_proxy(f"{ROBOT_VIEWER}/robot-only")


@app.get("/api/{path:path}")
async def robot_api(path: str, request: Request):
    qs = request.url.query
    url = f"{ROBOT_VIEWER}/api/{path}" + (f"?{qs}" if qs else "")
    return await _stream_proxy(url)


# ---------- MJPEG 스트림 ----------
@app.get("/stream/video-feed")
async def video_feed():
    return await _stream_proxy(VIDEO_FEED)


@app.get("/stream/depth-feed")
async def depth_feed():
    return await _stream_proxy(DEPTH_FEED)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PUBLIC_PORT)
