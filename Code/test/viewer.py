from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass

import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, JSONResponse

NO_CACHE = {
    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    "Pragma": "no-cache",
}


@dataclass(slots=True)
class RuntimeStatus:
    state: str = "starting"
    message: str = "Waiting for LiDAR"
    packets: int = 0
    points: int = 0
    crc_errors: int = 0
    serial_reconnects: int = 0
    speed_dps: int = 0
    last_packet_monotonic: float = 0.0


class Viewer:
    def __init__(self, host: str, port: int, refresh_ms: int = 100) -> None:
        self.host = host
        self.port = port
        self.refresh_ms = max(50, refresh_ms)
        self._lock = threading.Lock()
        self._images: dict[str, bytes] = {}
        self._versions: dict[str, int] = {}
        self._status = RuntimeStatus()

        self.app = FastAPI(title="MS200 LiDAR Viewer", docs_url=None, redoc_url=None)
        self.app.add_api_route("/", self.index, response_class=HTMLResponse)
        self.app.add_api_route("/image/{name}", self.image)
        self.app.add_api_route("/api/state", self.state)
        self.app.add_api_route("/healthz", self.healthz)
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None

    def start(self, timeout: float = 5.0) -> None:
        if self._thread and self._thread.is_alive():
            return
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="warning",
            access_log=False,
        )
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, name="web-viewer", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + timeout
        while not self._server.started and self._thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not self._server.started:
            raise RuntimeError(f"web server did not start on {self.host}:{self.port}")

    def stop(self, timeout: float = 5.0) -> None:
        if self._server:
            self._server.should_exit = True
        if self._thread:
            self._thread.join(timeout)

    def update_jpeg(self, name: str, jpeg: bytes) -> None:
        if not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("update_jpeg requires JPEG encoded bytes")
        with self._lock:
            self._images[name] = jpeg
            self._versions[name] = self._versions.get(name, 0) + 1

    def update_status(self, **changes: object) -> None:
        with self._lock:
            for key, value in changes.items():
                if not hasattr(self._status, key):
                    raise KeyError(key)
                setattr(self._status, key, value)

    def _snapshot(self) -> dict[str, object]:
        with self._lock:
            status = asdict(self._status)
            versions = dict(self._versions)
        last = float(status.pop("last_packet_monotonic"))
        status["last_packet_age_s"] = round(time.monotonic() - last, 2) if last else None
        status["versions"] = versions
        return status

    def image(self, name: str) -> Response:
        with self._lock:
            data = self._images.get(name)
        if data is None:
            return Response(status_code=404)
        return Response(data, media_type="image/jpeg", headers=NO_CACHE)

    def state(self) -> JSONResponse:
        return JSONResponse(self._snapshot(), headers=NO_CACHE)

    def healthz(self) -> JSONResponse:
        snapshot = self._snapshot()
        healthy = snapshot["state"] == "streaming"
        return JSONResponse(snapshot, status_code=200 if healthy else 503, headers=NO_CACHE)

    def index(self) -> HTMLResponse:
        refresh_ms = int(self.refresh_ms)
        page = f"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MS200 LiDAR</title>
<style>
:root {{ color-scheme: dark; font-family: system-ui, sans-serif; }}
body {{ margin:0; background:#0b1118; color:#e8eef5; }}
header {{ padding:14px 20px; background:#131e29; display:flex; gap:20px; align-items:center; flex-wrap:wrap; }}
h1 {{ font-size:20px; margin:0; }} #status {{ color:#ffcf67; }}
main {{ padding:16px; max-width:900px; margin:auto; }}
.card {{ background:#131e29; border:1px solid #263849; border-radius:10px; padding:12px; }}
img {{ display:block; width:100%; height:auto; background:#05080b; }}
#details {{ margin-top:10px; color:#9eb1c5; font:14px ui-monospace,monospace; }}
</style></head><body><header><h1>MS200 LiDAR Viewer</h1><span id="status">接続中…</span></header>
<main><div class="card"><img id="scan" alt="LiDAR scan"><div id="details"></div></div></main>
<script>
let known = -1;
async function refresh() {{
  const status = document.getElementById('status');
  try {{
    const response = await fetch('/api/state?t=' + Date.now(), {{cache:'no-store'}});
    const data = await response.json();
    status.textContent = data.state + ' — ' + data.message;
    status.style.color = data.state === 'streaming' ? '#65e59b' : '#ffcf67';
    const version = (data.versions || {{}}).MS200;
    if (version !== undefined && version !== known) {{
      known = version;
      document.getElementById('scan').src = '/image/MS200?v=' + version;
    }}
    document.getElementById('details').textContent =
      `packets ${{data.packets}} | points ${{data.points}} | speed ${{data.speed_dps}}°/s | CRC errors ${{data.crc_errors}} | age ${{data.last_packet_age_s ?? '-'}}s`;
  }} catch (_) {{ status.textContent='Webサーバーとの接続が切れました'; status.style.color='#ff6b6b'; }}
}}
refresh(); setInterval(refresh, {refresh_ms});
</script></body></html>"""
        return HTMLResponse(page, headers=NO_CACHE)
