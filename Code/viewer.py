from __future__ import annotations

import io
import threading
from typing import Optional

from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, JSONResponse
import uvicorn

try:
    import numpy as np
except ImportError:
    np = None


"""File pass:MainWorkspace/browser_viewer/viewer.py"""

"""
viewer.py
Raspberry Pi 用の軽量画像ビューア (FastAPI)

pip install fastapi uvicorn

使い方 (ラズパイ側):
    import viewer
    v = viewer.Viewer()
    v.start()
    ...
    v.update("cam0", jpeg_bytes)   # or PNG bytes / np.ndarray(already-encoded) 等
    ...
    v.stop()

Windows 側からは http://<ラズパイのIP>:8000/ をブラウザで開くだけ。
"""

# 画像フォーマット判定用マジックバイト
_MAGIC = {
    b"\x89PNG": "image/png",
    b"\xff\xd8": "image/jpeg",
}

# ブラウザ / 中間キャッシュに画像をキャッシュさせない
_NO_CACHE = {
    "Cache-Control": "no-store, no-cache, must-revalidate",
    "Pragma": "no-cache",
}


class Viewer:

    def __init__(self, host: str = "0.0.0.0", port: int = 8000, refresh_ms: int = 200):
        self.host = host
        self.port = port
        self.refresh_ms = refresh_ms

        # images/versions は uvicorn のワーカースレッドと呼び出し側スレッドの
        # 両方からアクセスされるため lock で保護する
        self._lock = threading.Lock()
        self._images: dict[str, tuple[bytes, str]] = {}
        self._versions: dict[str, int] = {}

        self.app = FastAPI()
        self.app.add_api_route("/", self.index)
        self.app.add_api_route("/image/{name}", self.image)
        self.app.add_api_route("/versions", self.versions)

        self._server: Optional[uvicorn.Server] = None
        self._thread: Optional[threading.Thread] = None

    # --------------------------------------------------

    def start(self) -> None:
        config = uvicorn.Config(
            self.app, host=self.host, port=self.port, log_level="warning"
        )
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        if self._server:
            self._server.should_exit = True
        if self._thread:
            self._thread.join(timeout=timeout)

    def clear(self) -> None:
        with self._lock:
            self._images.clear()
            self._versions.clear()

    # --------------------------------------------------

    def update(self, name: str, image) -> None:
        if np is not None and isinstance(image, np.ndarray):
            image = image.tobytes()
        elif isinstance(image, (memoryview, bytearray)):
            image = bytes(image)
        elif isinstance(image, io.BytesIO):
            image = image.getvalue()

        if not isinstance(image, bytes):
            raise TypeError("Unsupported image type")

        mime = next((m for sig, m in _MAGIC.items() if image.startswith(sig)), None)
        if mime is None:
            raise ValueError("Only PNG/JPEG supported.")

        with self._lock:
            self._images[name] = (image, mime)
            self._versions[name] = self._versions.get(name, 0) + 1

    # --------------------------------------------------

    def image(self, name: str):
        with self._lock:
            entry = self._images.get(name)

        if entry is None:
            return Response(status_code=404)

        img, mime = entry
        return Response(content=img, media_type=mime, headers=_NO_CACHE)

    def versions(self):
        with self._lock:
            return JSONResponse(dict(self._versions))

    def index(self):
        with self._lock:
            names = list(self._images.keys())

        cards = "\n".join(
            f'<div class="card"><h2>{n}</h2><img id="img-{n}" src="/image/{n}"></div>'
            for n in names
        )

        html = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Viewer</title>
<style>
 body {{ font-family: sans-serif; margin: 30px; background: #f0f0f0; }}
 .card {{ margin-bottom: 30px; }}
 img {{ max-width: 100%; border: 1px solid gray; }}
</style>
</head>
<body>
<h1>Viewer</h1>
{cards}
<script>
let known = {{}};

async function refresh() {{
    try {{
        const res = await fetch("/versions");
        const versions = await res.json();
        for (const [name, v] of Object.entries(versions)) {{
            if (known[name] !== v) {{
                known[name] = v;
                const el = document.getElementById("img-" + name);
                if (el) el.src = "/image/" + name + "?v=" + v;
            }}
        }}
    }} catch (e) {{ /* サーバ再起動中などは無視 */ }}
}}

setInterval(refresh, {self.refresh_ms});
</script>
</body>
</html>"""
        return HTMLResponse(html)