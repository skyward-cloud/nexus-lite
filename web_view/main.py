#!/usr/bin/env python3
"""px4_bridge web 操作面板：pywebview + Three.js。

订阅 /px4_bridge/out/status 与点云话题，页面按钮发布 /px4_bridge/in/task_cmd。

用法:
  source install/setup.bash
  pip install -r web_view/requirements.txt
  python3 web_view/main.py
  python3 web_view/main.py --cloud-topic /lidar/points_world
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

import webview

from ros_bridge import RosBridge

FRONTEND_DIR = Path(__file__).resolve().parent / "frontend"


class Api:
    """供前端 window.pywebview.api 调用。"""

    def __init__(self) -> None:
        self._bridge: Optional[RosBridge] = None
        self._window: Any = None
        self._status_lock = threading.Lock()
        self._pending_status: Optional[dict[str, Any]] = None
        self._pending_cloud: Optional[tuple[list[float], int]] = None
        self._cloud_lock = threading.Lock()

    def bind(self, bridge: RosBridge, window: Any) -> None:
        self._bridge = bridge
        self._window = window

    def ready(self) -> dict[str, str]:
        return {"ok": "1"}

    def submit_task(
        self,
        task_type: str,
        payload: Any = None,
        deadline_ms: int = 0,
        task_id: str = "",
    ) -> dict[str, Any]:
        if self._bridge is None:
            return {"ok": False, "error": "bridge 未就绪"}
        if payload is None:
            payload = {}
        if isinstance(payload, str):
            payload = json.loads(payload) if payload else {}
        try:
            tid = self._bridge.submit_task(
                str(task_type),
                dict(payload),
                deadline_ms=int(deadline_ms or 0),
                task_id=str(task_id) if task_id else None,
            )
            return {"ok": True, "task_id": tid}
        except Exception as exc:  # pylint: disable=broad-except
            return {"ok": False, "error": str(exc)}

    def on_status(self, status: dict[str, Any]) -> None:
        with self._status_lock:
            self._pending_status = status

    def on_cloud(self, flat_xyz: list[float], count: int) -> None:
        with self._cloud_lock:
            self._pending_cloud = (flat_xyz, count)

    def poll(self) -> dict[str, Any]:
        """前端轮询：取最新状态与点云。"""
        out: dict[str, Any] = {}
        with self._status_lock:
            if self._pending_status is not None:
                out["status"] = self._pending_status
                self._pending_status = None
        with self._cloud_lock:
            if self._pending_cloud is not None:
                flat, count = self._pending_cloud
                out["cloud"] = {"xyz": flat, "count": count}
                self._pending_cloud = None
        return out


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        return


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_frontend_server() -> tuple[ThreadingHTTPServer, str]:
    port = _free_port()
    handler = partial(_QuietHandler, directory=str(FRONTEND_DIR))
    server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    thread = threading.Thread(target=server.serve_forever, name="web-static", daemon=True)
    thread.start()
    return server, f"http://127.0.0.1:{port}/index.html"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="px4_bridge web 操作面板")
    p.add_argument(
        "--cloud-topic",
        default="/lidar/points_world",
        help="点云话题 (sensor_msgs/PointCloud2)",
    )
    p.add_argument("--width", type=int, default=1400)
    p.add_argument("--height", type=int, default=900)
    p.add_argument("--debug", action="store_true", help="打开开发者工具")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not (FRONTEND_DIR / "index.html").is_file():
        print(f"缺少前端文件: {FRONTEND_DIR / 'index.html'}", file=sys.stderr)
        return 1

    api = Api()
    bridge = RosBridge(
        status_cb=api.on_status,
        cloud_cb=api.on_cloud,
        cloud_topic=str(args.cloud_topic),
    )
    try:
        bridge.start()
    except Exception as exc:  # pylint: disable=broad-except
        print(f"启动 ROS 失败: {exc}", file=sys.stderr)
        return 1

    httpd, url = start_frontend_server()
    window = webview.create_window(
        title="nexus-lite · PX4 Bridge",
        url=url,
        js_api=api,
        width=int(args.width),
        height=int(args.height),
        background_color="#0e1418",
    )
    api.bind(bridge, window)

    def _on_closed() -> None:
        bridge.stop()
        httpd.shutdown()

    window.events.closed += _on_closed
    try:
        webview.start(debug=bool(args.debug))
    finally:
        bridge.stop()
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
