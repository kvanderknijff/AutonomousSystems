import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from planner_service import PlannerController

logger = logging.getLogger(__name__)

WEB_DIR = Path(__file__).resolve().parent / "web"


class ReuseHTTPServer(HTTPServer):
    allow_reuse_address = True


def make_handler(controller: "PlannerController"):
    class FormationHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/api/status":
                self._send_json(200, controller.get_status())
                return

            if self.path in ("/", "/index.html"):
                self._serve_file("index.html", "text/html; charset=utf-8")
                return

            self.send_error(404)

        def do_POST(self) -> None:
            if self.path == "/api/stop":
                controller.stop_all()
                self._send_json(200, {"ok": True})
                return

            if self.path != "/api/formation":
                self.send_error(404)
                return

            length = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._send_json(400, {"ok": False, "error": "Invalid JSON"})
                return

            name = str(body.get("formation", "")).strip().lower()
            center_x = body.get("center_x")
            center_y = body.get("center_y")
            spacing = body.get("spacing")
            auto_center = body.get("auto_center")
            if auto_center is not None:
                auto_center = bool(auto_center)

            try:
                result = controller.request_formation(
                    name,
                    float(center_x) if center_x is not None else None,
                    float(center_y) if center_y is not None else None,
                    float(spacing) if spacing is not None else None,
                    auto_center=auto_center,
                )
            except ValueError as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
                return

            self._send_json(200, {"ok": True, "formation": result})

        def _serve_file(self, filename: str, content_type: str) -> None:
            path = WEB_DIR / filename
            if not path.is_file():
                self.send_error(404)
                return
            data = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send_json(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt: str, *args) -> None:
            logger.info("%s - %s", self.address_string(), fmt % args)

    return FormationHandler


def run_web_server(controller: "PlannerController", host: str, port: int) -> None:
    handler = make_handler(controller)
    try:
        httpd = ReuseHTTPServer((host, port), handler)
    except OSError as exc:
        logger.error(
            "Web UI failed to bind %s:%s — %s. "
            "Another run.py may still be running; try: pkill -f 'python3 run.py' or ss -tlnp | grep %s",
            host,
            port,
            exc,
            port,
        )
        raise
    logger.info("Formation web UI at http://%s:%s", host, port)
    httpd.serve_forever()


def start_web_server(controller: "PlannerController", host: str, port: int) -> threading.Thread:
    thread = threading.Thread(
        target=run_web_server,
        args=(controller, host, port),
        daemon=False,
        name="formation-web-ui",
    )
    thread.start()
    return thread
