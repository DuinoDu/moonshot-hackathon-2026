#!/usr/bin/env python3
"""笔记本端象棋引擎服务：把本机 Pikafish 包成 HTTP 接口，供机器人查询走法。

在笔记本上运行（引擎常驻，不用每次请求重启加载 NNUE）：

    python3 engine_server.py                          # 默认 0.0.0.0:8790
    PIKAFISH_BIN=/path/to/pikafish python3 engine_server.py
    CHESS_ENGINE_PORT=9000 python3 engine_server.py

接口（机器人端配套客户端见同目录 engine_client.py）：

    GET  /api/ping
        -> {"ok": true, "engine": "...", "engineAlive": true, "startFen": "..."}

    POST /api/bestmove
        请求: {"fen": "...", "moves": ["h2e2", ...], "movetime": 1000, "depth": null}
              fen 缺省为起始局面；moves 可选，是在 fen 之后续走的 UCI 着法；
              movetime 单位毫秒（50~30000）；给了 depth 则用 go depth 代替 movetime。
        响应: {"bestmove": "h2e2", "ponder": "h9g7", "score": {"type": "cp", "value": 31},
               "pv": ["h2e2", ...], "elapsedMs": 1003}
        引擎认输/无着法时 bestmove 为 "(none)"。
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
import os
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
# moonbot/moonshot-hackathon-2026/01_dual_arm_engine/orin2mac/ -> moonbot/Pikafish/src/pikafish
DEFAULT_ENGINE = ROOT.parents[2] / "Pikafish" / "src" / "pikafish"
START_FEN = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1"


def engine_path():
    return Path(os.environ.get("PIKAFISH_BIN", DEFAULT_ENGINE)).expanduser().resolve()


class Engine:
    """常驻 Pikafish 进程，串行处理请求（UCI 协议本身不支持并发 go）。"""

    def __init__(self):
        self.proc = None
        self.lock = threading.Lock()

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _read_until(self, prefixes, timeout):
        lines = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.proc.stdout.readline()
            if line == "":
                if self.proc.poll() is not None:
                    raise RuntimeError("Pikafish process exited unexpectedly")
                time.sleep(0.02)
                continue
            line = line.rstrip("\n")
            lines.append(line)
            if any(line.startswith(prefix) for prefix in prefixes):
                return line, lines
        raise TimeoutError("Pikafish did not answer in time")

    def _send(self, command):
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()

    def _start(self):
        binary = engine_path()
        if not binary.exists():
            raise FileNotFoundError(f"Pikafish binary not found: {binary}")
        if not os.access(binary, os.X_OK):
            raise PermissionError(f"Pikafish binary is not executable: {binary}")

        self.proc = subprocess.Popen(
            [str(binary)],
            cwd=str(binary.parent),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._read_until(["Pikafish", "Stockfish"], timeout=5)
        self._send("uci")
        self._read_until(["uciok"], timeout=10)
        self._send("isready")
        self._read_until(["readyok"], timeout=10)

    def _ensure(self):
        if not self.alive():
            if self.proc is not None:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self._start()

    def stop(self):
        with self.lock:
            if self.alive():
                try:
                    self._send("quit")
                except Exception:
                    pass
                self.proc.kill()
            self.proc = None

    def bestmove(self, fen, moves, movetime, depth):
        with self.lock:
            self._ensure()

            position = f"position fen {fen}"
            if moves:
                position += " moves " + " ".join(moves)

            started = time.monotonic()
            self._send("ucinewgame")
            self._send("isready")
            self._read_until(["readyok"], timeout=10)
            self._send(position)
            if depth:
                self._send(f"go depth {depth}")
                timeout = 120
            else:
                self._send(f"go movetime {movetime}")
                timeout = max(10, movetime / 1000 + 8)
            best, lines = self._read_until(["bestmove"], timeout=timeout)
            elapsed_ms = int((time.monotonic() - started) * 1000)

            parts = best.split()
            result = {
                "bestmove": parts[1] if len(parts) > 1 else "",
                "ponder": parts[3] if len(parts) > 3 and parts[2] == "ponder" else None,
                "score": None,
                "pv": [],
                "elapsedMs": elapsed_ms,
                "raw": lines[-40:],
            }

            for line in reversed(lines):
                if line.startswith("info") and " score " in line:
                    tokens = line.split()
                    idx = tokens.index("score")
                    result["score"] = {"type": tokens[idx + 1], "value": int(tokens[idx + 2])}
                    if "pv" in tokens:
                        result["pv"] = tokens[tokens.index("pv") + 1:]
                    break

            return result


ENGINE = Engine()


class Handler(BaseHTTPRequestHandler):
    def send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        sys.stderr.write("[engine_server] %s - %s\n" % (self.client_address[0], fmt % args))

    def do_GET(self):
        if self.path == "/":
            self.send_json(
                200,
                {
                    "service": "moonbot chess engine server",
                    "endpoints": {
                        "GET /api/ping": "健康检查",
                        "POST /api/bestmove": '{"fen": "...", "moves": [...], "movetime": 1000}',
                    },
                },
            )
            return
        if self.path in ("/api/ping", "/api/status"):
            binary = engine_path()
            self.send_json(
                200,
                {
                    "ok": binary.exists() and os.access(binary, os.X_OK),
                    "engine": str(binary),
                    "engineAlive": ENGINE.alive(),
                    "startFen": START_FEN,
                },
            )
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/api/bestmove":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length) or b"{}")

            fen = str(payload.get("fen") or START_FEN).strip()
            moves = payload.get("moves") or []
            if isinstance(moves, str):
                moves = moves.split()
            movetime = max(50, min(int(payload.get("movetime", 1000)), 30000))
            depth = payload.get("depth")
            depth = max(1, min(int(depth), 60)) if depth else None

            self.send_json(200, ENGINE.bestmove(fen, moves, movetime, depth))
        except Exception as exc:
            self.send_json(500, {"error": str(exc), "engine": str(engine_path())})


def main():
    port = int(os.environ.get("CHESS_ENGINE_PORT", "8790"))
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"Chess engine server: http://0.0.0.0:{port}  (robot uses http://<laptop-ip>:{port})")
    print(f"Engine: {engine_path()}")
    try:
        server.serve_forever()
    finally:
        ENGINE.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
