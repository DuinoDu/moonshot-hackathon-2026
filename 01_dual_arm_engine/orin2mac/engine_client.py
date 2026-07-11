#!/usr/bin/env python3
"""机器人端引擎客户端：把 FEN 发给笔记本上的 engine_server.py，拿回 bestmove。

只依赖标准库，可直接在真机运行。笔记本先启动服务：

    python3 engine_server.py                 # 在本目录运行，监听 8790

机器人端命令行用法：

    export CHESS_ENGINE_URL=http://<laptop-ip>:8790
    python3 engine_client.py --ping
    python3 engine_client.py --fen "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1"
    python3 engine_client.py --fen "..." --moves "h2e2 h9g7" --movetime 2000

代码里用法（供 galbot_scripts/capture_and_recognize.py 等脚本调用，
需先把本目录加入 sys.path 或与调用脚本放在同级）：

    from engine_client import EngineClient
    client = EngineClient()                      # 读 CHESS_ENGINE_URL 环境变量
    result = client.best_move(fen, movetime=1500)
    print(result["bestmove"])                    # 如 "h2e2"；无着法时为 "(none)"
"""

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

DEFAULT_URL = os.environ.get("CHESS_ENGINE_URL", "http://127.0.0.1:8790")


class EngineError(RuntimeError):
    """引擎服务不可达或返回错误。"""


class EngineClient:
    def __init__(self, base_url=None, timeout=15.0):
        self.base_url = (base_url or DEFAULT_URL).rstrip("/")
        self.timeout = timeout

    def _request(self, path, payload=None, timeout=None):
        url = self.base_url + path
        data = None
        headers = {}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as resp:
                body = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            try:
                detail = json.loads(exc.read().decode("utf-8")).get("error", "")
            except Exception:
                detail = ""
            raise EngineError(f"engine server error ({exc.code}): {detail}") from exc
        except (urllib.error.URLError, OSError) as exc:
            raise EngineError(f"cannot reach engine server at {url}: {exc}") from exc
        if "error" in body:
            raise EngineError(body["error"])
        return body

    def ping(self):
        """返回服务状态字典；连不上时抛 EngineError。"""
        return self._request("/api/ping")

    def best_move(self, fen, moves=None, movetime=1000, depth=None):
        """请求最优着法。

        fen: 当前局面 FEN（含走子方）；moves: 可选，fen 之后续走的 UCI 着法列表；
        movetime: 思考毫秒数；depth: 给定则按深度搜索（优先于 movetime）。
        返回服务端响应字典，关键字段 bestmove / ponder / score / pv / elapsedMs。
        """
        payload = {"fen": fen, "movetime": movetime}
        if moves:
            payload["moves"] = list(moves)
        if depth:
            payload["depth"] = depth
        # 引擎思考时间之外再留网络余量
        timeout = max(self.timeout, movetime / 1000 + 10, 120 if depth else 0)
        return self._request("/api/bestmove", payload, timeout=timeout)


def main():
    parser = argparse.ArgumentParser(description="Query the laptop chess engine server")
    parser.add_argument("--server", default=DEFAULT_URL, help="engine server base URL")
    parser.add_argument("--ping", action="store_true", help="only check server status")
    parser.add_argument("--fen", help="position FEN (side to move included)")
    parser.add_argument("--moves", default="", help="UCI moves after the FEN, space separated")
    parser.add_argument("--movetime", type=int, default=1000, help="think time in ms")
    parser.add_argument("--depth", type=int, help="search by depth instead of movetime")
    args = parser.parse_args()

    client = EngineClient(args.server)
    try:
        if args.ping:
            print(json.dumps(client.ping(), ensure_ascii=False, indent=2))
            return
        if not args.fen:
            parser.error("--fen is required (or use --ping)")
        result = client.best_move(
            args.fen, moves=args.moves.split(), movetime=args.movetime, depth=args.depth
        )
        result.pop("raw", None)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except EngineError as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
