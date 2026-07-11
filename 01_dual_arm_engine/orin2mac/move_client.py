#!/usr/bin/env python3
"""Mac 端走法发送器：把引擎输出的 UCI 走法（如 c2b2）换算成机械臂物理坐标，
发给 Orin 上已验证的 whd_dual_arm_live.py `/api/board_move` 执行抓放。

坐标换算（关键）：
    引擎 UCI 格：列 a-i（9 列）× 行 0-9（10 行），红方底线是 rank 0
    机械臂示教点：列 a-j（10 列）× 行 1-9（9 行），棋盘相对机械臂横放（转了 90°）
    ——UCI 的“行”对应物理的“列”，UCI 的“列”对应物理的“行”。
    具体转向由 --orient 0~3 决定（默认 0），首次使用请先校准：

        # 让机械臂悬停到 UCI e0（红帅位）换算出的物理点上空，看落点对不对：
        python3 move_client.py --goto e0 --orient 0
        # 不对就换 --orient 1/2/3 重试；确定后写进环境变量一劳永逸：
        export MOVE_ORIENT=2

日常用法（Mac 上）：

    export ROBOT_ARM_URL=http://192.168.1.88:8088
    python3 move_client.py --move c2b2                 # 执行一步棋
    python3 move_client.py --move c2b2 --dry-run       # 只看换算结果不下发
    python3 move_client.py --move h7e7 --capture --discard j9
                                                       # 吃子：先把 e7 上的子移到 j9 弃子区

代码里用法：

    from move_client import MoveSender
    sender = MoveSender()                  # 读 ROBOT_ARM_URL / MOVE_ORIENT 环境变量
    sender.send("c2b2")                    # 阻塞到机械臂完成，失败抛 MoveError
"""

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

DEFAULT_ARM_URL = os.environ.get("ROBOT_ARM_URL", "http://192.168.1.88:8088")
DEFAULT_ORIENT = int(os.environ.get("MOVE_ORIENT", "0"))

UCI_FILES = "abcdefghi"   # 9 列
PHYS_COLS = "abcdefghij"  # 10 列（对应 UCI 的 10 个 rank）


class MoveError(RuntimeError):
    """走法非法、换算失败或机械臂执行失败。"""


def parse_uci_square(sq):
    sq = sq.strip().lower()
    if len(sq) != 2 or sq[0] not in UCI_FILES or not sq[1].isdigit():
        raise MoveError(f"无效 UCI 格 {sq!r}，应为 a0..i9")
    return UCI_FILES.index(sq[0]), int(sq[1])  # (file 0-8, rank 0-9)


def uci_to_phys(sq, orient):
    """UCI 格 -> 机械臂示教点名（a1..j9）。orient 0~3 对应 4 种棋盘朝向。"""
    f, r = parse_uci_square(sq)
    if orient == 0:
        c, w = r, f + 1
    elif orient == 1:
        c, w = 9 - r, f + 1
    elif orient == 2:
        c, w = r, 9 - f
    elif orient == 3:
        c, w = 9 - r, 9 - f
    else:
        raise MoveError("orient 必须是 0/1/2/3")
    return f"{PHYS_COLS[c]}{w}"


def split_uci_move(move):
    move = move.strip().lower()
    if len(move) != 4:
        raise MoveError(f"无效 UCI 走法 {move!r}，应为如 c2b2 的 4 字符")
    return move[:2], move[2:]


class MoveSender:
    def __init__(self, base_url=None, orient=None, timeout=120.0):
        self.base_url = (base_url or DEFAULT_ARM_URL).rstrip("/")
        self.orient = DEFAULT_ORIENT if orient is None else int(orient)
        self.timeout = timeout

    def _post(self, path, payload):
        url = self.base_url + path
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError) as exc:
            raise MoveError(f"无法连接机械臂服务 {url}: {exc}") from exc
        if not body.get("ok"):
            raise MoveError(f"机械臂执行失败: {json.dumps(body, ensure_ascii=False)}")
        return body

    def convert(self, move):
        """UCI 走法 -> (物理 src, 物理 dst)，不下发。"""
        src, dst = split_uci_move(move)
        return uci_to_phys(src, self.orient), uci_to_phys(dst, self.orient)

    def send(self, move, capture=False, discard=None, move_timeout=60):
        """执行一步棋。capture=True 时先把落点上的子移到 discard 弃子点。

        返回 {"move": ..., "src": ..., "dst": ..., "capture": ..., "results": [...]}
        """
        src, dst = self.convert(move)
        results = []
        if capture:
            if not discard:
                raise MoveError("吃子必须指定弃子点 discard（示教点名，如 j9）")
            results.append(self._post(
                "/api/board_move", {"src": dst, "dst": discard, "timeout": move_timeout}))
        results.append(self._post(
            "/api/board_move", {"src": src, "dst": dst, "timeout": move_timeout}))
        return {"move": move, "src": src, "dst": dst,
                "capture": discard if capture else None, "results": results}

    def goto(self, uci_square, hover_only=True, move_timeout=60):
        """悬停到某 UCI 格上空（用于校准 orient），返回机械臂响应。"""
        point = uci_to_phys(uci_square, self.orient)
        body = self._post("/api/board_goto",
                          {"point": point, "hover_only": hover_only, "timeout": move_timeout})
        body["phys_point"] = point
        return body


def main():
    parser = argparse.ArgumentParser(description="Send engine moves to the robot arm")
    parser.add_argument("--robot", default=DEFAULT_ARM_URL, help="robot arm server URL")
    parser.add_argument("--orient", type=int, default=DEFAULT_ORIENT, choices=[0, 1, 2, 3],
                        help="board orientation mapping (calibrate with --goto)")
    parser.add_argument("--move", help="UCI move to execute, e.g. c2b2")
    parser.add_argument("--capture", action="store_true", help="destination has an enemy piece")
    parser.add_argument("--discard", help="physical point for captured pieces, e.g. j9")
    parser.add_argument("--goto", metavar="SQ", help="hover above a UCI square to calibrate orient")
    parser.add_argument("--dry-run", action="store_true", help="print conversion only, do not send")
    parser.add_argument("--timeout", type=int, default=60, help="arm motion timeout seconds")
    args = parser.parse_args()

    sender = MoveSender(args.robot, args.orient)
    try:
        if args.goto:
            if args.dry_run:
                print(f"{args.goto} -> {uci_to_phys(args.goto, args.orient)} (orient={args.orient})")
                return
            print(json.dumps(sender.goto(args.goto, move_timeout=args.timeout),
                             ensure_ascii=False, indent=2))
            return
        if not args.move:
            parser.error("--move or --goto is required")
        src, dst = sender.convert(args.move)
        print(f"UCI {args.move} -> 物理 {src} → {dst} (orient={args.orient})")
        if args.dry_run:
            return
        result = sender.send(args.move, capture=args.capture,
                             discard=args.discard, move_timeout=args.timeout)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except MoveError as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
