# 01_dual_arm_engine — 双臂控制 + 引擎桥接

真机验证过的机械臂执行链路，共 4 个文件，全部在实际使用中：

```
Mac（笔记本）                          Orin（机器人，192.168.1.88）
├─ orin2mac/engine_server.py  ◄──────  orin2mac/engine_client.py
│  Pikafish HTTP 服务，端口 8790        真机上查询 bestmove（部署在 ~/Desktop/orin2mac/）
│
└─ orin2mac/move_client.py    ──────►  whd_dual_arm_live.py
   UCI 走法 → 物理示教点，下发抓放       双臂 Web 控制面板，端口 8088
                                       （/api/board_move、/api/board_goto）
```

## 文件说明

- **`whd_dual_arm_live.py`** — 在 Orin 真机上跑。双臂 Web 控制面板 + 三路相机 MJPEG 流，
  端口 8088。控制下发路径（`/api/move`、`/api/pose`、`/api/board_move` 等）已在真机验证，勿改。

  ```bash
  source ~/Desktop/GalbotSDK/galbot_sdk/linux-aarch64-gcc940/setup.sh
  python3 whd_dual_arm_live.py     # 浏览器打开 http://<robot-ip>:8088，先点「初始化 SDK」
  ```

- **`orin2mac/engine_server.py`** — 在 Mac 上跑。把本机 Pikafish 包成 HTTP 接口（默认 8790），
  默认引擎路径指向仓库外层的 `moonbot/Pikafish/src/pikafish`，可用 `PIKAFISH_BIN` 覆盖。

  ```bash
  python3 orin2mac/engine_server.py    # 机器人用 http://192.168.1.188:8790
  ```

- **`orin2mac/engine_client.py`** — 在 Orin 真机上跑（已部署到 `~/Desktop/orin2mac/`）。
  只依赖标准库，把 FEN 发给 Mac 的 engine_server 拿回 bestmove。

  ```bash
  export CHESS_ENGINE_URL=http://192.168.1.188:8790
  python3 engine_client.py --ping
  python3 engine_client.py --fen "..." --movetime 2000
  ```

- **`orin2mac/move_client.py`** — 在 Mac 上跑。把引擎输出的 UCI 走法（如 `c2b2`）换算成
  机械臂示教点（棋盘相对机械臂转了 90°，朝向由 `--orient 0~3` / `MOVE_ORIENT` 决定），
  发给 whd_dual_arm_live 的 `/api/board_move` 执行抓放。

  ```bash
  export ROBOT_ARM_URL=http://192.168.1.88:8088
  python3 orin2mac/move_client.py --goto e0 --orient 0   # 首次先校准朝向
  python3 orin2mac/move_client.py --move c2b2            # 执行一步棋
  python3 orin2mac/move_client.py --move h7e7 --capture --discard j9   # 吃子
  ```
