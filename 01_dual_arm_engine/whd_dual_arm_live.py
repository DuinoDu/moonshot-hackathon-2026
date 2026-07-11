#!/usr/bin/env python3
"""
Galbot G1 双臂 Web 控制 —— 增强版（三路相机实时 MJPEG 同屏）

在你原有 whd_dual_arm_web_control.py 基础上增加：
  * 头部 / 左手 / 右手 三路相机「实时视频流」同屏（原来是点按钮抓单帧）
  * /stream?camera=xxx  MJPEG 多帧流端点 + 后台采集线程
  * 方向键「按住持续动、松手立即停」，默认走高频流式控制：服务端 30Hz 循环
    IK + set_joint_commands 匀速推进末端（/api/jog，TTL 0.5s 断线自停），
    松手 /api/jog_stop 停发即停；「兼容模式」可切回旧版规划式点动（/api/move
    低延迟连发 + /api/hold_stop 急停）
  * 「棋盘标定与抓放」区块：9×10 可点击网格，jog 到位后一键标记当前点；
    标满 3 点后可自动飞到任意点的插值位置；支持 A→B 整套抓放。
    存储/插值复用 galbot_scripts/board_teach.py，数据存 board_points.json
控制逻辑完全沿用你验证过、能真正驱动机器人的实现（motion.set_end_effector_pose +
robot.set_joint_positions 阻塞下发），未改动。

运行：
  cd ~/Desktop/WHD           # 或任意目录
  source ~/Desktop/GalbotSDK/galbot_sdk/linux-aarch64-gcc940/setup.sh
  python3 whd_dual_arm_live.py
  浏览器打开 http://<robot-ip>:8088
  先点「初始化 SDK」，视频会在初始化后开始播放。
"""

import json
import os
import platform
import base64
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs


SDK_ROOT = os.environ.get("GALBOT_SDK_ROOT", "/home/galbot/Desktop/GalbotSDK")
PORT = int(os.environ.get("WHD_ARM_PORT", "8088"))
BODY_MAX_DELTA = 0.35
STREAM_CAMERAS = ["head_left", "left_arm", "right_arm"]
CAPTURE_FPS = 15

machine = platform.machine().lower()
if machine in ("aarch64", "arm64"):
    sdk_python_rels = ("galbot_sdk/linux-aarch64-gcc940/lib/python",)
elif machine in ("x86_64", "amd64"):
    sdk_python_rels = ("galbot_sdk/linux-x86_64-gcc940/lib/python",)
else:
    sdk_python_rels = (
        "galbot_sdk/linux-aarch64-gcc940/lib/python",
        "galbot_sdk/linux-x86_64-gcc940/lib/python",
    )

for rel in sdk_python_rels:
    path = os.path.join(SDK_ROOT, rel)
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)

try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1 import ControlStatus, G1JointGroup, GalbotMotion, GalbotRobot, SensorType
except Exception as exc:
    gm = None
    ControlStatus = None
    G1JointGroup = None
    GalbotMotion = None
    GalbotRobot = None
    SensorType = None
    SDK_IMPORT_ERROR = repr(exc)
else:
    SDK_IMPORT_ERROR = ""

# 棋盘标定存储/插值复用 galbot_scripts/board_teach.py
GALBOT_SCRIPTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "galbot_scripts")
if os.path.isdir(GALBOT_SCRIPTS_DIR) and GALBOT_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, GALBOT_SCRIPTS_DIR)
try:
    import board_teach as bt
    BT_IMPORT_ERROR = ""
except Exception as exc:
    bt = None
    BT_IMPORT_ERROR = repr(exc)


# --------------------------------------------------------------------------- #
# 实时视频：只保留最新帧的槽位
# --------------------------------------------------------------------------- #
class FrameSlot:
    def __init__(self):
        self.jpeg = None
        self.seq = 0
        self.cond = threading.Condition()

    def set(self, jpeg_bytes):
        with self.cond:
            self.jpeg = jpeg_bytes
            self.seq += 1
            self.cond.notify_all()

    def wait_newer(self, last_seq, timeout=2.0):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            return self.jpeg, self.seq


CAMERA_FRAMES = {name: FrameSlot() for name in STREAM_CAMERAS}


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>WHD 双臂末端控制（实时视觉）</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f6f7f9; --panel: #ffffff; --line: #d9dee7;
      --text: #161a22; --muted: #5f6877; --accent: #1668dc; --danger: #c92a2a;
    }
    * { box-sizing: border-box; }
    body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
           background: var(--bg); color: var(--text); }
    header { padding: 18px 22px; border-bottom: 1px solid var(--line); background: var(--panel);
             display: flex; align-items: center; justify-content: space-between; gap: 16px; }
    h1 { font-size: 20px; margin: 0; }
    main { max-width: 1180px; margin: 0 auto; padding: 22px;
           display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    section { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 16px; }
    .wide { grid-column: 1 / -1; }
    h2 { font-size: 16px; margin: 0 0 12px; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    button { appearance: none; border: 1px solid var(--line); background: #fff; min-height: 44px;
             border-radius: 7px; padding: 8px 10px; font-size: 14px; cursor: pointer; }
    button:hover { border-color: var(--accent); color: var(--accent); }
    button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
    button.danger { border-color: #f0b5b5; color: var(--danger); }
    .check { display: flex; align-items: center; gap: 8px; min-height: 40px; }
    .check input { width: auto; }
    label { display: grid; gap: 6px; color: var(--muted); font-size: 13px; }
    input { width: 100%; border: 1px solid var(--line); border-radius: 7px; padding: 9px 10px; font-size: 14px; }
    pre { min-height: 160px; max-height: 320px; overflow: auto; margin: 0; padding: 12px; border-radius: 7px;
          background: #111827; color: #dbeafe; white-space: pre-wrap; font-size: 13px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: end; }
    .row > * { flex: 1 1 160px; }
    .camera-strip { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
    .camera-view { display: grid; gap: 8px; }
    .camera-view .cap { display:flex; justify-content:space-between; align-items:center; font-size:13px; color:var(--muted); }
    .camera-view .cap b { color: var(--text); }
    .camera-view img { width: 100%; aspect-ratio: 16 / 9; object-fit: contain;
                       background: #111827; border: 1px solid var(--line); border-radius: 7px; }
    .hint { color: var(--muted); font-size: 13px; font-weight: 400; }
    .steps { display: flex; gap: 8px; align-items: center; flex-wrap: wrap;
             margin-top: 12px; font-size: 13px; }
    .steppreset { display: inline-flex; align-items: center; border: 1px solid var(--line);
                  border-radius: 7px; padding: 3px 6px; cursor: pointer; background: #fff; }
    .steppreset input { width: 66px; border: 0; padding: 4px; font-size: 14px; }
    .steppreset.active { border: 2px solid var(--accent); padding: 2px 5px; }
    .steppreset.active input { color: var(--accent); font-weight: 700; }
    button:disabled { opacity: .4; cursor: not-allowed; }
    button:disabled:hover { border-color: var(--line); color: inherit; }
    .armfields { border: 0; padding: 0; margin: 0; }
    .board-wrap { display: grid; grid-template-columns: auto 1fr; gap: 18px; align-items: start; }
    .board-grid { display: grid; grid-template-columns: repeat(10, 42px); gap: 4px; }
    .board-grid .cell { min-height: 34px; padding: 0; font-size: 11px; color: var(--muted); }
    .board-grid .cell.taught { background: #e8f6ea; border-color: #7bc47f; color: #1a7f37; }
    .board-grid .cell.sel { border: 2px solid var(--accent); color: var(--accent); font-weight: 700; }
    .board-side { display: grid; gap: 10px; }
    .board-side .hint { color: var(--muted); font-size: 13px; }
    @media (max-width: 760px) {
      main { grid-template-columns: 1fr; padding: 14px; }
      .wide { grid-column: auto; }
      .camera-strip { grid-template-columns: 1fr; }
      .board-wrap { grid-template-columns: 1fr; }
      .board-grid { grid-template-columns: repeat(10, 1fr); }
    }
  </style>
</head>
<body>
  <header>
    <h1>WHD 双臂末端控制（实时视觉）</h1>
    <button class="primary" onclick="initSdk()">初始化 SDK</button>
  </header>
  <main>
    <!-- 三路实时视频，置顶 -->
    <section class="wide">
      <h2>实时相机（三路同屏）</h2>
      <div class="camera-strip">
        <div class="camera-view">
          <div class="cap"><b>头部视角</b><button onclick="reloadStream('head_left')">重连</button></div>
          <img id="live-head_left" alt="head_left">
        </div>
        <div class="camera-view">
          <div class="cap"><b>左手相机</b><button onclick="reloadStream('left_arm')">重连</button></div>
          <img id="live-left_arm" alt="left_arm">
        </div>
        <div class="camera-view">
          <div class="cap"><b>右手相机</b><button onclick="reloadStream('right_arm')">重连</button></div>
          <img id="live-right_arm" alt="right_arm">
        </div>
      </div>
    </section>

    <section class="wide">
      <h2>参数</h2>
      <div class="row">
        <label>头部转动步进 rad
          <input id="step" type="number" min="0.005" max="0.08" step="0.005" value="0.02"></label>
        <label>动作超时 s
          <input id="timeout" type="number" min="2" max="30" step="1" value="8"></label>
        <label>夹爪开口 m
          <input id="gripperOpen" type="number" min="0" max="0.12" step="0.005" value="0.10"></label>
        <label>夹爪闭合 m
          <input id="gripperClose" type="number" min="0" max="0.12" step="0.005" value="0.02"></label>
        <label>身体升降步进
          <input id="bodyStep" type="number" min="0.02" max="0.20" step="0.01" value="0.05"></label>
        <label class="check"><input id="snappy" type="checkbox">跟手模式（jog 加速 ×1.5 + 更高频率）</label>
        <label class="check"><input id="legacyJog" type="checkbox">兼容模式（旧版规划式 jog，流式异常时勾选）</label>
        <label class="check"><input id="collisionCheck" type="checkbox">启用碰撞检测</label>
      </div>
      <div class="steps">
        <b>水平步进 XY(m)</b>
        <span class="steppreset" id="xyChip0" onclick="selStep('xy',0)">
          <input id="xyStep0" type="number" min="0.001" max="0.08" step="0.001" value="0.005"></span>
        <span class="steppreset" id="xyChip1" onclick="selStep('xy',1)">
          <input id="xyStep1" type="number" min="0.001" max="0.08" step="0.001" value="0.02"></span>
        <span class="steppreset" id="xyChip2" onclick="selStep('xy',2)">
          <input id="xyStep2" type="number" min="0.001" max="0.08" step="0.001" value="0.04"></span>
        <b>升降步进 Z(m)</b>
        <span class="steppreset" id="zChip0" onclick="selStep('z',0)">
          <input id="zStep0" type="number" min="0.001" max="0.08" step="0.001" value="0.005"></span>
        <span class="steppreset" id="zChip1" onclick="selStep('z',1)">
          <input id="zStep1" type="number" min="0.001" max="0.08" step="0.001" value="0.01"></span>
        <span class="steppreset" id="zChip2" onclick="selStep('z',2)">
          <input id="zStep2" type="number" min="0.001" max="0.08" step="0.001" value="0.03"></span>
        <span class="hint">点击选档（蓝框为当前档），数值可直接改；水平与升降各自独立</span>
      </div>
    </section>

    <section>
      <h2>右臂末端（主操作臂） <span class="hint">键盘：W/S 前后　A/D 左右　E/Q 或 R/F 升降　O/C 夹爪</span></h2>
      <div class="grid">
        <button onpointerdown="holdMove(event,'right_arm','x',1)">前 +X</button>
        <button onpointerdown="holdMove(event,'right_arm','z',1)">上 +Z</button>
        <button onpointerdown="holdMove(event,'right_arm','y',1)">左 +Y</button>
        <button onpointerdown="holdMove(event,'right_arm','x',-1)">后 -X</button>
        <button onpointerdown="holdMove(event,'right_arm','z',-1)">下 -Z</button>
        <button onpointerdown="holdMove(event,'right_arm','y',-1)">右 -Y</button>
        <button onclick="gripper('right','open')">张开夹爪</button>
        <button onclick="gripper('right','close')">闭合夹爪</button>
        <button onclick="gripperState('right')">夹爪状态</button>
        <button onclick="pose('right_arm')">读取位姿</button>
        <button class="primary" onclick="boardHome()">右臂复位</button>
        <button onclick="saveHome()">当前位姿设为复位点</button>
      </div>
    </section>

    <section>
      <h2>左臂末端 <span class="hint">已停用（硬件故障）</span></h2>
      <label class="check">
        <input id="leftEnable" type="checkbox"
               onchange="document.getElementById('leftFields').disabled = !this.checked">
        临时启用左臂控制（确认硬件恢复后再勾选）
      </label>
      <fieldset id="leftFields" class="armfields" disabled>
        <div class="grid">
          <button onpointerdown="holdMove(event,'left_arm','x',1)">前 +X</button>
          <button onpointerdown="holdMove(event,'left_arm','z',1)">上 +Z</button>
          <button onpointerdown="holdMove(event,'left_arm','y',1)">左 +Y</button>
          <button onpointerdown="holdMove(event,'left_arm','x',-1)">后 -X</button>
          <button onpointerdown="holdMove(event,'left_arm','z',-1)">下 -Z</button>
          <button onpointerdown="holdMove(event,'left_arm','y',-1)">右 -Y</button>
          <button onclick="gripper('left','open')">张开夹爪</button>
          <button onclick="gripper('left','close')">闭合夹爪</button>
          <button onclick="gripperState('left')">夹爪状态</button>
          <button onclick="pose('left_arm')">读取位姿</button>
          <button class="primary" onclick="moveToPose('left_arm','left_home_A')">左臂到位姿A</button>
        </div>
      </fieldset>
    </section>

    <section class="wide">
      <h2>棋盘标定与抓放</h2>
      <div class="board-wrap">
        <div id="boardGrid" class="board-grid"></div>
        <div class="board-side">
          <div>选中点：<b id="selPoint">-</b> <span id="selInfo" class="hint"></span>
               <span id="boardCount" class="hint"></span></div>
          <div class="hint">坐标：列 a-j（横向 10）、行 1-9（纵向 9），左下 a1、右上 j9。
               流程：用上面右臂按钮把夹爪 jog 到某交叉点抓取高度 → 点网格选中该点 →
               「标记当前位姿」。标满 4 角（a1 j1 a9 j9）后，其余点先「移到点上空」目测，偏了再微调补标。</div>
          <div class="grid">
            <button class="primary" onclick="boardRecord()">标记当前位姿为选中点</button>
            <button onclick="boardGoto(true)">移到选中点上空</button>
            <button onclick="boardGoto(false)">下到抓取高度</button>
            <button class="danger" onclick="boardRemove()">删除选中点标定</button>
            <button onclick="setSrc()">选中点设为起点</button>
            <button onclick="setDst()">选中点设为终点</button>
          </div>
          <div class="row">
            <label>释放高度 m（落子松爪高度，相对标定抓取高度）
              <input id="cfgDrop" type="number" min="0" max="0.10" step="0.005"
                     onchange="saveBoardCfg('drop', this.value)"></label>
            <label>悬停高度 m（平移安全高度）
              <input id="cfgHover" type="number" min="0.03" max="0.30" step="0.01"
                     onchange="saveBoardCfg('hover', this.value)"></label>
            <label>直线速度 m/s（抓放每段的移动速度）
              <input id="cfgSpeed" type="number" min="0.02" max="0.30" step="0.02"
                     onchange="saveBoardCfg('stream_speed', this.value)"></label>
            <label>抓放开口 m
              <input id="cfgOpenW" type="number" min="0" max="0.12" step="0.005"
                     onchange="saveBoardCfg('open_width', this.value)"></label>
            <label>抓放闭合 m
              <input id="cfgCloseW" type="number" min="0" max="0.12" step="0.005"
                     onchange="saveBoardCfg('close_width', this.value)"></label>
          </div>
          <div class="row">
            <label>抓放起点 <input id="mvSrc" placeholder="如 b1"></label>
            <label>抓放终点 <input id="mvDst" placeholder="如 e3"></label>
            <button class="primary" onclick="boardMove()">执行抓放 A→B</button>
          </div>
        </div>
      </div>
    </section>

    <section class="wide">
      <h2>全局</h2>
      <div class="row">
        <button onclick="motionZero()">SDK 零位</button>
        <button onclick="if(confirm('该动作会同时驱动左臂（当前已停用），确定执行？')) preset('ready')">双臂准备姿态</button>
        <button class="danger" onclick="stop()">停止轨迹</button>
        <button class="danger" onclick="shutdownSdk()">释放 SDK</button>
      </div>
    </section>

    <section>
      <h2>头部姿态</h2>
      <div class="grid">
        <button onclick="headMove(0, 1)">左转</button>
        <button onclick="headMove(1, 1)">上看</button>
        <button onclick="headPose()">读取头部</button>
        <button onclick="headMove(0, -1)">右转</button>
        <button onclick="headMove(1, -1)">下看</button>
        <button onclick="headSet(0, 0)">回中</button>
      </div>
    </section>

    <section>
      <h2>身体升降</h2>
      <div class="grid">
        <button onclick="bodyMove(1)">身体上升</button>
        <button onclick="bodyMove(-1)">身体下降</button>
        <button onclick="bodyPose()">读取身体</button>
        <button onclick="bodyPreset('high')">标准高位</button>
        <button onclick="bodyPreset('low')">低位</button>
        <button onclick="bodyPreset('current_safe')">校正姿态</button>
      </div>
    </section>

    <section class="wide">
      <h2>状态</h2>
      <pre id="log"></pre>
    </section>
  </main>
  <script>
    const log = document.getElementById('log');
    function write(x) {
      const s = typeof x === 'string' ? x : JSON.stringify(x, null, 2);
      log.textContent = new Date().toLocaleTimeString() + "  " + s + "\n" + log.textContent;
    }
    async function call(path, body = {}) {
      try {
        const res = await fetch(path, { method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(body) });
        write(await res.json());
      } catch (e) { write("请求失败: " + e); }
    }
    function step() { return Number(document.getElementById('step').value || 0.02); }
    function timeout() { return Number(document.getElementById('timeout').value || 8); }
    function collisionCheck() { return document.getElementById('collisionCheck').checked; }
    function snappy() { return document.getElementById('snappy').checked; }
    function legacyJog() { return document.getElementById('legacyJog').checked; }
    // 步进档位：XY 与 Z 各 3 档，点击切换，数值可编辑
    let stepSel = { xy: 1, z: 1 };
    function selStep(group, i) {
      stepSel[group] = i;
      localStorage.setItem('whd_stepSel_' + group, String(i));
      renderStepChips();
    }
    function renderStepChips() {
      for (const g of ['xy', 'z'])
        for (let i = 0; i < 3; i++)
          document.getElementById(g + 'Chip' + i).classList.toggle('active', stepSel[g] === i);
    }
    function jogStep(axis) {
      const g = axis === 'z' ? 'z' : 'xy';
      const v = Number(document.getElementById(g + 'Step' + stepSel[g]).value || 0.02);
      return snappy() ? v * 1.5 : v;
    }
    function jogInterval() { return snappy() ? 90 : 120; }
    // 流式 jog 速度 m/s：由当前档步进换算（0.02 -> 0.12m/s），服务端再钳位
    function jogSpeed(axis) { return jogStep(axis) * 6; }
    // 参数记忆：输入框与开关写入 localStorage，刷新/下次打开自动恢复
    const PERSIST_IDS = ['step','timeout','gripperOpen','gripperClose','bodyStep',
                         'xyStep0','xyStep1','xyStep2','zStep0','zStep1','zStep2',
                         'snappy','legacyJog','collisionCheck'];
    function loadPrefs() {
      for (const id of PERSIST_IDS) {
        const el = document.getElementById(id);
        const v = localStorage.getItem('whd_' + id);
        if (!el || v === null) continue;
        if (el.type === 'checkbox') el.checked = v === '1'; else el.value = v;
      }
    }
    function savePrefs() {
      for (const id of PERSIST_IDS) {
        const el = document.getElementById(id);
        if (el) localStorage.setItem('whd_' + id,
          el.type === 'checkbox' ? (el.checked ? '1' : '0') : el.value);
      }
    }
    document.addEventListener('change', savePrefs);
    loadPrefs();
    for (const g of ['xy', 'z']) {
      const v = localStorage.getItem('whd_stepSel_' + g);
      if (v !== null) stepSel[g] = Math.min(2, Math.max(0, Number(v)));
    }
    renderStepChips();
    // 视频流控制（含自动重连）
    const STREAMS = ['head_left','left_arm','right_arm'];
    const streamRetry = {};
    function streamSrc(name){ return '/stream?camera=' + encodeURIComponent(name) + '&t=' + Date.now(); }
    function reloadStream(name){ document.getElementById('live-' + name).src = streamSrc(name); }
    function startStreams(){ STREAMS.forEach(reloadStream); }
    // 自动重连：流断开 onerror 后 2s 重试；另每 5s 健康检查——SDK 已初始化
    // 但图像未在播（包括刚打开页面还没拉流、服务重启后连接失效）就自动重拉
    for (const name of STREAMS) {
      document.getElementById('live-' + name).onerror = () => {
        clearTimeout(streamRetry[name]);
        streamRetry[name] = setTimeout(() => reloadStream(name), 2000);
      };
    }
    setInterval(async () => {
      try {
        const res = await fetch('/api/status', { method: 'POST',
          headers: {'Content-Type': 'application/json'}, body: '{}' });
        const d = await res.json();
        if (!d.initialized) return;
        for (const name of STREAMS) {
          const img = document.getElementById('live-' + name);
          if (!img.src || img.naturalWidth === 0) reloadStream(name);
        }
      } catch (e) { /* 服务不可达时静默，恢复后下一轮自动重拉 */ }
    }, 5000);

    async function initSdk() {
      await call('/api/init');
      setTimeout(startStreams, 500);   // 初始化后再拉流
    }
    function shutdownSdk() { call('/api/shutdown'); }
    function stop() { call('/api/stop'); }
    function pose(arm) { call('/api/pose', {arm}); }
    // 预设位姿（x,y,z,qx,qy,qz,qw）
    const PRESET_POSES = {
      left_home_A: [0.29628746746183005, 0.19529178892271168, 1.246997995222601,
                    0.5121277084358556, 0.33666764246402336, -0.44672314889309833, 0.6517810499032559],
    };
    function moveToPose(arm, presetName) {
      call('/api/move_pose', { arm, target_pose: PRESET_POSES[presetName],
        timeout: timeout(), enable_collision_check: collisionCheck() });
    }
    // 右臂复位点存服务端 board_points.json（config.home_pose），
    // 网页按钮 / CLI / 抓放结束自动复位 三方共用同一个值
    function boardHome() { call('/api/board_home', { timeout: timeout() }); }
    function saveHome() { call('/api/board_home_set'); }
    function saveBoardCfg(key, value) {
      call('/api/board_config', { key, value: Number(value) });
    }
    function headPose() { call('/api/head_pose'); }
    function headMove(index, dir) { call('/api/head_move', {index, dir, step: step(), timeout: timeout()}); }
    function headSet(joint1, joint2) { call('/api/head_set', {positions: [joint1, joint2], timeout: timeout()}); }
    function bodyStep() { return Number(document.getElementById('bodyStep').value || 0.05); }
    function bodyPose() { call('/api/body_pose'); }
    function bodyMove(dir) { call('/api/body_move', {dir, step: bodyStep(), timeout: timeout()}); }
    function bodyPreset(name) { call('/api/body_preset', {name, timeout: timeout()}); }
    function motionZero() { call('/api/motion_zero', {timeout: timeout()}); }
    // 按住持续动、松手立即停（类似油门）：
    // 按住期间以低延迟非阻塞方式连发小步增量；松手立刻 /api/hold_stop 急停轨迹，
    // 并在最后一条在途 move 落地后再补发一次 stop，防止它比 stop 晚到导致多走一步。
    // 鼠标按钮与键盘 wsadrf 共用 startHold/endHold。
    let holdTimer = null, holdBusy = false, holding = false, holdArm = null, holdLegacy = false;
    function sendStop(arm) {
      fetch('/api/hold_stop', { method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ arm }) })
        .then(res => res.json()).then(data => { if (!data.ok) write(data); })
        .catch(e => write("急停失败: " + e));
    }
    function startHold(arm, axis, dir) {
      if (holding) return;
      holding = true; holdArm = arm; holdLegacy = legacyJog();
      if (holdTimer) clearInterval(holdTimer);
      if (holdLegacy) {
        // 旧版：规划式点动（每条指令走一次运动规划）
        const send = () => {
          if (!holding || holdBusy) return; holdBusy = true;
          fetch('/api/move', { method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ arm, axis, dir, step: jogStep(axis), timeout: 1,
              enable_collision_check: collisionCheck(), low_latency: true }) })
            .then(res => res.json()).then(data => { if (!data.ok) write(data); })
            .catch(e => write("请求失败: " + e)).finally(() => { holdBusy = false; });
        };
        send();
        holdTimer = setInterval(send, jogInterval());
      } else {
        // 流式：服务端 30Hz IK+set_joint_commands；这里只按 150ms 续期（TTL 0.5s）
        const send = () => {
          if (!holding) return;
          fetch('/api/jog', { method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ arm, axis, dir, speed: jogSpeed(axis) }) })
            .then(res => res.json()).then(data => { if (!data.ok) { write(data); endHold(); } })
            .catch(e => write("请求失败: " + e));
        };
        send();
        holdTimer = setInterval(send, 150);
      }
    }
    function endHold() {
      if (!holding) return;
      holding = false;
      if (holdTimer) clearInterval(holdTimer); holdTimer = null;
      const arm = holdArm;
      if (holdLegacy) {
        sendStop(arm);
        const t = setInterval(() => { if (!holdBusy) { clearInterval(t); sendStop(arm); } }, 50);
        setTimeout(() => clearInterval(t), 2000);
      } else {
        fetch('/api/jog_stop', { method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({ arm }) })
          .then(res => res.json()).then(data => { if (!data.ok) write(data); })
          .catch(e => write("停止失败: " + e));
      }
    }
    function holdMove(event, arm, axis, dir) {
      event.preventDefault();
      startHold(arm, axis, dir);
      const stopHold = () => {
        endHold();
        window.removeEventListener('pointerup', stopHold);
        window.removeEventListener('pointercancel', stopHold);
      };
      window.addEventListener('pointerup', stopHold);
      window.addEventListener('pointercancel', stopHold);
    }
    // 键盘 jog（右臂）：按住 w/s=前后 a/d=左右 r/f 或 e/q=升降，松开立即停；o/c=夹爪开/合
    const KEY_JOG = { w: ['x', 1], s: ['x', -1], a: ['y', 1], d: ['y', -1],
                      r: ['z', 1], f: ['z', -1], e: ['z', 1], q: ['z', -1] };
    let keyHeld = null;
    window.addEventListener('keydown', (e) => {
      if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
      const tag = (e.target.tagName || '').toUpperCase();
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      const k = e.key.toLowerCase();
      if (k === 'o') { gripper('right', 'open'); return; }
      if (k === 'c') { gripper('right', 'close'); return; }
      const m = KEY_JOG[k];
      if (!m || keyHeld) return;
      e.preventDefault();
      keyHeld = k;
      startHold('right_arm', m[0], m[1]);
    });
    window.addEventListener('keyup', (e) => {
      if (e.key.toLowerCase() === keyHeld) { keyHeld = null; endHold(); }
    });
    window.addEventListener('blur', () => { if (keyHeld) { keyHeld = null; endHold(); } });
    function preset(name) { call('/api/preset', {name, timeout: timeout()}); }
    function gripper(side, mode) {
      const width = mode === 'open'
        ? Number(document.getElementById('gripperOpen').value || 0.10)
        : Number(document.getElementById('gripperClose').value || 0.02);
      call('/api/gripper', {side, width});
    }
    function gripperState(side) { call('/api/gripper_state', {side}); }

    // ---- 棋盘标定与抓放 ---- //
    let selPoint = null, boardPoints = {}, boardBusy = false;
    async function boardState() {
      try {
        const res = await fetch('/api/board_state', { method: 'POST',
          headers: {'Content-Type': 'application/json'}, body: '{}' });
        const d = await res.json();
        if (!d.ok) { write(d); return; }
        boardPoints = d.points || {};
        document.getElementById('boardCount').textContent =
          '｜已标定 ' + d.count + '/90｜执行臂 ' + d.config.arm;
        // 回填可调配置（正在编辑的输入框不覆盖）
        for (const [id, key] of [['cfgDrop', 'drop'], ['cfgHover', 'hover'],
                                 ['cfgSpeed', 'stream_speed'],
                                 ['cfgOpenW', 'open_width'], ['cfgCloseW', 'close_width']]) {
          const el = document.getElementById(id);
          if (el && document.activeElement !== el && d.config[key] !== undefined)
            el.value = d.config[key];
        }
        renderBoard();
      } catch (e) { write('读取标定数据失败: ' + e); }
    }
    function renderBoard() {
      const grid = document.getElementById('boardGrid');
      grid.innerHTML = '';
      // 横 10 列（a→j）× 竖 9 行（上 9 → 下 1），与棋盘实际摆放一致：
      // 左上 a9，右上 j9，左下 a1，右下 j1
      for (let r = 9; r >= 1; r--) {
        for (const c of 'abcdefghij') {
          const name = c + r;
          const btn = document.createElement('button');
          btn.className = 'cell' + (boardPoints[name] ? ' taught' : '')
                                 + (name === selPoint ? ' sel' : '');
          btn.textContent = name;
          btn.onclick = () => { selPoint = name; renderBoard(); updateSel(); };
          grid.appendChild(btn);
        }
      }
    }
    function updateSel() {
      document.getElementById('selPoint').textContent = selPoint || '-';
      document.getElementById('selInfo').textContent = !selPoint ? ''
        : (boardPoints[selPoint] ? '（已标定）' : '（未标定，运动将用插值）');
    }
    async function boardCall(path, body) {
      if (boardBusy) { write('上一个棋盘动作还在执行中，请稍候'); return; }
      boardBusy = true;
      try { await call(path, body); } finally { boardBusy = false; }
      boardState();
    }
    function needSel() { if (!selPoint) { write('先在网格上点选一个点'); return false; } return true; }
    function boardRecord() { if (needSel()) boardCall('/api/board_record', {point: selPoint}); }
    function boardRemove() { if (needSel()) boardCall('/api/board_remove', {point: selPoint}); }
    function boardGoto(hoverOnly) {
      if (!needSel()) return;
      write((hoverOnly ? '移动到 ' : '下到 ') + selPoint + ' ...');
      boardCall('/api/board_goto', {point: selPoint, hover_only: hoverOnly, timeout: timeout()});
    }
    function setSrc() { if (needSel()) document.getElementById('mvSrc').value = selPoint; }
    function setDst() { if (needSel()) document.getElementById('mvDst').value = selPoint; }
    function boardMove() {
      const src = document.getElementById('mvSrc').value.trim();
      const dst = document.getElementById('mvDst').value.trim();
      if (!src || !dst) { write('请先填写抓放起点和终点'); return; }
      write('抓放 ' + src + ' → ' + dst + ' 执行中（期间视频会短暂卡顿）...');
      // 夹爪宽度不再随请求传：统一用服务端 board_points.json 配置
      // （上方「抓放开口/闭合」输入框改的就是它）
      boardCall('/api/board_move', { src, dst, timeout: timeout() });
    }
    boardState();
    call('/api/status');
  </script>
</body>
</html>
"""


class RobotController:
    def __init__(self):
        self.lock = threading.Lock()
        self.robot = None
        self.motion = None
        self.initialized = False
        self.body_anchor = None
        self.cached_poses = {}
        self._capture_started = False
        # 高频流式 jog（IK + set_joint_commands）状态
        self.jog_lock = threading.Lock()
        self.jog_state = {"arm": None, "axis": "x", "dir": 1, "speed": 0.1, "deadline": 0.0}
        self.jog_thread = None
        # 棋盘段直线规划自适应：连续失败 2 次后本进程不再尝试直线
        self._line_fail_streak = 0
        self._line_disabled = False

    def init(self):
        if SDK_IMPORT_ERROR:
            return {"ok": False, "error": "SDK import failed", "detail": SDK_IMPORT_ERROR}
        with self.lock:
            if self.initialized:
                return {"ok": True, "message": "SDK already initialized"}
            self.robot = GalbotRobot()
            self.motion = GalbotMotion()
            if SensorType is not None:
                sensors = {
                    SensorType.HEAD_LEFT_CAMERA,
                    SensorType.LEFT_ARM_CAMERA,
                    SensorType.RIGHT_ARM_CAMERA,
                }
                robot_ok = self.robot.init(sensors)
            else:
                robot_ok = self.robot.init()
            try:
                motion_ok = self.motion.init()
            except Exception:
                motion_ok = False
            time.sleep(2)
            self.initialized = bool(robot_ok and motion_ok)
        if self.initialized:
            self._start_capture()
        return {
            "ok": self.initialized,
            "robot_init": bool(robot_ok),
            "motion_init": bool(motion_ok),
            "note": "Cartesian end-effector control requires both robot_init and motion_init. RGB cameras are enabled during robot_init.",
        }

    # ---- 实时视频采集 ---- #
    def _start_capture(self):
        if self._capture_started:
            return
        self._capture_started = True
        for name in STREAM_CAMERAS:
            threading.Thread(target=self._capture_loop, args=(name,), daemon=True).start()

    def _capture_loop(self, name):
        sensor = self._camera_sensor(name)
        slot = CAMERA_FRAMES[name]
        period = 1.0 / CAPTURE_FPS
        last_report = 0.0
        ok_count = empty_count = 0
        while self.initialized:
            t0 = time.time()
            try:
                with self.lock:
                    data = self.robot.get_rgb_data(sensor) if self.robot else None
                if data and "data" in data:
                    raw = data["data"]
                    jpeg = base64.b64decode(raw) if isinstance(raw, str) else bytes(raw)
                    slot.set(jpeg)
                    ok_count += 1
                else:
                    empty_count += 1
                    if empty_count <= 3 or empty_count % 30 == 0:
                        keys = list(data.keys()) if isinstance(data, dict) else type(data).__name__
                        print(f"[capture:{name}] empty frame #{empty_count}, data={keys}")
            except Exception as exc:
                print(f"[capture:{name}] EXC: {exc!r}")
                time.sleep(0.5)
            if t0 - last_report >= 5.0:
                print(f"[capture:{name}] ok={ok_count} empty={empty_count} in last 5s")
                ok_count = empty_count = 0
                last_report = t0
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def require_init(self):
        if not self.initialized or self.robot is None or self.motion is None:
            raise RuntimeError("SDK not initialized. Click 初始化 SDK first.")

    def pose(self, arm):
        self.require_init()
        self._check_arm(arm)
        with self.lock:
            status, pose = self.motion.get_end_effector_pose_on_chain(
                chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
            if self._motion_ok(status):
                self.cached_poses[arm] = self._list(pose)
        return {"ok": self._motion_ok(status), "status": str(status), "arm": arm, "pose": self._list(pose)}

    def head_pose(self):
        self.require_init()
        with self.lock:
            names = self.robot.get_joint_names(True, ["head"])
            positions = self.robot.get_joint_positions(["head"], [])
        return {"ok": True, "names": list(names), "positions": self._list(positions)}

    def head_set(self, positions, timeout):
        self.require_init()
        positions = self._list(positions or [])
        if len(positions) != 2:
            raise ValueError("head positions must contain 2 values")
        positions = [max(-1.2, min(float(v), 1.2)) for v in positions]
        timeout = max(2.0, min(float(timeout), 30.0))
        with self.lock:
            status = self.robot.set_joint_positions(positions, ["head"], [], True, 0.15, timeout)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status), "positions": positions}

    def head_move(self, index, direction, step, timeout):
        self.require_init()
        index = int(index)
        if index < 0 or index >= 2:
            raise ValueError("head joint index must be 0..1")
        step = max(0.005, min(float(step), 0.20))
        direction = 1 if float(direction) >= 0 else -1
        timeout = max(2.0, min(float(timeout), 30.0))
        with self.lock:
            positions = self._list(self.robot.get_joint_positions(["head"], []))
            if len(positions) < 2:
                raise RuntimeError(f"Expected 2 head joints, got {len(positions)}")
            positions[index] = max(-1.2, min(positions[index] + direction * step, 1.2))
            status = self.robot.set_joint_positions(positions, ["head"], [], True, 0.15, timeout)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status), "positions": positions}

    def body_pose(self):
        self.require_init()
        with self.lock:
            names = self.robot.get_joint_names(True, ["leg"])
            positions = self._list(self.robot.get_joint_positions(["leg"], []))
            if self.body_anchor is None and len(positions) >= 5:
                self.body_anchor = list(positions[:5])
        return {"ok": True, "names": list(names), "positions": positions,
                "anchor_positions": self.body_anchor, "height_delta": self._body_delta(positions)}

    def body_preset(self, name, timeout):
        self.require_init()
        timeout = max(3.0, min(float(timeout), 30.0))
        current = self._list(self.robot.get_joint_positions(["leg"], []))
        if len(current) < 5:
            raise RuntimeError(f"Expected 5 leg joints, got {len(current)}")
        anchor = self._ensure_body_anchor(current)
        if name == "low":
            positions = self._body_positions_from_delta(-BODY_MAX_DELTA, current, anchor)
        elif name == "high":
            positions = self._body_positions_from_delta(BODY_MAX_DELTA, current, anchor)
        elif name == "current_safe":
            self.body_anchor = list(current[:5])
            positions = list(current[:5])
        else:
            raise ValueError("body preset must be low/high/current_safe")
        with self.lock:
            status = self.robot.set_joint_positions(positions, ["leg"], [], True, 0.10, timeout)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status), "preset": name,
                "current_positions": current, "positions": positions,
                "anchor_positions": self.body_anchor, "height_delta": self._body_delta(positions)}

    def body_move(self, direction, step, timeout):
        self.require_init()
        direction = 1 if float(direction) >= 0 else -1
        step = max(0.01, min(float(step), 0.20))
        timeout = max(3.0, min(float(timeout), 30.0))
        with self.lock:
            current = self._list(self.robot.get_joint_positions(["leg"], []))
            if len(current) < 5:
                raise RuntimeError(f"Expected 5 leg joints, got {len(current)}")
            anchor = self._ensure_body_anchor(current)
            current_delta = self._body_delta(current)
            target_delta = max(-BODY_MAX_DELTA, min(current_delta + direction * step, BODY_MAX_DELTA))
            positions = self._body_positions_from_delta(target_delta, current, anchor)
            status = self.robot.set_joint_positions(positions, ["leg"], [], True, 0.10, timeout)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status),
                "current_positions": current, "positions": positions,
                "anchor_positions": self.body_anchor, "height_delta": self._body_delta(positions)}

    def motion_zero(self, timeout):
        self.require_init()
        timeout = max(5.0, min(float(timeout), 30.0))
        params = gm.Parameter()
        with self.lock:
            status = self.motion.move_whole_body_joint_zero(True, 0.2, timeout, params)
        return {"ok": self._motion_ok(status), "status": str(status), "timeout": timeout,
                "hint": "If this returns FAULT, the motion planner/controller state is unhealthy before any web target is involved."}

    def move_relative(self, arm, axis, direction, step, timeout,
                      enable_collision_check=False, low_latency=False):
        self.require_init()
        self._check_arm(arm)
        if axis not in ("x", "y", "z"):
            raise ValueError("axis must be x/y/z")
        step = max(0.001, min(float(step), 0.08))
        direction = 1 if float(direction) >= 0 else -1
        timeout = max(0.2, min(float(timeout), 30.0))
        low_latency = bool(low_latency)

        with self.lock:
            if low_latency and arm in self.cached_poses:
                target = list(self.cached_poses[arm])
                pose_status = gm.MotionStatus.SUCCESS
            else:
                pose_status, pose = self.motion.get_end_effector_pose_on_chain(
                    chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
                if not self._motion_ok(pose_status):
                    return {"ok": False, "stage": "get_pose", "status": str(pose_status)}
                target = self._list(pose)
            idx = {"x": 0, "y": 1, "z": 2}[axis]
            target[idx] += direction * step

            params = gm.Parameter(True, True, min(timeout, 1.0) if low_latency else timeout,
                                  "with_chain_only", False, bool(enable_collision_check), "base_link")
            params.set_move_line(False)

            status = self.motion.set_end_effector_pose(
                target_pose=target, end_effector_frame=arm, reference_frame="base_link",
                enable_collision_check=bool(enable_collision_check),
                is_blocking=not low_latency,
                timeout=min(timeout, 1.0) if low_latency else timeout, params=params)
            command_ok = self._motion_command_ok(status, low_latency)
            if command_ok:
                self.cached_poses[arm] = list(target)
        result = {"ok": self._motion_command_ok(status, low_latency), "status": str(status),
                  "arm": arm, "target_pose": target, "collision_check": bool(enable_collision_check),
                  "low_latency": low_latency}
        if status == gm.MotionStatus.FAULT:
            result["hint"] = ("Motion planner reported FAULT. Try disabling collision check, "
                              "reduce step size, move to ready pose, and confirm service_motion_plan is healthy.")
        return result

    def move_pose(self, arm, target_pose, timeout, enable_collision_check=False,
                  move_line=False):
        """move_line=False 为原有已验证行为（关节空间规划）；
        move_line=True 时末端走笛卡尔直线，路径高度可控（棋盘抓放用）。"""
        self.require_init()
        self._check_arm(arm)
        target = self._list(target_pose or [])
        if len(target) != 7:
            raise ValueError("target_pose must contain 7 values: x,y,z,qx,qy,qz,qw")
        target = [float(v) for v in target]
        timeout = max(2.0, min(float(timeout), 30.0))
        with self.lock:
            params = gm.Parameter(True, True, timeout, "with_chain_only", False,
                                  bool(enable_collision_check), "base_link")
            params.set_move_line(bool(move_line))
            status = self.motion.set_end_effector_pose(
                target_pose=target, end_effector_frame=arm, reference_frame="base_link",
                enable_collision_check=bool(enable_collision_check),
                is_blocking=True, timeout=timeout, params=params)
            if self._motion_ok(status):
                self.cached_poses[arm] = list(target)
        result = {"ok": self._motion_ok(status), "status": str(status), "arm": arm,
                  "target_pose": target, "collision_check": bool(enable_collision_check),
                  "move_line": bool(move_line)}
        if status == gm.MotionStatus.FAULT:
            result["hint"] = ("Motion planner reported FAULT. Try disabling collision check, "
                              "confirm the target pose is reachable, and that SingoriX WBCS is running.")
        return result

    def _board_leg(self, arm, pose, timeout, use_line=True):
        """棋盘运动段。use_line 时先试直线（短超时 6s 兜底，失败不至于每段
        白等完整超时），失败退回关节空间；直线连续失败 2 次后本进程内不再
        尝试直线——短段（≤transit_seg）关节空间规划的下垂只有毫米级，安全性
        由切段保证，不必为直线反复付出规划超时的代价。"""
        if use_line and not self._line_disabled:
            r = self.move_pose(arm, pose, min(float(timeout), 6.0), move_line=True)
            if r.get("ok"):
                self._line_fail_streak = 0
                return r
            self._line_fail_streak += 1
            if self._line_fail_streak >= 2:
                self._line_disabled = True
                print("[board] 直线规划连续失败，后续段改用关节空间规划（短段安全，速度更快）")
            r2 = self.move_pose(arm, pose, timeout, move_line=False)
            r2["line_fallback"] = True
            return r2
        r = self.move_pose(arm, pose, timeout, move_line=False)
        if use_line:
            r["line_skipped"] = True
        return r

    def _board_transit(self, arm, target, timeout, seg=0.10, use_line=True):
        """悬停高度的长平移：切分成 ≤seg 米的短段逐段走。
        每段优先直线；即使某段退回关节空间规划，短段的路径下垂也只有毫米级，
        从根上杜绝长平移下垂刮到棋盘。"""
        cur = self.pose(arm)
        if not cur.get("ok"):
            return {"ok": False, "stage": "read_pose", "detail": cur}
        p0 = cur["pose"]
        dx = target[0] - p0[0]
        dy = target[1] - p0[1]
        dz = target[2] - p0[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        n = max(1, int(dist / seg + 0.999))
        fallbacks = 0
        for i in range(1, n + 1):
            t = i / n
            wp = [p0[0] + dx * t, p0[1] + dy * t, p0[2] + dz * t] + list(target[3:7])
            r = self._board_leg(arm, wp, timeout, use_line)
            if not r.get("ok"):
                r["waypoint"] = f"{i}/{n}"
                return r
            if r.get("line_fallback"):
                fallbacks += 1
        return {"ok": True, "waypoints": n, "dist_m": round(dist, 3),
                "line_fallbacks": fallbacks}

    @staticmethod
    def _nlerp(q0, q1, t):
        """四元数归一化线性插值（半球对齐），段内姿态平滑过渡"""
        if sum(a * b for a, b in zip(q0, q1)) < 0:
            q1 = [-v for v in q1]
        q = [(1 - t) * a + t * b for a, b in zip(q0, q1)]
        n = sum(v * v for v in q) ** 0.5 or 1.0
        return [v / n for v in q]

    def _stream_line(self, arm, target, timeout, speed=0.12):
        """流式严格直线：30Hz 沿直线插值位姿 → IK（上一拍关节角作初值）→
        set_joint_commands 直发。不经过运动规划器，笛卡尔路径就是喂进去的
        直线本身，最低点=两端点较低者，不存在规划下垂。IK 失败立即停在原地。
        与高频键盘 jog 完全同一套机制。"""
        self.require_init()
        speed = max(0.02, min(float(speed), 0.30))
        rate_dt = 1.0 / 30.0
        with self.lock:
            p_status, pose = self.motion.get_end_effector_pose_on_chain(
                chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
            joints = self._list(self.robot.get_joint_positions([arm], []))
        if not self._motion_ok(p_status) or not joints:
            return {"ok": False, "stage": "read_pose", "status": str(p_status)}
        p0 = self._list(pose)
        tgt = [float(v) for v in target]
        d = [tgt[i] - p0[i] for i in range(3)]
        dist = (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5
        n = max(1, int(dist / (speed * rate_dt) + 0.999))
        deadline = time.time() + max(float(timeout), dist / speed * 1.5 + 5.0)
        last = list(joints)
        for i in range(1, n + 1):
            if time.time() > deadline:
                return {"ok": False, "stage": f"timeout@{i}/{n}"}
            if not self.initialized:
                return {"ok": False, "stage": "sdk_shutdown"}
            t = i / n
            wp = [p0[0] + d[0] * t, p0[1] + d[1] * t, p0[2] + d[2] * t] \
                + self._nlerp(p0[3:7], tgt[3:7], t)
            tick0 = time.time()
            with self.lock:
                ik_status, sol = self.motion.inverse_kinematics(
                    target_pose=wp, chain_names=[arm],
                    initial_joint_positions={arm: list(last)},
                    enable_collision_check=False)
                if not self._motion_ok(ik_status):
                    return {"ok": False, "stage": f"ik@{i}/{n}", "status": str(ik_status),
                            "hint": "IK 失败（工作空间边界/奇异位形），机械臂已停在原地"}
                jp = sol.get(arm) if isinstance(sol, dict) else None
                if not jp and isinstance(sol, dict) and sol:
                    jp = next(iter(sol.values()))
                if not jp:
                    return {"ok": False, "stage": f"ik_empty@{i}/{n}"}
                cmds = []
                for v in jp:
                    c = gm.JointCommand()
                    c.position = float(v)
                    cmds.append(c)
                c_status = self.robot.set_joint_commands(cmds, [arm], [], 0.0)
            if c_status != ControlStatus.SUCCESS:
                return {"ok": False, "stage": f"joint_cmd@{i}/{n}", "status": str(c_status)}
            last = list(jp)
            elapsed = time.time() - tick0
            if elapsed < rate_dt:
                time.sleep(rate_dt - elapsed)
        time.sleep(0.3)   # 到点后稳定
        with self.lock:
            f_status, f_pose = self.motion.get_end_effector_pose_on_chain(
                chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
        final_err = None
        if self._motion_ok(f_status):
            fp = self._list(f_pose)
            self.cached_poses[arm] = fp
            final_err = round(sum((fp[i] - tgt[i]) ** 2 for i in range(3)) ** 0.5, 4)
        return {"ok": True, "mode": "stream_line", "dist_m": round(dist, 3),
                "ticks": n, "final_err_m": final_err}

    def _board_movers(self, cfg):
        """按配置返回 (leg, transit) 两个运动执行器。
        use_stream=1（默认）：所有段用流式严格直线（保证路径高度）；
        use_stream=0：退回规划器路径（直线优先 + 短段切分）。"""
        use_stream = float(cfg.get("use_stream", 1)) >= 0.5
        if use_stream:
            speed = float(cfg.get("stream_speed", 0.12))

            def sleg(a, t, to):
                return self._stream_line(a, t, to, speed)

            return sleg, sleg
        seg = float(cfg.get("transit_seg", 0.10))
        use_line = float(cfg.get("use_line", 1)) >= 0.5

        def leg(a, t, to):
            return self._board_leg(a, t, to, use_line)

        def transit(a, t, to):
            return self._board_transit(a, t, to, seg, use_line)

        return leg, transit

    def gripper(self, side, width):
        self.require_init()
        side = str(side)
        if side == "left":
            group = G1JointGroup.left_gripper
        elif side == "right":
            group = G1JointGroup.right_gripper
        else:
            raise ValueError("side must be left/right")
        width = max(0.0, min(float(width), 0.12))
        with self.lock:
            status = self.robot.set_gripper_command(group, width, 0.05, 10, True)
            fallback = False
            if status == ControlStatus.DATA_FETCH_FAILED:
                # 阻塞模式需要读夹爪状态反馈来确认到位；状态流不可用时
                # 降级为非阻塞纯发指令（指令通道若正常，夹爪仍会动作）
                status = self.robot.set_gripper_command(group, width, 0.05, 10, False)
                fallback = True
        result = {"ok": status == ControlStatus.SUCCESS, "status": str(status),
                  "side": side, "width": width, "non_blocking_fallback": fallback}
        if status == ControlStatus.DATA_FETCH_FAILED:
            result["hint"] = ("夹爪状态数据取不到。先点「夹爪状态」诊断；若也失败："
                              "1) 重启本面板进程（robot.init 每个进程只首次生效，"
                              "「释放 SDK」后网页重初始化会出现这种半失效状态）；"
                              "2) 仍失败则检查急停/夹爪硬件，必要时重启机器人。")
        return result

    def gripper_state(self, side):
        """诊断用：直接读夹爪状态流"""
        self.require_init()
        side = str(side)
        if side not in ("left", "right"):
            raise ValueError("side must be left/right")
        with self.lock:
            st = self.robot.get_gripper_state(f"{side}_gripper")
        if st is None:
            return {"ok": False, "side": side, "state": None,
                    "hint": f"{side}_gripper 状态流无数据：夹爪驱动没在发布状态。"
                            "对比另一侧夹爪；两侧都无数据则重启机器人让控制栈重新拉起。"}
        return {"ok": True, "side": side,
                "width": float(st.width), "velocity": float(st.velocity),
                "effort": float(st.effort), "is_moving": bool(st.is_moving),
                "timestamp_ns": int(st.timestamp_ns)}

    def preset(self, name, timeout):
        self.require_init()
        if name != "ready":
            raise ValueError("unknown preset")
        timeout = max(5.0, min(float(timeout), 30.0))
        joint_pos = [
            0.5, 1.5, 1.0, 0.0, 0.0,
            0.0, 0.0,
            2.0, -1.5, -0.6, -1.7, 0.0, -0.8, 0.0,
            -2.0, 1.5, 0.6, 1.7, 0.0, 0.8, 0.0,
        ]
        groups = ["leg", "head", "left_arm", "right_arm"]
        with self.lock:
            status = self.robot.set_joint_positions(joint_pos, groups, [], True, 0.1, timeout)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status), "preset": name}

    def stop(self):
        self.require_init()
        with self.lock:
            status = self.robot.stop_trajectory_execution()
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status)}

    # ---- 高频流式 jog：每周期 IK + set_joint_commands，速度连续、松手即停 ---- #
    def jog_command(self, arm, axis, direction, speed):
        """刷新流式 jog 的方向/速度/存活期限（前端按住期间每 150ms 调一次）。
        期限 0.5s：前端断线/崩溃后最多 0.5s 自动停，防失控。"""
        self.require_init()
        self._check_arm(arm)
        if axis not in ("x", "y", "z"):
            raise ValueError("axis must be x/y/z")
        speed = max(0.005, min(float(speed), 0.25))
        direction = 1 if float(direction) >= 0 else -1
        with self.jog_lock:
            self.jog_state.update({"arm": arm, "axis": axis, "dir": direction,
                                   "speed": speed, "deadline": time.time() + 0.5})
            if not (self.jog_thread and self.jog_thread.is_alive()):
                self.jog_thread = threading.Thread(target=self._jog_loop, daemon=True)
                self.jog_thread.start()
        return {"ok": True, "mode": "joint_stream", "arm": arm, "axis": axis,
                "dir": direction, "speed": speed}

    def jog_stop(self, arm=None):
        with self.jog_lock:
            arm = arm or self.jog_state.get("arm")
            self.jog_state["deadline"] = 0.0
        t = self.jog_thread
        if t is not None:
            t.join(timeout=0.5)
        result = {"ok": True, "stopped": True, "arm": arm}
        # 刷新真实位姿缓存，保证旧版 /api/move 等后续操作不基于过期缓存
        if arm in ("left_arm", "right_arm") and self.initialized:
            with self.lock:
                status, pose = self.motion.get_end_effector_pose_on_chain(
                    chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
            if self._motion_ok(status):
                self.cached_poses[arm] = self._list(pose)
                result["pose"] = self._list(pose)
        return result

    def _jog_loop(self):
        """30Hz：目标位姿沿轴匀速推进 → IK（上一拍关节角作初值）→ 流式下发关节位置。
        期限过期 / IK 失败（工作空间边界）/ 下发失败 都会立即退出，停发即停走。"""
        rate_dt = 1.0 / 30.0
        arm = None
        target = None
        last_joints = None
        while self.initialized:
            with self.jog_lock:
                st = dict(self.jog_state)
            if time.time() > st["deadline"]:
                break
            if arm != st["arm"] or target is None:
                arm = st["arm"]
                with self.lock:
                    p_status, pose = self.motion.get_end_effector_pose_on_chain(
                        chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
                    joints = self._list(self.robot.get_joint_positions([arm], []))
                if not self._motion_ok(p_status) or not joints:
                    print(f"[jog] 初始位姿/关节读取失败: {p_status}")
                    break
                target = self._list(pose)
                last_joints = list(joints)
            tick0 = time.time()
            idx = {"x": 0, "y": 1, "z": 2}[st["axis"]]
            target[idx] += st["dir"] * st["speed"] * rate_dt
            with self.lock:
                ik_status, sol = self.motion.inverse_kinematics(
                    target_pose=list(target), chain_names=[arm],
                    initial_joint_positions={arm: list(last_joints)},
                    enable_collision_check=False)
                if not self._motion_ok(ik_status):
                    print(f"[jog] IK 失败: {ik_status}（可能到达工作空间边界/奇异位形），已停止")
                    break
                jp = sol.get(arm) if isinstance(sol, dict) else None
                if not jp and isinstance(sol, dict) and sol:
                    jp = next(iter(sol.values()))
                if not jp:
                    print("[jog] IK 无解，已停止")
                    break
                cmds = []
                for v in jp:
                    c = gm.JointCommand()
                    c.position = float(v)
                    cmds.append(c)
                c_status = self.robot.set_joint_commands(cmds, [arm], [], 0.0)
            if c_status != ControlStatus.SUCCESS:
                print(f"[jog] set_joint_commands 失败: {c_status}，已停止")
                break
            last_joints = list(jp)
            self.cached_poses[arm] = list(target)
            elapsed = time.time() - tick0
            if elapsed > 0.3:
                # 本拍被其他阻塞运动占锁太久，末端可能已被移走：下一拍重读位姿
                target = None
                continue
            if elapsed < rate_dt:
                time.sleep(rate_dt - elapsed)
        with self.jog_lock:
            self.jog_state["deadline"] = 0.0

    def hold_stop(self, arm):
        """松手急停：停掉当前轨迹，并用真实位姿覆盖低延迟缓存。
        按住期间缓存目标领先于真实位置，不刷新缓存的话下次按住会朝旧目标跳变。"""
        self.require_init()
        self._check_arm(arm)
        with self.lock:
            status = self.robot.stop_trajectory_execution()
            pose_status, pose = self.motion.get_end_effector_pose_on_chain(
                chain_name=arm, frame_id="EndEffector", reference_frame="base_link")
            if self._motion_ok(pose_status):
                self.cached_poses[arm] = self._list(pose)
            else:
                self.cached_poses.pop(arm, None)
        return {"ok": status == ControlStatus.SUCCESS, "status": str(status), "arm": arm,
                "pose": self._list(pose) if self._motion_ok(pose_status) else None}

    def shutdown(self):
        # 不调用 request_shutdown/wait_for_shutdown/destroy：这些调用会在 C++ 层
        # 阻塞且持有 GIL，把整个解释器冻死（Ctrl+C、信号处理器全部失效）。
        # 只置空引用，连接资源在进程退出时由 OS 回收。
        with self.jog_lock:
            self.jog_state["deadline"] = 0.0
        with self.lock:
            self.initialized = False
            self._capture_started = False
            self.robot = None
            self.motion = None
            self.body_anchor = None
            self.cached_poses = {}
        return {"ok": True, "message": "SDK references dropped (resources reclaimed on process exit)"}

    def status(self):
        return {"ok": True, "initialized": self.initialized, "sdk_root": SDK_ROOT,
                "sdk_import_error": SDK_IMPORT_ERROR}

    # ---- 棋盘标定 / 抓放：存储与插值复用 board_teach.py，
    #      所有运动/夹爪均走上面已验证的 pose / move_pose / gripper ---- #
    @staticmethod
    def _board_store():
        if bt is None:
            raise RuntimeError("board_teach 模块不可用: " + BT_IMPORT_ERROR)
        return bt.load_store()

    def board_state(self):
        store = self._board_store()
        return {"ok": True, "count": len(store["points"]), "config": store["config"],
                "points": {k: [round(v, 4) for v in p] for k, p in store["points"].items()}}

    def board_record(self, point):
        store = self._board_store()
        bt.parse_point(point)
        r = self.pose(store["config"]["arm"])
        if not r.get("ok"):
            return {"ok": False, "stage": "read_pose", "detail": r}
        store["points"][point.lower()] = r["pose"]
        bt.save_store(store)
        return {"ok": True, "recorded": point.lower(), "pose": r["pose"],
                "count": len(store["points"])}

    def board_remove(self, point):
        store = self._board_store()
        existed = store["points"].pop(str(point).lower(), None) is not None
        if existed:
            bt.save_store(store)
        return {"ok": True, "removed": existed, "point": str(point).lower()}

    # 网页可调的数值配置项（写入 board_points.json 的 config）
    # use_stream: 1=流式严格直线（默认，保证路径高度）; 0=规划器路径
    # stream_speed: 流式直线的末端速度 m/s
    # use_line: 规划器路径下是否每段先尝试直线规划
    BOARD_CFG_KEYS = ("drop", "hover", "transit_seg", "grip_settle",
                      "use_line", "use_stream", "stream_speed",
                      "open_width", "close_width")

    def board_config(self, key, value):
        store = self._board_store()
        if key not in self.BOARD_CFG_KEYS:
            raise ValueError(f"config key 必须是 {'/'.join(self.BOARD_CFG_KEYS)}")
        value = float(value)
        if value < 0 or value > 1.0:
            raise ValueError("数值超出合理范围 [0, 1]")
        if key == "use_line":
            # 打开直线开关时重置自适应禁用状态，给直线模式重新尝试的机会
            self._line_fail_streak = 0
            self._line_disabled = False
        store["config"][key] = value
        bt.save_store(store)
        return {"ok": True, "key": key, "value": value, "config": store["config"]}

    def board_home_set(self):
        """把当前末端位姿存为复位点（观棋位），网页/CLI/抓放序列共用"""
        store = self._board_store()
        r = self.pose(store["config"]["arm"])
        if not r.get("ok"):
            return {"ok": False, "stage": "read_pose", "detail": r}
        store["config"]["home_pose"] = r["pose"]
        bt.save_store(store)
        return {"ok": True, "home_pose": [round(v, 4) for v in r["pose"]]}

    def board_home(self, timeout):
        """右臂复位到观棋位：先垂直升到安全高度，再短段平移过去"""
        store = self._board_store()
        cfg = store["config"]
        arm = cfg["arm"]
        home = cfg.get("home_pose")
        if not home or len(home) != 7:
            return {"ok": False, "error": "未设置复位点，请先点「当前位姿设为复位点」"}
        home = [float(v) for v in home]
        timeout = max(15.0, float(timeout or 15))
        leg, transit = self._board_movers(cfg)
        cur = self.pose(arm)
        if not cur.get("ok"):
            return {"ok": False, "stage": "read_pose", "detail": cur}
        safe_z = max(cur["pose"][2], home[2])
        r = leg(arm, [cur["pose"][0], cur["pose"][1], safe_z] + home[3:7], timeout)
        if not r.get("ok"):
            return {"ok": False, "stage": "rise", "detail": r}
        r = transit(arm, home, timeout)
        if not r.get("ok"):
            return {"ok": False, "stage": "transit", "detail": r}
        return {"ok": True, "home_pose": [round(v, 4) for v in home]}

    def board_goto(self, point, hover_only, timeout):
        """升到悬停高度 → 平移到目标点上空 → （可选）下降到抓取高度"""
        store = self._board_store()
        arm = store["config"]["arm"]
        timeout = max(15.0, float(timeout or 15))   # 长平移+规划耗时，超时给足
        dest, source = bt.point_pose(store, point)
        cur = self.pose(arm)
        if not cur.get("ok"):
            return {"ok": False, "stage": "read_pose", "detail": cur}
        hover_z = max(cur["pose"][2], dest[2] + float(store["config"]["hover"]))
        leg, transit = self._board_movers(store["config"])

        legs = [("rise", [cur["pose"][0], cur["pose"][1], hover_z] + dest[3:7], leg),
                ("translate", [dest[0], dest[1], hover_z] + dest[3:7], transit)]
        if not hover_only:
            legs.append(("descend", dest, leg))
        for stage, target, mover in legs:
            r = mover(arm, target, timeout)
            if not r.get("ok"):
                return {"ok": False, "stage": stage, "detail": r}
        return {"ok": True, "point": str(point).lower(), "source": source,
                "hover_only": bool(hover_only), "pose": [round(v, 4) for v in dest]}

    def board_move(self, src, dst, timeout):
        """完整抓放：开爪 → A上空 → 下降 → 闭爪 → 升回悬停 → B上空 →
        降到释放高度（落点上方 drop 米，不下到抓取高度）→ 开爪松手 → 升回"""
        store = self._board_store()
        cfg = store["config"]
        arm, side = cfg["arm"], cfg["side"]
        # 夹爪宽度只认服务端配置（网页「抓放开口/闭合」或 CLI config 修改），
        # 不接受请求参数覆盖，保证单一真值来源
        open_w = float(cfg["open_width"])
        close_w = float(cfg["close_width"])
        settle = float(cfg.get("grip_settle", 0.8))
        hover = float(cfg["hover"])
        drop = min(float(cfg.get("drop", 0.03)), hover)
        timeout = max(15.0, float(timeout or 15))   # 跨棋盘长平移+规划，超时给足
        leg, transit = self._board_movers(cfg)
        a, a_src = bt.point_pose(store, src)
        b, b_src = bt.point_pose(store, dst)

        def lift(p, dz):
            q = list(p)
            q[2] += dz
            return q

        def up(p):
            return lift(p, hover)

        steps = [
            ("gripper_open", lambda: self.gripper(side, open_w), settle),
            (f"{src}@hover", lambda: transit(arm, up(a), timeout), 0),
            (f"{src}@grasp", lambda: leg(arm, a, timeout), 0),
            ("gripper_close", lambda: self.gripper(side, close_w), settle),
            (f"{src}@lift", lambda: leg(arm, up(a), timeout), 0),
            (f"{dst}@hover", lambda: transit(arm, up(b), timeout), 0),
        ]
        if drop < hover - 1e-6:
            # 落子高度 = 全部标定点 z 的平均值 + drop，整盘统一，不随单点示教误差波动
            release = [b[0], b[1], bt.mean_z(store["points"]) + drop] + list(b[3:7])
            steps.append((f"{dst}@drop", lambda: leg(arm, release, timeout), 0))
        steps += [
            ("gripper_release", lambda: self.gripper(side, open_w), settle),
            (f"{dst}@retreat", lambda: leg(arm, up(b), timeout), 0),
        ]
        # 落子后自动复位到观棋位，给相机让出完整棋局视角
        home = cfg.get("home_pose")
        if home and len(home) == 7:
            home = [float(v) for v in home]
            steps.append(("home", lambda: transit(arm, home, timeout), 0))
        done = []
        for stage, fn, sleep_after in steps:
            r = fn()
            done.append(stage)
            if not r.get("ok"):
                return {"ok": False, "stage": stage, "detail": r, "done": done,
                        "hint": f"序列在「{stage}」这一步失败并中止（前面已完成: {' → '.join(done[:-1])}）。"
                                "棋子可能停在半路，请手动 jog / 重新抓放恢复；"
                                "若 detail 里是 FAULT/超时，可加大动作超时后重试。"}
            if sleep_after:
                time.sleep(sleep_after)
        return {"ok": True, "src": str(src).lower(), "dst": str(dst).lower(),
                "src_source": a_src, "dst_source": b_src, "steps": done}

    @staticmethod
    def _check_arm(arm):
        if arm not in ("left_arm", "right_arm"):
            raise ValueError("arm must be left_arm/right_arm")

    @staticmethod
    def _camera_sensor(name):
        if SensorType is None:
            raise RuntimeError("SensorType is not available from SDK")
        mapping = {
            "head_left": SensorType.HEAD_LEFT_CAMERA,
            "left_arm": SensorType.LEFT_ARM_CAMERA,
            "right_arm": SensorType.RIGHT_ARM_CAMERA,
        }
        if name not in mapping:
            raise ValueError("camera must be head_left/left_arm/right_arm")
        return mapping[name]

    @staticmethod
    def _motion_ok(status):
        return gm is not None and status == gm.MotionStatus.SUCCESS

    @staticmethod
    def _motion_command_ok(status, low_latency=False):
        if gm is None:
            return False
        if status == gm.MotionStatus.SUCCESS:
            return True
        return bool(low_latency) and status == gm.MotionStatus.IN_PROGRESS

    def _ensure_body_anchor(self, current):
        if self.body_anchor is None:
            self.body_anchor = list(current[:5])
        return self.body_anchor

    def _body_delta(self, positions):
        if self.body_anchor is None or len(positions) < 3:
            return 0.0
        deltas = [float(positions[i]) - float(self.body_anchor[i]) for i in range(3)]
        return sum(deltas) / len(deltas)

    @staticmethod
    def _body_positions_from_delta(delta, current, anchor):
        if not isinstance(current, list) or len(current) < 5:
            raise ValueError("current leg positions must contain 5 values")
        if not isinstance(anchor, list) or len(anchor) < 5:
            raise ValueError("anchor leg positions must contain 5 values")
        delta = max(-BODY_MAX_DELTA, min(float(delta), BODY_MAX_DELTA))
        positions = [float(v) for v in current[:5]]
        for idx in range(3):
            positions[idx] = float(anchor[idx]) + delta
        return positions

    @staticmethod
    def _list(value):
        return [float(x) if isinstance(x, (int, float)) or hasattr(x, "__float__") else x for x in list(value)]


controller = RobotController()


class WHDHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/" or parsed.path.startswith("/index.html"):
            self._send(200, HTML, "text/html; charset=utf-8")
            return
        if parsed.path == "/stream":
            q = parse_qs(parsed.query)
            self._stream(q.get("camera", ["head_left"])[0])
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def _stream(self, camera):
        if camera not in CAMERA_FRAMES:
            self._send_json(404, {"ok": False, "error": "unknown camera"})
            return
        slot = CAMERA_FRAMES[camera]
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = -1
        try:
            while True:
                jpeg, last = slot.wait_newer(last, timeout=2.0)
                if jpeg is None:
                    if not controller.initialized:
                        break
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        try:
            data = self._read_json()
            path = urlparse(self.path).path
            if path == "/api/init":
                result = controller.init()
            elif path == "/api/status":
                result = controller.status()
            elif path == "/api/shutdown":
                result = controller.shutdown()
            elif path == "/api/pose":
                result = controller.pose(data.get("arm"))
            elif path == "/api/head_pose":
                result = controller.head_pose()
            elif path == "/api/head_set":
                result = controller.head_set(data.get("positions", [0, 0]), data.get("timeout", 8))
            elif path == "/api/head_move":
                result = controller.head_move(data.get("index", 0), data.get("dir", 1),
                                              data.get("step", 0.02), data.get("timeout", 8))
            elif path == "/api/body_pose":
                result = controller.body_pose()
            elif path == "/api/body_move":
                result = controller.body_move(data.get("dir", 1), data.get("step", 0.05), data.get("timeout", 8))
            elif path == "/api/body_preset":
                result = controller.body_preset(data.get("name", "high"), data.get("timeout", 8))
            elif path == "/api/motion_zero":
                result = controller.motion_zero(data.get("timeout", 15))
            elif path == "/api/move":
                result = controller.move_relative(
                    data.get("arm"), data.get("axis"), data.get("dir", 1),
                    data.get("step", 0.03), data.get("timeout", 8),
                    data.get("enable_collision_check", False), data.get("low_latency", False))
            elif path == "/api/move_pose":
                result = controller.move_pose(
                    data.get("arm"), data.get("target_pose"), data.get("timeout", 8),
                    data.get("enable_collision_check", False), data.get("move_line", False))
            elif path == "/api/gripper":
                result = controller.gripper(data.get("side"), data.get("width", 0.05))
            elif path == "/api/gripper_state":
                result = controller.gripper_state(data.get("side"))
            elif path == "/api/preset":
                result = controller.preset(data.get("name", "ready"), data.get("timeout", 10))
            elif path == "/api/stop":
                result = controller.stop()
            elif path == "/api/hold_stop":
                result = controller.hold_stop(data.get("arm"))
            elif path == "/api/jog":
                result = controller.jog_command(data.get("arm"), data.get("axis"),
                                                data.get("dir", 1), data.get("speed", 0.1))
            elif path == "/api/jog_stop":
                result = controller.jog_stop(data.get("arm"))
            elif path == "/api/board_state":
                result = controller.board_state()
            elif path == "/api/board_record":
                result = controller.board_record(data.get("point"))
            elif path == "/api/board_remove":
                result = controller.board_remove(data.get("point"))
            elif path == "/api/board_goto":
                result = controller.board_goto(data.get("point"), data.get("hover_only", True),
                                               data.get("timeout", 15))
            elif path == "/api/board_config":
                result = controller.board_config(data.get("key"), data.get("value"))
            elif path == "/api/board_home":
                result = controller.board_home(data.get("timeout", 15))
            elif path == "/api/board_home_set":
                result = controller.board_home_set()
            elif path == "/api/board_move":
                result = controller.board_move(data.get("src"), data.get("dst"),
                                               data.get("timeout", 15))
            else:
                self._send_json(404, {"ok": False, "error": "not found"})
                return
            self._send_json(200, result)
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})

    def log_message(self, fmt, *args):
        pass

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _send_json(self, code, payload):
        self._send(code, json.dumps(payload, ensure_ascii=False, indent=2), "application/json; charset=utf-8")

    def _send(self, code, body, content_type):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _die(signum, _frame):
    # 与 wrist_camera_server.py 相同：不做任何 SDK 清理（request_shutdown 等
    # 调用会在 C++ 层阻塞且持有 GIL，把解释器冻死），直接硬退，
    # daemon 线程和 SDK 连接随进程一起被 OS 回收
    print(f"\nStopping (signal {signum})...", flush=True)
    controller.initialized = False
    sys.stdout.flush()
    os._exit(0)


def _install_exit_handlers():
    """robot.init() 后 SDK 会在 C 层接管 SIGINT（有时还把它从线程信号掩码里
    屏蔽掉），Ctrl+C 到不了 Python。这里重新注册处理器并解除主线程的屏蔽，
    两步都做才能保证信号真正送达 _die。只能在主线程调用。"""
    signal.signal(signal.SIGINT, _die)
    signal.signal(signal.SIGTERM, _die)
    try:
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})
    except (AttributeError, ValueError, OSError):
        pass


def main():
    print("WHD dual-arm web controller (live streams)")
    print("SDK_ROOT:", SDK_ROOT)
    if SDK_IMPORT_ERROR:
        print("SDK import error:", SDK_IMPORT_ERROR)
        print("Tip: source the SDK setup.sh before running this script.")
    # auto-init 里 robot.init 可能阻塞数秒，先装好处理器保证这期间 Ctrl+C 有效
    _install_exit_handlers()
    # 自动初始化 SDK（失败不阻止服务器启动，可在网页上再手动点「初始化 SDK」重试）
    if os.environ.get("WHD_AUTO_INIT", "1") != "0":
        print("Auto-initializing SDK ...")
        try:
            result = controller.init()
            print("Auto-init result:", result)
        except Exception as exc:
            print("Auto-init failed:", repr(exc))
            print("Tip: source the SDK setup.sh, or click 初始化 SDK on the web page.")
        _install_exit_handlers()  # init 会覆盖信号处理器，装回来
    print(f"Open http://0.0.0.0:{PORT}")
    server = WHDHTTPServer(("0.0.0.0", PORT), Handler)

    # HTTP 服务放到 daemon 线程；主线程只负责睡眠 + 周期性夺回信号处理器
    # （网页上后点「初始化 SDK」会再次覆盖 SIGINT，signal.signal 只能在
    # 主线程调用，没法在 /api/init 里装，所以这里每秒重装一次兜底）
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        while True:
            _install_exit_handlers()
            time.sleep(1.0)
    except KeyboardInterrupt:
        _die(signal.SIGINT, None)


if __name__ == "__main__":
    main()
