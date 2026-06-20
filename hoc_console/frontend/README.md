# HOC WebSocket troubleshooting

## 正确启动顺序

WebSocket 由 **`hoc_server`** 提供，不会随 `portfolio_demo` 自动启动。

```bash
# 终端 1：仿真 + 监控
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI

# 终端 2：HOC（同时起 hoc_server + Vite 前端）
ros2 launch hoc_console hoc.launch.py
```

浏览器打开 **http://\<机器 IP\>:5173**（不要只开前端、不启 hoc_server）。

## 连接方式

| 模式 | 页面地址 | WebSocket |
|------|----------|-----------|
| 开发 `hoc.launch.py` | `:5173` | 经 Vite 代理 `ws://<host>:5173/hoc-ws` → 后端 `:8765` |
| 生产 `hoc_prod.launch.py` | `:8080` | 同端口 `ws://<host>:8080/hoc-ws` |
| 仅后端 | — | `ws://<host>:8765` |

前端已改为使用 **页面 hostname**，不再硬编码 `localhost`（远程/SSH 浏览器时 `localhost` 会连到本机而非 ROS 机器）。

## 常见原因

1. **未启动 hoc_server** — 页面显示「WS 断开」，8765 无监听  
   ```bash
   ss -tlnp | grep 8765
   ```

2. **只启动了 HOC，未启动仿真/监控** — 实验能点开始，但雷达图/时序曲线无数据  
   需要同时运行 `portfolio_demo`（或一键脚本）：
   ```bash
   ./scripts/start_hoc_experiment.sh
   ```
   或分两个终端：
   ```bash
   ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=DIRECT
   ros2 launch hoc_console hoc.launch.py
   ```

3. **端口被旧进程占用** — 日志出现 `Cannot bind WebSocket port 8765` 或 `Port 5173 is already in use`  
   ```bash
   pkill -f "/hoc_console/hoc_server"
   pkill -f "vite --host"
   ss -tlnp | grep -E ':5173|:8765'
   ```

4. **监控仍在预热** — 分布面板显示「样本不足」，需等待 `dist_monitor` 收集约 50 个对齐样本（约 30–60 秒）后 KL/W1/MMD 才会非零

5. **缺少 Python 依赖** — 日志出现 `websockets not installed`  
   ```bash
   python3 -m pip install websockets aiohttp --break-system-packages
   ```

3. **8765 端口被占用** — 重复启动了 hoc_server  
   ```bash
   pkill -f hoc_server
   ros2 launch hoc_console hoc.launch.py
   ```

4. **只转发 5173 未转发 8765** — 开发模式已用 Vite `/hoc-ws` 代理，一般只需开放 **5173**；若直连 8765 需同时转发该端口。

5. **只开了 `npm run dev` 没开 hoc_server** — 代理目标 8765 不存在，WS 一直重连。

## 自检

```bash
# 1. 启动 hoc_server
ros2 run hoc_console hoc_server --ros-args -p serve_frontend:=false

# 2. 另开终端测试握手
python3 - <<'PY'
import asyncio, websockets
async def t():
    async with websockets.connect('ws://127.0.0.1:8765') as ws:
        print(await ws.recv())
asyncio.run(t())
PY
# 期望: {"type": "connected", ...}
```

或使用仓库脚本：`./scripts/verify_hoc.sh`

## 自定义地址

前端构建时可设置：

```bash
VITE_WS_URL=ws://192.168.1.10:8765 npm run build
```
