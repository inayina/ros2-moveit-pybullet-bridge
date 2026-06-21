# 五仓统一架构总图

**项目**：机器人系统集成与验证平台  
**更新**：2026-06-21  
**用途**：作品集主图、README、幻灯片、面试白板  

> 图源文件：本页 Mermaid 可直接粘贴到 GitHub README；导出 PNG 见文末命令。

---

## 1. 总览（五仓 + 协议）

```mermaid
flowchart TB
  subgraph L0["L0 · 展示层"]
    DF["Dashboard Frontend<br/>纯 HTML/JS"]
    HOC["HOC 控制台<br/>React + ECharts"]
    RViz["RViz2 / MoveIt"]
  end

  subgraph L1["L1 · 聚合 / 桥接层"]
    DAPI["robot-ops-dashboard<br/>FastAPI · WS /ws/status · MQTT cache"]
    HS["hoc_server<br/>ROS ↔ WebSocket :8765"]
  end

  subgraph L2["L2 · 集成协议"]
    HTTP["HTTP REST"]
    WS["WebSocket"]
    MQTT["MQTT"]
    ROS["ROS 2 DDS"]
  end

  subgraph AMR["AMR 子系统 · amr_warehouse_navigation"]
    WMS["Mock WMS API"]
    NAV["Nav2 + Gazebo Harmonic"]
    WMS --> NAV
  end

  subgraph EDGE["边缘 / 孪生子系统 · ros2-robot-digital-twin"]
    MCU["STM32 + MPU6050"]
    ESP["ESP32-S3 micro-ROS"]
    MB["Motor / Encoder bench"]
    MCU --> ESP
    ESP --> MB
  end

  subgraph DATA["数据子系统 · robot-arm-episode-data-lab"]
    FSM["任务 FSM · pick-lift"]
    LR["LeRobot v2.1 export"]
    FSM --> LR
  end

  subgraph BRIDGE["操作臂 / Sim2Real · ros2-moveit-pybullet-bridge"]
    MG["MoveIt 2 move_group"]
    PB["pybullet_bridge<br/>Sim / Real 双源 PyBullet"]
    DM["dist_monitor · KL/W1/MMD"]
    RE["risk_engine · R0–R3"]
    PR["PolicyRunner · BasePolicy"]
    MG --> PB
    PB --> DM --> RE
    PR --> PB
  end

  DF <-->|"REST + WS"| DAPI
  HOC <-->|"WS"| HS
  RViz --> MG

  DAPI <-->|"任务 proxy"| HTTP
  HTTP <--> WMS
  DAPI <-->|"状态 / 命令"| MQTT
  MQTT <--> ESP
  HS <-->|"topic / service"| ROS
  ROS <--> PB
  ROS <--> ESP
  ROS <--> NAV

  LR -->|"real_source:=lerobot<br/>LEROBOT_EXPORT"| PB
  RE -->|"R3 e_stop"| PB
  RE --> HS
  DM --> HS
  DAPI -.->|"Evaluation 只读 JSON"| DF

  classDef portal fill:#e3f2fd,stroke:#1976D2,stroke-width:2px
  classDef edge fill:#e8f5e9,stroke:#4CAF50
  classDef manip fill:#fff3e0,stroke:#FF9800
  classDef data fill:#fff9c4,stroke:#FBC02D
  class DAPI,DF portal
  class ESP,MCU,MB edge
  class PB,DM,RE,HOC,HS manip
  class LR,FSM data
```

**读图口诀**

1. **左半（Dashboard 生态）**：浏览器 → FastAPI → HTTP/MQTT → AMR 或边缘 bench  
2. **右半（Bridge 生态）**：MoveIt/Policy → PyBullet 双源 → 监控/风险 → HOC  
3. **数据仓**自底向上喂 LeRobot 给 Bridge，不经过 Dashboard  
4. **Dashboard 不直连 ROS 2**；Bridge 不替代 Nav2  

---

## 2. Dashboard 三条数据链（细节）

```mermaid
flowchart LR
  subgraph Chain1["① AMR / WMS 任务链"]
    direction TB
    C1A["Dashboard UI"] --> C1B["POST /api/wms/tasks"]
    C1B --> C1C["Mock WMS HTTP"]
    C1C --> C1D["Nav2 NavigateToPose"]
    C1D --> C1E["Gazebo AMR"]
    C1E --> C1F["状态回写 → Dashboard"]
  end

  subgraph Chain2["② IMU / 遥测链（只读）"]
    direction TB
    C2A["MPU6050"] --> C2B["ESP32 micro-ROS"]
    C2B --> C2C["ROS 2 topics"]
    C2C --> C2D["MQTT robot/imu · robot/state"]
    C2D --> C2E["GET /api/robot/status<br/>WS /ws/status"]
  end

  subgraph Chain3["③ Motor bench（受限命令）"]
    direction TB
    C3A["Dashboard UI"] --> C3B["POST /api/robot/motor/cmd"]
    C3B --> C3C["MQTT robot/motor/cmd"]
    C3C --> C3D["ROS /motor/cmd → bench"]
    C3D --> C3E["MQTT robot/motor/status"]
    C3E --> C3F["Motor 曲线卡片"]
  end
```

---

## 3. Bridge 操作臂闭环（细节）

```mermaid
flowchart TB
  subgraph In["输入"]
    MGT["MoveIt Plan & Execute"]
    POL["PolicyRunner<br/>Replay / SineWave"]
  end

  subgraph Exec["执行 · pybullet_bridge"]
    CMD["/bridge/command"]
    SIM["Sim-Source PyBullet"]
    REAL["Real-Source<br/>域随机化 / LeRobot"]
    CMD --> SIM
    CMD --> REAL
  end

  subgraph Obs["观测"]
    JS["/joint_states → MoveIt / TF"]
    DJS["/bridge/sim|real/joint_states"]
    SIM --> JS
    SIM --> DJS
    REAL --> DJS
  end

  subgraph Saf["监控 · 风险"]
    MON["dist_monitor<br/>KL · W1 · MMD"]
    RSK["risk_engine R0–R3"]
    DJS --> MON --> RSK
  end

  subgraph Out["运维"]
    HOC2["HOC WebSocket<br/>急停 · 注入 · 报告"]
    RPT["HTML / JSON / CSV / rosbag"]
    RSK --> HOC2
    MON --> HOC2
    HOC2 --> RPT
  end

  MGT --> CMD
  POL --> CMD
  RSK -->|"R3 e_stop"| CMD
  LR2["LeRobot export"] -.-> REAL
```

---

## 4. 仓库 ↔ 层级对照

| 仓库 | 架构层 | 对外协议 | Demo 入口 |
|------|--------|----------|-----------|
| `robot-ops-dashboard` | L0–L1 主展示 | REST · WS · MQTT | `uvicorn` + 静态前端 |
| `amr_warehouse_navigation` | AMR 执行 | HTTP WMS · ROS Nav2 | Mock WMS + Gazebo |
| `ros2-robot-digital-twin` | 边缘 L4 | micro-ROS · MQTT | IMU / motor bench |
| `robot-arm-episode-data-lab` | 离线数据 | 文件 LeRobot | `batch_collect` + export |
| `ros2-moveit-pybullet-bridge` | 臂验证 L4–L5 | ROS 2 · HOC WS | `portfolio_demo` + HOC |

---

## 5. 边界（主图配套口径）

| 组件 | 是 | 不是 |
|------|-----|------|
| Dashboard | 运维驾驶舱、任务/遥测聚合 | Nav2 控制台、底盘 PID |
| MQTT motor cmd | 低频 bench 探针 | 完整运动控制平面 |
| Evaluation 层 | mock/baseline/reserved 验收展示 | VLA/RL 训练结果 |
| Bridge Real-Source | 双 PyBullet / LeRobot 回放 | 真机驱动（Phase-2+） |
| MoveIt 路径 | FollowJointTrajectory relay | 完整 ros2_control HW 接口 |

---

## 6. 导出 PNG（可选）

```bash
# 需安装 @mermaid-js/mermaid-cli
npm install -g @mmdc/mermaid-cli

cd docs/portfolio
mmdc -i unified-architecture-overview.mmd -o ../assets/unified-architecture-overview.png -b transparent
```

也可从 [Mermaid Live Editor](https://mermaid.live) 粘贴 §1 代码导出 SVG/PNG，保存为：

- bridge：`docs/assets/unified-architecture-overview.png`
- dashboard：`artifacts/screenshots/unified-architecture-overview.png`

---

## 7. 相关文档

- [MASTER_PORTFOLIO_PLAN.md](./MASTER_PORTFOLIO_PLAN.md)
- [INTERVIEW_PREP.md](./INTERVIEW_PREP.md)
- dashboard：[portfolio_demo_summary.md](https://github.com/inayina/robot-ops-dashboard/blob/main/docs/portfolio_demo_summary.md)
