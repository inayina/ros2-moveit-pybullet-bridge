# 真实机械臂接入就绪度（Real-Machine Readiness）检查清单

**文档版本**：v1.0
**状态**：Released
**关联里程碑**：RM-M1 / RM-SPEC-02
**用途**：真实机械臂（Franka Emika Panda / KUKA iiwa7 / UR / 定制臂）接入硬件系统前的安全与集成预验收 Checklist。

> [!WARNING]
> 本清单由 `ros2-moveit-pybullet-bridge` 仿真验证平台依据数字孪生及仿真预验收规范制定。
> **当前仓库仅支持完成就绪度的“仿真预验收”与“契约逻辑校验”，无法替代现场物理环境中的真机安全调试与最终安全准入。**

---

## 1. 适用范围 (Scope)

本就绪度清单适用于将控制算法（MoveIt 2 规划轨迹、PolicyRunner 神经网络策略等）从仿真器（PyBullet / MuJoCo）下发至真实物理机械臂前的**系统集成验收**阶段。验收旨在拦截包括控制冲突、坐标跃变、标定漂移、进程失控及电气不匹配在内的物理安全隐患。

---

## 2. 状态定义

- **Pass**：已在目标真实硬件上完全符合要求，且有可追溯现场日志/测量记录。
- **Sim Precheck**：仅代码、单元测试或仿真预检查通过；不能替代 Hardware Pass。
- **Planned**：只有设计或路线图，尚无充分执行证据。
- **Hardware Pending**：必须在真实硬件现场验证；该状态会阻止 Hardware Go。
- **Fail**：不满足要求，必须整改。
- **N/A**：目标系统明确不涉及该项；不能用 N/A 代替尚未执行的硬件验证。
- **Evidence**：引用的数据、配置文件或测试报告路径。

---

## 3. 硬件接口就绪度 (Hardware Interface)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| HW-01 | **控制柜与机械臂供电及接地**：控制柜接地电阻 < 4Ω；电缆无外露破损；漏电保护开关及安全继电器工作正常。 | `Hardware Pending` | 当前无真实硬件现场记录 | Developer | 实机现场复核电气接线 |
| HW-02 | **末端执行器（夹爪/工具）安装**：机械法兰安装螺栓力矩匹配；气路/电路走线留出最大运动裕量，无拉扯风险。 | `Hardware Pending` | 当前无真实硬件现场记录 | Developer | 现场确认实机理线 |
| HW-03 | **传感器安装与稳固**：相机（Wrist / Scene）固定结构件无松动；FT传感器安装无应力集中。 | `Hardware Pending` | 当前无真实硬件现场记录 | Developer | 拧紧标定前力矩螺栓 |
| HW-04 | **急停回路物理连接**：急停按钮（硬件物理 E-Stop）通过安全双通道回路接入控制柜或安全 PLC，按下后能切断马达母线电源。 | `Hardware Pending` | 当前无真实硬件现场记录 | Developer | 现场按下测试下电响应 |

---

## 4. 控制接口就绪度 (Control Interface)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| CT-01 | **硬件驱动与 `ros2_control` 接口**：`SystemInterface` 驱动能正常启动，读写周期 jitter < 1ms，关节 names 匹配 profile。 | `Sim Precheck / Hardware Pending` | [robot_profiles.py](../pybullet_bridge/pybullet_bridge/robot_profiles.py) 仅证明 profile 配置 | Developer | 现场连接 libfranka 测试 |
| CT-02 | **重力补偿与负载标定**：末端工具质心（CoG）、质量（Mass）已写入控制柜；空载与持物状态下的静态重力平衡偏差 < 5N。 | `Sim Precheck / Hardware Pending` | `changeDynamics` 仅证明仿真预检查 | Developer | 现场启动重力自学习程序 |
| CT-03 | **底层 PID 环路对齐**：仿真 PID 控制增益与实机控制柜速度/位置环参数完成数学等效映射，阶跃上升时间无明显超调。 | `Planned / Hardware Pending` | [panda_action_adapter.py](../pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py) 只有参数预留 | Developer | 现场微调伺服刚度系数 |
| CT-04 | **传动与控制死区管理**：标定齿轮箱回程空程死区；在 ROS 2 控制层设置小偏差控制死区（位置偏差 < $10^{-4}$ rad 不输出指令），防止电机蜂鸣。 | `Sim Precheck / Hardware Pending` | [panda_action_adapter.py](../pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py) 仅证明仿真 deadzone 逻辑 | Developer | 实机小幅度扫频校准 |

---

## 5. 夹爪接口就绪度 (Gripper Interface)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| GR-01 | **夹爪控制通信契约**：支持以开度 `[0.0, 0.04]m` 连续或以百分比离散控制，目标指令无静默丢包或超时。 | `Sim Precheck / Hardware Pending` | [panda_handoff.py](../pybullet_bridge/pybullet_bridge/learning/panda_handoff.py) 仅证明静态宽度限制 | Developer | 实机驱动连接测试 |
| GR-02 | **夹爪状态反馈**：能输出当前测量开度与力矩，且支持“抓取完成（Object Grabbed）”信号的无延时反馈。 | `Planned / Hardware Pending` | [test_panda_action_adapter.py](../pybullet_bridge/test/test_panda_action_adapter.py) 不证明真实夹爪反馈 | Developer | 夹爪空闭合测试 |

---

## 6. 传感器与多源融合就绪度 (Sensors & Fusion)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| SN-01 | **异频时间戳对齐**：使用 `ApproximateTime` 同步机制将相机图像（30Hz）、FT数据（100Hz）及关节角（100Hz）的对齐时差控制在 10ms 以内。 | `Planned / Hardware Pending` | [ROADMAP.md](ROADMAP.md) 只有里程碑规范 | Developer | 单元测试模拟异频输入验证 |
| SN-02 | **动态 FT 重力过滤**：融合算法能根据实时加速度前馈，扣除夹爪重力与运动惯性，输出真实末端接触力力矩。 | `Planned / Hardware Pending` | [ROADMAP.md](ROADMAP.md) 只有设计说明 | Developer | 机械臂空载挥舞时校准 FT 零漂 |
| SN-03 | **相机内外参校验**：手眼标定参数导入，通过 5 点法标定验证，在静态场景下末端重建目标位置极差 < 2mm。 | `Planned / Hardware Pending` | [ROADMAP.md](ROADMAP.md) 只有设计说明 | Developer | 运行手眼标定流程 |

---

## 7. 安全防护与熔断就绪度 (Safety Layer)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| SF-01 | **指令 $C^2$ 连续性限幅**：控制层配置速度、加速度及加加速度限制；突变跳变输入能被安全层无条件拦截并触发 Hold。 | `Sim Precheck / Hardware Pending` | [panda_action_adapter.py](../pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py) 仅证明仿真限幅 | Developer | 注入跳变指令进行仿真测试 |
| SF-02 | **抱闸释放流程（Sag Control）**：上电抱闸释放瞬间，算法进入 Servo Hold 锁定重力下移，在稳定 0.5s 后才允许动作。 | `Planned / Hardware Pending` | [ROADMAP.md](ROADMAP.md) 只有设计说明 | Developer | 实机上电抱闸测试 |
| SF-03 | **软件熔断联动（R0-R3）**：当网络断流、关节位置偏差累积或分布偏移超限时，`risk_engine` 能在 100ms 内熔断。 | `Sim Precheck / Hardware Pending` | 仿真 risk/adapter 测试，不证明硬件 E-Stop | Developer | 注入通讯故障验证 E-Stop 切换 |

---

## 8. 网络、时间同步与 DDS (Network & DDS)

| 编号 | 检查项与验收标准 | 状态 | 证据 (Evidence) | 负责人 | 后续行动 (Next Action) |
|---|---|---|---|---|---|
| NT-01 | **DDS 局域网广播隔离**：配置专属 `ROS_DOMAIN_ID`；通过 XML 通信白名单限制，防止其他局域网节点误发控制指令。 | `Planned / Hardware Pending` | [12-real-machine-readiness-spec.md](design/12-real-machine-readiness-spec.md) 只有设计规范 | Developer | 现场排查并绑定本地 IP 网卡 |
| NT-02 | **系统时间同步（NTP/PTP）**：控制端与机器人控制柜通过 PTP 精确对齐，时间偏差 < 50μs。 | `Planned / Hardware Pending` | [12-real-machine-readiness-spec.md](design/12-real-machine-readiness-spec.md) 只有设计规范 | Developer | 实机部署 linuxptp 服务 |

---

## 9. 回退方案 (Fallback Plan)

1. **网络断开/通信亚健康**：触发 `SF-03`，机器人自动刹车减速并转为位置伺服模式，保持原位不倾坠。
2. **策略推理超时/卡顿**：`PolicyRunner` 软件看门狗在超过 500ms 未收到更新时强制切入 `HOLD` 状态。
3. **坐标系或标定失效**：在运行轨迹前进行 5 点法校验，若静态场景漂移超限，系统锁死，必须要求人工在 HOC 控制台进行干预重置。

---

## 10. 准入判决门禁 (Go / No-Go Decision)

所有标注为 **P0**（核心安全项，包括 CT-01、CT-02、SF-01、SF-03、NT-01）的条款必须在真实硬件上达到 **Pass**，方允许系统进行实机首次低速 bring-up 调试。`Sim Precheck`、`Planned` 或 `Hardware Pending` 都不能视为 Pass。

**当前判决：Hardware No-Go。** 当前项目没有真实 Panda 现场验收产物，不得连接真实物理硬件，也不得声称已完成真实 Sim2Real。
