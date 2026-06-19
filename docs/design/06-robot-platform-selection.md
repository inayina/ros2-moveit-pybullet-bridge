# 06 · 机器人平台选型评估（KUKA iiwa 统一 vs 多机型并存）

**文档版本**：v0.1  
**依赖**：[01 · 系统架构与需求](./01-system-architecture-and-requirements.md)、[03 · 分布监控算法详设](./03-distribution-monitoring-algorithm.md)  
**关联仓库**：
- `ros2-moveit-pybullet-bridge`（本仓库）
- `robot-arm-episode-data-lab`（`/home/ina/robot-sim-lab/robot-arm-episode-data-lab`）

---

## 1. 背景与问题

当前两个仓库使用的机器人平台不一致，导致 **M4 分布监控** 与 **LeRobot 数据集** 联动时需要额外处理维度与语义映射：

| 仓库 | 当前默认机型 | DOF | URDF 来源 | 主要用途 |
|------|-------------|-----|-----------|----------|
| `robot-arm-episode-data-lab` | KUKA LBR iiwa（`kuka_iiwa/model.urdf`） | **7** | PyBullet `pybullet_data` | 采集 episode、LeRobot 导出 |
| `ros2-moveit-pybullet-bridge` M1/M3/M4 | 2-DOF 平面臂 | **2** | 自研 `planar_2dof.urdf` | 快速桥接、双源监控、CI |
| `ros2-moveit-pybullet-bridge` M2 | UR5 | **6** | `ur_description` | MoveIt2 规划闭环演示 |

**核心矛盾**：`dist_monitor` 要求 Sim/Real **关节维数一致**；episode-data-lab 已产出 **7 维** `observation.state`，而桥接默认是 **2 维**，无法直接做 KL/MMD 标定。

**待决问题**：是否将全栈统一为 **KUKA iiwa（7-DOF）**？

---

## 2. 评估维度

| 维度 | 权重建议 | 说明 |
|------|----------|------|
| **数据闭环** | 高 | 与 episode-data-lab / LeRobot 是否零映射对接 |
| **M4 监控可用性** | 高 | dist_monitor 能否开箱标定 |
| **MoveIt2 集成** | 中 | M2 规划闭环成熟度、文档、Jazzy 支持 |
| **实现成本** | 中 | 迁移工作量、测试回归范围 |
| **CI / 调试速度** | 中 | 单元测试、无头仿真耗时 |
| **真机迁移** | 低（当前无硬件） | 未来 FRI / 驱动的可扩展性 |
| **教学 / 作品集** | 低 | 演示效果、面试叙事 |

评分：**5 = 最好，1 = 最差**（见 §4 对比表）。

---

## 3. 候选方案

### 方案 A：全栈统一 KUKA iiwa 7-DOF（推荐用于数据 + 监控主线）

**描述**：`pybullet_bridge`、`moveit_config`、`dist_monitor` 默认加载与 episode-data-lab 相同的 `kuka_iiwa/model.urdf`（7 关节）；M2 MoveIt 改用 `lbr-stack` 或 `kuka_lbr_iiwa_moveit_config`。

**优点**
- episode-data-lab 的 `lerobot_export` 可直接作为 Real 基线，**无需关节映射**
- M4 离线 `offline_compare` / 在线 `real_source:=lerobot` 即插即用
- 与数据采集 HAL 的 `state_dim=7` 语义一致
- PyBullet 内置模型，episode-data-lab 已验证 pick-lift 场景

**缺点**
- 7-DOF 比 2-DOF 重：CI 更慢、MMD 计算维度更高（仍 <50ms，已验证 6-DOF）
- 需新建/引入 MoveIt 配置（UR5 现成配置不能复用）
- `pybullet_data` 的 iiwa 与 `lbr-stack` 官方 URDF **版本/网格可能略有差异**，需做一次 FK/关节名对齐
- 2-DOF 平面臂的「极简演示」优势丧失

**工作量粗估**：3–5 人日（URDF 接入桥接 + launch + 基础 MoveIt 配置 + 更新测试）

---

### 方案 B：保持多机型并存（现状）

**描述**：M1/M3/M4 继续用 2-DOF；M2 用 UR5；episode-data-lab 保持 KUKA iiwa；通过映射层或独立标定流程衔接。

**优点**
- M1 验证脚本、CI 保持最快
- M2 继续对齐 ROS 社区 UR5 教程（资料多）
- 无需改动已有里程碑演示

**缺点**
- **与 episode-data-lab 联动始终需要适配层**（DOF、关节名、home 位姿）
- M4 验收要在 2-DOF 上重做一套数据集，或维护映射代码
- 三套 URDF / 关节名，文档与 mental model 负担大

**工作量粗估**：维持成本低，但每次跨仓库集成 +2–4 人日

---

### 方案 C：分层策略 ——「2-DOF 测管道，iiwa 跑集成」（推荐折中）

**描述**：
- **CI / 单元测试 / 节点测试**：保留 2-DOF（`planar_2dof.urdf`），验证话题、KL/MMD 逻辑、risk 链路
- **集成 / 标定 / 作品集主线**：新增 `iiwa7` profile，默认用于 `full_system.launch.py`、M4 标定、与 episode-data-lab 联调
- M2 MoveIt：短期可保留 UR5 演示，长期增加 `m2_iiwa_demo.launch.py`

**优点**
- 测试速度快 + 数据闭环兼得
- 迁移可分阶段，不阻塞现有 M1 CI
- README 可明确「开发用 2-DOF，标定用 iiwa」

**缺点**
- 需维护两套 launch / 参数 profile
- 团队需理解「哪条链路用哪个机型」

**工作量粗估**：4–6 人日（在方案 A 基础上保留双 profile）

---

### 方案 D：全栈改 UR5（6-DOF）

**描述**：episode-data-lab 改为 UR5 采集，桥接侧以现有 M2 为主。

**优点**
- M2 MoveIt 已就绪
- UR 生态文档丰富

**缺点**
- **需重写 episode-data-lab 全部采集场景**（IK、RRT、抓取、20+ episode 数据）
- LeRobot 已导出 7 维 KUKA 数据作废
- 6-DOF 与现 7-DOF 数据集仍不兼容
- **综合成本最高，不推荐**（除非战略上放弃 KUKA 作品集叙事）

**工作量粗估**：10+ 人日

---

## 4. 方案对比评分

| 维度 | A 全 iiwa | B 多机型 | C 分层 iiwa | D 全 UR5 |
|------|-----------|----------|-------------|----------|
| 数据闭环 | **5** | 2 | **5** | 1 |
| M4 监控 | **5** | 2 | **5** | 3 |
| MoveIt 集成 | 3 | 4 | 3 | **5** |
| 实现成本 | 3 | **5** | 3 | 1 |
| CI 速度 | 2 | **5** | 4 | 3 |
| 真机扩展 | 4 | 3 | 4 | 4 |
| 作品集一致 | **5** | 2 | **5** | 3 |
| **加权倾向** | 数据主线首选 | 仅短期省事 | **综合推荐** | 不推荐 |

---

## 5. 关键对齐细节（若选 iiwa）

### 5.1 URDF 来源对照

| 来源 | 路径/包 | 备注 |
|------|---------|------|
| episode-data-lab | `pybullet_data/kuka_iiwa/model.urdf` | 已用于采集与 LeRobot 导出 |
| PyBullet 桥接（建议） | 同上，或复制到 `pybullet_bridge/urdf/kuka_iiwa/` | 避免依赖搜索路径差异 |
| ROS2 MoveIt（建议） | [`lbr-stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack) `jazzy` 分支，`iiwa7` | Jazzy 官方维护活跃；含 mock + move_group |
| 备选 MoveIt | [`kuka_robot_descriptions`](https://github.com/kroshu/kuka_robot_descriptions) `kuka_lbr_iiwa_moveit_config` | 注意文档提到 iiwa 加速度限制可能导致规划失败 |

### 5.2 关节与消息字段

| 字段 | episode-data-lab | 桥接需对齐 |
|------|------------------|------------|
| 关节数 | 7（`state_dim`） | `home_positions` 长度 7 |
| LeRobot 列名 | `observation.state` | `lerobot_loader` 已支持 |
| 关节名 | `joint_0` … `joint_6`（导出） | `JointState.name` 建议一致 |
| 控制频率 | 10 Hz（采集） | 桥接 100 Hz；对齐容差 `align_tolerance_sec≥0.05` |
| 速度 | 无（由 loader 差分估计） | 可选在桥接发布 `velocity` 提升 MMD 质量 |

### 5.3 不宜混淆的机型

- **PyBullet `kuka_iiwa`**：LBR iiwa 7 轴（与 episode-data-lab 一致）
- **KUKA KR 系列（kr6 等）**：不同构型，不可混用
- **UR5 6-DOF**：少 1 关节，运动学完全不同

---

## 6. 建议决策

### 6.1 结论

| 目标 | 建议 |
|------|------|
| **与 episode-data-lab + M4 联动** | **统一 KUKA iiwa 7-DOF**（方案 A 或 C） |
| **日常开发 / CI** | 保留 2-DOF profile（方案 C） |
| **仅做 MoveIt 教程演示** | 可暂时保留 UR5 M2，不作为数据主线 |
| **episode-data-lab 改 UR5** | **不建议**（方案 D） |

**综合推荐：方案 C（分层策略）** —— 用 2-DOF 保 CI 速度，用 iiwa 作为 **数据—监控—标定** 的默认集成机型。

### 6.2 推荐实施顺序

**已实现（Plan C Phase 1）**：

- `robot_profiles.py`：`planar_2dof` | `iiwa7`
- `full_system.launch.py` 默认 `iiwa7`；`m1_demo` 默认 `planar_2dof`
- `portfolio_demo.launch.py` 作品集一键启动
- `pybullet_bridge/urdf/kuka_iiwa/` 与 episode-data-lab 同构 URDF

**已实现（Plan C Phase 2）**：

- `moveit_config/srdf/kuka_iiwa.srdf` — planning group `manipulator`
- `m2_iiwa_demo.launch.py` — MoveIt2 + PyBullet `FollowJointTrajectory` 闭环
- `scripts/check_iiwa_joint_consistency.py` — PyBullet / MoveIt 关节顺序校验

**待办（Phase 3，可选）**：

- 实现 Ros2Robot HAL（见 episode-data-lab `migration_ros2_moveit.md`）
- 实时写入与 LeRobot 导出并存

### 6.3 暂不建议「全部立刻换成 iiwa」的范围

- **单元测试**：`test_sliding_window`、`test_kl_divergence` 等纯算法测试无需改机型
- **M1 `verify_m1.sh`**：可保留 2-DOF 作为 smoke test
- **已有 UR5 M2 文档截图**：保留作历史里程碑，新主线用 iiwa

---

## 7. 决策检查清单

在启动迁移前确认：

- [ ] 目标场景是 **数据闭环 + M4 标定** 还是 **MoveIt 教学演示**？
- [ ] 是否接受维护 **双 robot profile**（方案 C）？
- [ ] MoveIt 选型：`lbr-stack`（mock 简单）还是 `kuka_robot_descriptions`？
- [ ] episode-data-lab 现有 `dataset/v1/lerobot_export`（20 ep × 7-DOF）是否作为标定基准保留？
- [ ] 夹爪维度：`gripper_urdf` 9-DOF 是否纳入监控（当前 M4 仅臂关节 7-DOF）？

---

## 8. 一句话建议

> **若目标是让 `robot-arm-episode-data-lab` 与 `dist_monitor` 顺畅联动，应把集成主线统一到 KUKA iiwa 7-DOF；不必删掉 2-DOF，但应用 iiwa 作为默认标定与 LeRobot 对接机型。**

---

**上一篇**：[05 · ROS2 节点接口与数据流规格](./05-ros2-node-interface-and-dataflow-spec.md)  
**相关**：`robot-arm-episode-data-lab/docs/reference/migration_ros2_moveit.md`
