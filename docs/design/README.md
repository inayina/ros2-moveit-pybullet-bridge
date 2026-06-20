# 技术设计文档索引

**项目**：基于 ROS2 + MoveIt2 + PyBullet 的虚实映射与分布监控系统  
**环境**：ROS 2 Jazzy · Python 3.12 · PyBullet 3.x · MoveIt2 2.9.x

## 文档列表

| 序号 | 文档 | 说明 |
|------|------|------|
| 01 | [系统架构与需求](./01-system-architecture-and-requirements.md) | 技术栈选型、分层架构、功能/非功能需求、无硬件验证方案 |
| 02 | [接口设计](./02-interface-design.md) | ROS 2 话题、服务、Action、自定义消息定义 |
| 03 | [分布监控算法详设](./03-distribution-monitoring-algorithm.md) | KL 散度、MMD、滑窗策略、阈值标定 |
| 04 | [人机运维控制台设计](./04-hoc-console-design.md) | HOC 技术选型、页面线框、交互流程、WebSocket 协议 |
| 05 | [ROS2 节点接口与数据流规格](./05-ros2-node-interface-and-dataflow-spec.md) | PyBullet Clock Sync、Topic/Service/Action、MoveIt2 配置、URDF 物理参数、闭环数据流 |
| 06 | [机器人平台选型评估](./06-robot-platform-selection.md) | KUKA iiwa 统一 vs 多机型；与 episode-data-lab 联动建议 |
| 07 | [作品集系统 Spec 补充](./07-portfolio-system-spec-supplement.md) | **求职总 Spec**：双源 changeDynamics、W1、Action Library、Fail-Safe、Sprint 路线、编码前置清单、面试讲解稿 |
| 08 | [双仓库通盘集成设计](./08-dual-repo-portfolio-integration-spec.md) | **episode-data-lab + bridge 一体架构**、LeRobot 契约、Real 三模式、HAL 映射、统一 Sprint |
| 09 | [风险监控补全实现 Spec](./09-risk-monitoring-completion-spec.md) | **S5a–S5d 已实现**：五维风险、R2 降级、看门狗/CSV/阈值热更新 |

## 阅读顺序

```
01 系统架构与需求
        ↓
08 双仓库通盘集成     ← episode-data-lab 联动时优先读（可与 01 并行）
        ↓
02 接口设计          ← 实现节点前先读
        ↓
05 节点接口与数据流   ← PyBullet 桥接 + MoveIt2 集成详读
        ↓
06 机器人平台选型     ← 与 episode-data-lab 联动前读（iiwa 统一评估）
        ↓
03 分布监控算法详设   ← 实现 dist_monitor 前读
        ↓
04 人机运维控制台设计 ← 实现 hoc_console 前读
        ↓
07 作品集系统 Spec 补充  ← 面试准备 / bridge 编码前总览
        ↓
08 双仓库通盘集成        ← 与 robot-arm-episode-data-lab 联调 / 作品集一体讲解
        ↓
09 风险监控补全 Spec     ← 五维风险 / R2 降级 / NFR 补全编码前读
```

## 包结构对照

```
ros2-moveit-pybullet-bridge/
├── pybullet_bridge/     ← 01, 02
├── dist_monitor/        ← 02, 03
├── risk_engine/         ← 02, 03
├── manipulation_actions/← 02, 07 (Pick/Place)
├── hoc_console/         ← 02, 04
├── moveit_config/       ← 01, 02
└── docs/design/         ← 本目录
```

## 版本记录

| 版本 | 日期 | 变更 |
|------|------|------|
| v0.1 | 2026-06-19 | 初版：架构、接口、算法、HOC 四篇文档 |
| v0.2 | 2026-06-19 | 新增 05：PyBullet Clock Sync、MoveIt2 联合配置、闭环数据流 |
| v0.3 | 2026-06-19 | M5 HOC 实现同步；iiwa 主线配图；样例实验报告 |
| v0.4 | 2026-06-19 | 新增 07：作品集总 Spec（W1、Action Library、Fail-Safe、前置清单） |
| v0.5 | 2026-06-19 | 新增 08：双仓库通盘集成（episode-data-lab + bridge） |
| v0.7 | 2026-06-20 | 09 Spec P4 完成：看门狗 HOLD、CSV 导出、verify_risk_complete |
