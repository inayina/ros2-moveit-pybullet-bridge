# 文档索引

本目录包含设计规格、集成指南、实验报告与作品集导出材料。

## 快速导航

| 你想… | 从这里开始 |
|--------|------------|
| 快速掌握整个项目 | [PROJECT_LEARNING_GUIDE.md](./PROJECT_LEARNING_GUIDE.md) |
| 安装与 Launch 参数 | [SETUP.md](./SETUP.md) |
| 与 episode-data-lab 联调 | [INTEGRATION.md](./INTEGRATION.md) |
| 跑实验、看报告 | [EXPERIMENTS.md](./EXPERIMENTS.md) |
| 对照验收标准查差距 | [ACCEPTANCE_GAP.md](./ACCEPTANCE_GAP.md) |
| 阅读系统架构 | [design/README.md](./design/README.md) |
| 整理作品集材料 | [portfolio/README.md](./portfolio/README.md) |
| 录制或讲解 Demo | [portfolio/DEMO_SCRIPT.md](./portfolio/DEMO_SCRIPT.md) |
| 查看验收摘要 | [portfolio/ACCEPTANCE_SUMMARY.md](./portfolio/ACCEPTANCE_SUMMARY.md) |
| 写简历项目描述 | [portfolio/RESUME_SUMMARY.md](./portfolio/RESUME_SUMMARY.md) |
| 查看 HTML 报告与 JSON 产物 | [samples/README.md](./samples/README.md) |
| 查看图表资源 | [assets/README.md](./assets/README.md) |

## 目录结构

```
docs/
├── README.md              ← 本页（总索引）
├── PROJECT_LEARNING_GUIDE.md 项目学习路线、模块地图、代码阅读顺序
├── SETUP.md               安装、编译、Launch、阈值配置
├── INTEGRATION.md         双仓库环境变量与联调步骤
├── EXPERIMENTS.md         实验流水线、指标解读、报告对照
├── ACCEPTANCE_GAP.md      验收标准差距台账
├── PORTFOLIO_REMAINING.md 待办跟踪
├── design/                01–09 技术设计 Spec
├── portfolio/             作品集入口、Demo 脚本、验收摘要、代码导览、简历描述、系统设计说明书 + Marp 幻灯片
├── samples/               HTML 报告、JSON 指标、NPZ 轨迹
└── assets/                PNG/GIF/SVG 配图
```

## 实验报告一览

| 报告 | 类型 | 说明 |
|------|------|------|
| [samples/dual-repo-experiment-report.html](./samples/dual-repo-experiment-report.html) | 跨仓库联调 | Sim（bridge）vs Real（LeRobot），含局限说明 |
| [samples/same-task-calibration-report.html](./samples/same-task-calibration-report.html) | 同任务校准 | 双源同 JointTrajectory，KL/W1/MMD 可解释 |
| [samples/dual-repo-integration-report.html](./samples/dual-repo-integration-report.html) | 验收摘要 | 指标表 + 嵌入图表 |
| [samples/sample-experiment-report.html](./samples/sample-experiment-report.html) | HOC 样例 | 五维风险 + dashboard 截图 |

## 一键脚本（`scripts/`）

| 脚本 | 用途 |
|------|------|
| `run_dual_repo_integration.sh` | 双仓库联调：校验 → 负对照 → 跨源对比 → 报告 |
| `run_same_task_calibration.sh` | 同任务校准：双源采集 → baseline 分割对比 → 报告 |
| `regenerate_all_reports.py` | **对齐**：从 JSON/NPZ 重生成图 1–9 + 三份 HTML |
| `capture_readme_assets.sh` | **P1-1**：README 真实配图（m3 / m2-iiwa-* / m5） |
| `capture_pick_lift_asset.py` | **P1-2**：从 episode-data-lab 成功抓取 episode 生成 `m6-pick-and-lift.gif` |
| `run_integration_demo.sh` | 本地冒烟 / 可选采集 |
| `run_tests.sh` | 单元 + 节点 + 集成测试 |
| `verify_bridge_comm.sh` | `FR-BRG-01/02`：bridge 延迟、100Hz 频率与 jitter 验收 |
| `verify_moveit_closure.sh` | `FR-MOV-01..04`：MoveIt 成功率、执行 RMSE、TF 与碰撞拒绝验收 |
| `verify_monitor_metrics.sh` | `FR-MON-01..05`：分布指标频率、MMD、阈值热更新、rosbag 与注入检出率 |
| `verify_risk_management.sh` | `FR-RSK-01..04`：R0-R3 延迟、R3 急停、归因与 acknowledge 互锁 |
| `verify_hoc_console.sh` | `FR-HOC-01..05`：HOC WebSocket 刷新/延迟、控制命令、在线参数与 JSON/CSV 导出 |
| `verify_performance_nfr.sh` | `NFR-P01..05`：控制回路 P95、100Hz/10Hz/5Hz 刷新、双源 240Hz 与实时因子 |
| `verify_reliability_nfr.sh` | `NFR-R01..04`：watchdog HOLD、reset 恢复、短时 smoke/RSS、HOC 独立 rosbag |
| `verify_safety_nfr.sh` | `NFR-S01..05`：急停速度归零、软限位 R2、watchdog HOLD、R2 降速、ack 恢复 |
| `verify_maintainability_nfr.sh` | `NFR-M/REP`：YAML/launch/package 结构、动态配置、coverage、确定性脚本产物 |
| `verify_portfolio.sh` | URDF + offline_compare + 15s demo |

详见 [EXPERIMENTS.md](./EXPERIMENTS.md)。
