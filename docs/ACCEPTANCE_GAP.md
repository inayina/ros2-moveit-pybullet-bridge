# 验收标准差距台账

**项目**：ros2-moveit-pybullet-bridge  
**更新**：2026-06-20  
**用途**：按 `docs/design/01-system-architecture-and-requirements.md` 的验收标准逐项记录当前实现、证据缺口与收尾取舍，避免把「未复验证据」「范围调整」和「真正未实现」混在一起。

---

## 1. 结论摘要

当前项目已经具备面试 Demo 主线：MoveIt/PyBullet 闭环、双源监控、五维风险、HOC 控制台、报告导出与本机验证脚本。主要差距不是核心代码不可用，而是**验收证据尚未统一沉淀**，尤其是公开 CI、Demo 视频、性能量化指标与早期需求口径变更记录。

| 类别 | 当前判断 | 处理方式 |
|------|----------|----------|
| 核心功能链路 | 基本满足 | 保持 `run_tests.sh`、`verify_portfolio.sh`、`verify_risk_complete.sh` 复验记录 |
| 公开交付证据 | 有本机证据，公开证据不足 | Push 后补 GitHub Actions 绿勾与 Demo 视频链接 |
| 性能/可靠性量化 | 多数未形成可引用报告 | 增加 P0/P1 量测任务，不为面试前扩代码 |
| 早期设计口径偏差 | 存在 `UR5/FR3`、完整 `ros2_control`、真机 Real 等偏差 | 写入范围调整或 Phase-2+，不再按当前里程碑阻塞 |

---

## 2. 状态定义

| 状态 | 含义 |
|------|------|
| 已满足 | 代码与脚本已覆盖，且已有本机或样例证据 |
| 待复验 | 功能存在，但缺少最新、可引用的验收记录 |
| 证据缺口 | 需要补量测、截图、视频、CI 或报告，不一定需要改代码 |
| 范围调整 | 早期验收标准与当前作品集边界不一致，需在说明中显式改口径 |
| Phase-2+ | 有价值但不阻塞当前面试 Demo / 作品集验收 |

---

## 3. 功能需求差距

### 3.1 桥接通信（FR-BRG）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-BRG-01 | 指令延迟 P99 < 20ms | 证据缺口 | 尚未看到独立 latency report；P1 增加 rosbag 时间戳量测或脚本输出 |
| FR-BRG-02 | 100Hz 发布，频率抖动 < 5% | 待复验 | `verify_portfolio.sh` 可证明链路，需补 `ros2 topic hz` 或脚本化摘要 |
| FR-BRG-03 | 至少验证 UR5/FR3 之一 | 范围调整 | 当前主线已改为 KUKA iiwa7；在说明中以 iiwa7 作为统一平台，UR5/FR3 不再作为当前验收项 |
| FR-BRG-04 | `DIRECT` / `GUI` launch 参数切换 | 已满足 | README 与 launch 流程已有 `sim_mode:=DIRECT/GUI` |
| FR-BRG-05 | 双 PyBullet 实例同步驱动，时间戳对齐 | 已满足 | `portfolio_demo`、双源样例报告与同任务校准报告覆盖 |

### 3.2 MoveIt2 规划闭环（FR-MOV）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-MOV-01 | 标准场景规划成功率 > 95% | 证据缺口 | 可演示 Plan & Execute，但未形成成功率统计；P1 录制成功链路并补小样本统计 |
| FR-MOV-02 | 通过 `ros2_control` 执行到 PyBullet，RMSE < 0.05 rad | 范围调整 | 当前以 `FollowJointTrajectory` relay 替代完整 `ros2_control`；RMSE 可量测，但完整硬件接口归 Phase-2+ |
| FR-MOV-03 | 完整 TF 树供 RViz2 可视化 | 待复验 | RViz 录屏已有，建议补 `tf2_tools` 或截图证据 |
| FR-MOV-04 | 碰撞场景规划被拒绝 | 证据缺口 | MoveIt 碰撞检测能力存在于配置层，但缺少显式验收场景与记录 |

### 3.3 分布监控（FR-MON）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-MON-01 | KL 滑窗 5s，10Hz 更新 | 待复验 | `dist_monitor` 与样例报告覆盖指标，需补频率量测摘要 |
| FR-MON-02 | MMD 统计量 + 置换检验 p-value | 已满足 | 指标链路与报告中已有 MMD 输出 |
| FR-MON-03 | YAML 阈值热加载 / 可配置阈值 | 待复验 | S5 已补阈值热更新能力；HOC Settings UI 不作为当前阻塞 |
| FR-MON-04 | 分布指标时序支持 rosbag2 回放分析 | 待复验 | HOC 报告与样例数据已落地；rosbag2 回放路径需要单独复验记录 |
| FR-MON-05 | 注入 ±20% 阻尼，检出率 > 90% | 证据缺口 | 已有 `inject_shift` 与报告，但缺少多轮检出率统计；P1/P2 用固定 seed 批跑沉淀 |

### 3.4 风险管理（FR-RSK）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-RSK-01 | 五维风险 R0-R3，等级变化延迟 < 500ms | 待复验 | 五维风险已实现；延迟指标未单独量测 |
| FR-RSK-02 | R3 自动急停，关节速度 < 100ms 归零 | 待复验 | 急停链路和 `verify_risk_complete.sh` 已覆盖；需补速度归零时延证据 |
| FR-RSK-03 | 等级变化附带 `primary_driver` | 已满足 | `risk_engine` 与 HOC 归因展示已覆盖 |
| FR-RSK-04 | HOC 确认后恢复运行 | 已满足 | HOC acknowledge / risk acknowledge 链路已覆盖 |

### 3.5 人机控制台（FR-HOC）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-HOC-01 | 风险等级与五维雷达，WS 延迟 < 200ms | 待复验 | UI 与 WebSocket 已有；延迟未单独量测 |
| FR-HOC-02 | 分布对比可视化，支持暂停/缩放 | 待复验 | 图表与截图已有；交互项需录屏或手测记录 |
| FR-HOC-03 | 急停、暂停、恢复，响应 < 100ms | 待复验 | 急停/恢复已覆盖；暂停语义与响应量测需确认 |
| FR-HOC-04 | 实验参数在线调节，下一周期生效 | 待复验 | 随机化/偏移服务已有；完整 Settings 面板不作为当前阻塞 |
| FR-HOC-05 | JSON/CSV 报告，含风险时序 + 分布指标 | 已满足 | S5 已补 CSV，样例 HTML/JSON 报告已入库 |

---

## 4. 非功能需求差距

### 4.1 性能（NFR-P）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-P01 | 控制回路 P95 < 50ms | 证据缺口 | 需要 rosbag 时间戳差分报告 |
| NFR-P02 | 关节状态 100Hz ± 5% | 待复验 | 与 FR-BRG-02 合并量测 |
| NFR-P03 | 分布指标 10Hz | 待复验 | 与 FR-MON-01 合并量测 |
| NFR-P04 | HOC 刷新率 ≥ 5Hz | 待复验 | WebSocket 推送频率可脚本化记录 |
| NFR-P05 | 双实例 240Hz，实时因子 ≥ 0.8 | 证据缺口 | 需要 DIRECT 与 GUI 分开记录，GUI 不作为 CI 阻塞 |

### 4.2 可靠性（NFR-R）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-R01 | 桥接节点崩溃触发安全停机 | Phase-2+ | 当前有看门狗/HOLD 与急停链路；跨进程 supervisor 不作为面试 Demo 阻塞 |
| NFR-R02 | 节点重启恢复上次安全状态 | Phase-2+ | 状态快照持久化未做，已列入未来扩展 |
| NFR-R03 | 连续 2 小时无内存泄漏 | 证据缺口 | 尚无长稳报告；面试前可做 15–30 分钟 smoke，2 小时留后续 |
| NFR-R04 | rosbag 独立录制不中断 | 待复验 | HOC rosbag/report 链路存在；需补异常场景记录 |

### 4.3 安全（NFR-S）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-S01 | 任意时刻急停，速度归零 | 待复验 | 功能已覆盖，缺少定量归零延迟 |
| NFR-S02 | 95% URDF limits 触发 R2 | 已满足 | S5 软限位已补；保留脚本复验记录 |
| NFR-S03 | 指令中断 > 500ms 自动 HOLD | 已满足 | S5 看门狗 HOLD 已补；`verify_risk_complete.sh` 复验 |
| NFR-S04 | R2 自动 50% 降速 | 已满足 | S5 R2 降级已补；保留墙钟对比记录 |
| NFR-S05 | R3 后 HOC 确认 + 自检恢复 | 待复验 | 确认链路已覆盖；自检通过条件需在 Demo 口径中说明 |

### 4.4 可维护性与可复现性（NFR-M / NFR-REP）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-M01 | YAML 配置，支持动态调参 | 待复验 | 核心 YAML 已有；所有节点全覆盖与动态参数需分项确认 |
| NFR-M02 | 一键 launch 完整系统 | 范围调整 | `portfolio_demo` 不自动启动浏览器 HOC；组合入口用 `hoc_experiment.launch.py` |
| NFR-M03 | ROS2 标准包结构，`colcon build` 可编译 | 已满足 | CI / 本机构建记录覆盖 |
| NFR-M04 | 单元测试覆盖率 > 70% | 证据缺口 | 当前有测试脚本但未产出 coverage 报告 |
| NFR-REP-01 | 随机过程可配置 seed | 已满足 | README / 报告 / launch 配置已覆盖 |
| NFR-REP-02 | 实验配置与结果关联存储 | 待复验 | 报告含 git hash/metadata；rosbag 与 YAML 关联需确认 |
| NFR-REP-03 | 标准 Demo 脚本输出确定性基准 | 已满足 | `verify_portfolio.sh`、双仓脚本与样例报告覆盖 |

---

## 5. 当前 P0/P1 差距清单

| 优先级 | 差距 | 验收完成标志 |
|--------|------|--------------|
| P0 | GitHub Actions 最新提交公开绿勾 | README badge / Actions 页面显示 latest run 通过 |
| P0 | 5–8 分钟完整 Demo 视频 | README 或 Release 中有视频链接，覆盖启动、偏移、R2/R3、导出 |
| P0 | 本机复验记录统一 | `run_tests.sh`、`verify_portfolio.sh`、`verify_risk_complete.sh` 的日期与结果写入 `PORTFOLIO_REMAINING.md` |
| P1 | 性能量测摘要 | 至少记录 topic hz、HOC WS 刷新率、急停响应、R2 降速墙钟对比 |
| P1 | MoveIt 成功链路证据 | RViz Plan & Execute 录屏或截图，附成功/失败样本说明 |
| P1 | 注入检出率统计 | 固定 seed 批跑 N 次，输出检出率表或 JSON |
| P1 | 覆盖率报告 | `pytest --cov` 或等价报告，说明未达 70% 时的原因 |

---

## 6. Phase-2+ 不阻塞项

以下项属于未来迁移或增强，不作为当前面试 Demo / 作品集验收阻塞：

| 项 | 当前口径 |
|----|----------|
| 真机 `real_source:=ros2` | 未来接真机驱动时实现；当前双源 PyBullet 与 LeRobot replay 替代 |
| 完整 `ros2_control` 硬件接口 | 当前 relay 已满足演示闭环；完整控制栈归后续工程化 |
| episode-data-lab `Ros2Robot` HAL | 双仓联动加分项，不阻塞 bridge 当前交付 |
| `/clock` + `use_sim_time` 全链路 | 严格 rosbag 同步增强，当前不阻塞 Demo |
| 2 小时长稳 / valgrind | 交付硬化项，面试前以短时 smoke 和 CI 证明为主 |
| HOC Settings 完整阈值 UI | 服务/配置能力优先，UI 打磨后续补 |

---

## 7. 更新规则

1. 验收脚本或报告新增后，先更新本文件对应行的状态与证据。
2. 若某项不再属于当前里程碑，必须改为「范围调整」或「Phase-2+」，并在 `README.md` / `PORTFOLIO_REMAINING.md` 保持同一口径。
3. 不把“代码已实现”直接等同于“验收已完成”；缺少可引用记录时仍标为「待复验」或「证据缺口」。

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [design/01-system-architecture-and-requirements.md](design/01-system-architecture-and-requirements.md) | 原始功能 / 非功能验收标准 |
| [PORTFOLIO_REMAINING.md](PORTFOLIO_REMAINING.md) | 面试 Demo 收尾任务与复验命令 |
| [design/09-risk-monitoring-completion-spec.md](design/09-risk-monitoring-completion-spec.md) | S5 风险补全历史缺口与完成定义 |
| [portfolio/SYSTEM_DESIGN_SPEC.md](portfolio/SYSTEM_DESIGN_SPEC.md) | 交付视角说明书与已知限制 |
| [samples/README.md](samples/README.md) | 样例报告与指标产物索引 |
