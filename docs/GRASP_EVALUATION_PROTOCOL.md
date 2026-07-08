# 机械臂抓取稳定性评测协议 (docs/GRASP_EVALUATION_PROTOCOL.md)

**文档版本**：v1.0  
**状态**：Released  
**关联里程碑**：RM-M4 / RM-SPEC-04  

本协议规定了 `ros2-moveit-pybullet-bridge` 平台在进行闭环策略评测时所采用的物理评估指标、抓取参数扫描（Sweep）方法及真机接入的准入成功率阈值。

---

## 1. 抓取与稳定性物理指标 (Evaluation Metrics)

系统在每一次抓取试验（Trial）中，均会采集并计算以下 10 维物理指标，用以评估抓取策略的表现：

| 指标 (Metric) | 单位 | 说明 | 判定依据 |
|---|---|---|---|
| `approach_pose_error_m` | 米 (m) | 夹爪 TCP 到目标期望抓取点的 3D 距离偏差 | 计算轨迹规划点与真值的末端差距 |
| `approach_orientation_error_rad` | 弧度 (rad) | 夹爪 TCP 姿态与目标期望姿态的角度偏差 | 旋转矩阵角差 |
| `gripper_closing_time_ms` | 毫秒 (ms) | 夹爪从发出 close 命令到检测到接触或闭紧的耗时 | 耗时过长说明卡爪或驱动器通信时滞 |
| `contact_detected` | 布尔 (bool) | 是否与目标物体建立物理接触 | 触觉感知或力矩传感器突变 |
| `lift_height_m` | 米 (m) | 抓取后抬升阶段，物体被拉离地面的垂直高度 | 物体真值 Z 坐标变化量 |
| `hold_duration_sec` | 秒 (s) | 抓取完成后保持不落的时长 | 默认测试窗口为 3.0s |
| `slip_distance_m` | 米 (m) | 运送与抬升期间物体相对于夹爪 TCP 的滑移距离 | 触觉滑移值或状态插值差值 |
| `ee_object_distance_m` | 米 (m) | 试验结束时，末端 TCP 与物体几何中心的相对距离 | 用于检验物体是否滑出或处于边缘 |
| `success` | 布尔 (bool) | 本次抓取试验是否成功 | 必须同时满足接触建立、抬升达标且无脱落 |
| `failure_reason` | 枚举 (enum) | 本次试验失败的详细归因 | 参照 §4 失败分类 |

---

## 2. 测试参数扫描计划 (Parametric Sweep)

为了测试抓取算法在不同物理边界条件下的鲁棒性，评估系统支持对以下参数进行自动/手动网格扫描：

1. **物体质量 (Object Mass)**: 扫描范围 `[0.05, 0.5] kg`，步长 0.05kg。验证阻抗与力控的自适应能力。
2. **物体摩擦系数 (Object Friction)**: 仿真中扫描指尖与物体接触面的 Lateral Friction 范围 `[0.2, 1.2]`。
3. **夹爪握力 (Gripper Force / Command)**: 扫描夹爪的最大输出力矩限制，验证轻握力与最大握力的边缘。
4. **接近速度 (Approach Speed)**: 扫描接近目标物瞬间的末端线速度 `[0.01, 0.15] m/s`。
5. **抬升速度 (Lift Speed)**: 扫描抬升瞬间的垂直向速度 `[0.02, 0.20] m/s`，评估重力反作用冲击下的脱落率。
6. **抓取偏置 (Grasp Pose Offset)**: 注入 $\pm 5\text{mm}$ 的位置扰动与 $\pm 0.05\text{rad}$ 的角度扰动，评估视觉定位误差容忍度。
7. **保持时长 (Hold Duration)**: 扫描 `[1.0, 10.0] s` 窗口下的滑移累积。

---

## 3. 真机接入准入标准 (Acceptance Criteria)

机械臂策略在仿真预验收阶段，必须满足以下统计学门槛方允许申请实机部署：

* **样本数量**: 每一组特定扫描参数下，必须重复进行至少 **20 次 Trial**，保证统计学置信度。
* **准入成功率**: 针对基准物体（例如重量 0.2kg、摩擦系数 0.8 的方块）在名义无偏置轨迹下，**成功率必须 $\ge 80\%$**。
* **掉落熔断限值**: 若发生物体掉落（`object_dropped`）的次数在单组测试中 $> 2$ 次，系统评测强制标记为 `FAIL`，拒绝准入，必须将对应 Trial 的 rosbag 路径和监控曲线导出交由算法研发进行优化。

---

## 4. Grasp 失败归因枚举列表 (Failure Reason Enum)

当 `success` 为 False 时，必须将 `failure_reason` 归类为以下之一：

* **`none`**: 成功（未失败）。
* **`missed_object`**: 夹爪运动到位，但未包络或碰触目标物体。
* **`no_contact`**: 夹爪已执行闭紧动作，但未检测到与物体的物理接触压力。
* **`insufficient_closure`**: 夹爪因受到外部摩擦阻碍提前卡死，未闭合至目标开度且反馈力矩不足。
* **`object_slipped`**: 抬升或运送过程中，物体因夹爪握力不够或运行抖动在指尖产生滑移超出阈值（滑移距 $> 1.5\text{cm}$）。
* **`object_dropped`**: 在抬升至目标高度或保持阶段，物体完全脱落，重新落到桌面上或掉出视野。
* **`planning_failed`**: MoveIt 运动规划失败，机械臂未开始执行动作。
* **`trajectory_timeout`**: 轨迹执行由于速度限制、奇异点减速或网络延迟导致超时。
* **`safety_stop`**: 触发了 `risk_engine` / `safety_monitor` 熔断或物理急停导致运动被强行切断。
* **`sensor_missing`**: FT 传感器或图像采集断流，导致判定数据缺失。
* **`unknown`**: 无法分类的其他突发硬件/仿真器故障。
