# 下游仓库架构定位与实机部署指南 (Sim2Real Deployment Guide)

在完整的作品集中，下游仓库 `ros2-moveit-pybullet-bridge` 承担着**部署验证、双源对比与在线监控**的收口职责。本文档明确了下游仓库的核心价值、后续的性能优化点，以及从虚拟仿真向**真机物理系统（Real Robot）**迁移时的关键注意事项与硬件安全性规范。

---

## 一、 下游仓库究竟起到了什么作用？

下游仓库绝非简单的“仿真回放工具”，其核心价值在于以下四个层面：

```
                        [上游遥操作 / 中游训练策略]
                                  │
                                  ▼ (Policy Actions)
                      ┌──────────────────────┐
                      │   PolicyRunner 节点   │
                      └──────────┬───────────┘
                                 │
            ┌────────────────────┴────────────────────┐
            ▼ (Sim Reference)                         ▼ (Real Feedback)
    ┌───────────────┐                         ┌───────────────┐
    │ PyBullet 仿真  ├────────┐       ┌───────┤   真实机器人   │
    └───────────────┘        │       │       └───────────────┘
                             ▼       ▼
                      ┌──────────────────────┐
                      │   dist_monitor 节点  │ <--- KL / MMD 偏差标定
                      └──────────┬───────────┘
                                 │
                                 ▼ (Drift Metrics / Risks)
                      ┌──────────────────────┐
                      │   risk_engine 节点   │ <--- 五维风险监测 & 降级保护
                      └──────────┬───────────┘
                                 │
                                 ▼ (WebSocket)
                      ┌──────────────────────┐
                      │   HOC 运维诊断控制台   │
                      └──────────────────────┘
```

1. **Sim2Real 效果的在线测试沙盒 (Policy Runner)**
   策略在 episode-data-lab 训练完毕后，其推理泛化性能不能直接交付真机。下游仓库提供了一个全闭环的仿真验证环境，能够在不损伤实机的情况下，直接压测策略（ACT/Diffusion）在接触动力学、碰撞规避和关节限位上的健壮性。
2. **异构物理数据的分布偏差标定 (MMD / KL 散度)**
   `dist_monitor` 实时订阅仿真真值与真实反馈（仿真中由 `vcan0` 或 `pybullet` 模拟），以滑窗形式计算最大均值差异（MMD）和相对熵（KL 散度）。在实机部署中，这能第一时间捕捉到由于光照改变、背景杂乱、机械磨损或负载突变引起的“协变量偏移 (Covariate Shift)”，避免策略在未知场景下做出失控动作。
3. **独立于控制环的主动防御系统 (Risk Engine)**
   在策略输出 Action 之后、实机执行之前，`risk_engine` 对关节超限、速度突变、碰撞边界、消息失联（心跳）及运行帧率进行并行检测，提供软件层面的 Fail-Safe（ Estop / Quick Stop 挂起），构建一道不依赖策略网络的最后安全防线。
4. **人机协同运维的可视化控制台 (HOC Console)**
   为数据采集工程师和现场运维人员提供 Web 端交互面板，通过 WebSocket 实时显示偏差状态与安全级，提供远程 E-Stop 拦截。

---

## 二、 现有系统有哪些优化点？

在进行大规模实机回放或高频（> 100Hz）闭环控制时，下游仓库有以下明确的优化方向：

### 1. 策略推理延迟优化（由 Python 转向 C++ / TensorRT）
* **现状**：目前 `PolicyRunner` 采用 Python 编写，利用 CPU 推理，导致 `replay` 策略在第一帧加载或进行多模态大模型前向推理时延迟突增（Max ~444ms）。
* **优化方案**：在实机部署时，需将训练出的 PyTorch 权重转化为 ONNX / TensorRT 引擎，并使用 C++ 编写 ROS 2 推理节点。这能将前向推理时延降低至 2ms 以下，满足 500Hz 控制环的需求。

### 2. DDS 通信与共享内存优化 (Shared Memory)
* **现状**：目前多节点（Runner ➡️ Monitor ➡️ Simulator）基于默认的 ROS 2 DDS 网络回环通信。虽然在 localhost 上延迟较小（< 2ms），但在多机部署或局域网环境下可能会有严重的抖动。
* **优化方案**：启用 FastDDS 或 CycloneDDS 的 **Shared Memory (SHM) 共享内存传输机制**。对于大体量数据（如 L6 感知层图像和高频关节状态），绕过内核网络栈，实现零拷贝（Zero-Copy）数据传输。

### 3. 碰撞体网格简化 (Collision Mesh Simplification)
* **现状**：MoveIt 规划和 PyBullet 检测使用的 URDF 直接加载了精细的视觉网格（Visual Mesh）作为碰撞网格（Collision Mesh），增加了仿真解算器的负担。
* **优化方案**：在 URDF 中将碰撞体替换为简化的凸包（Convex Hull）或包围盒（Primitive Shapes: Boxes, Cylinders），将单步碰撞检测开销缩减至微秒级。

---

## 三、 真机接入注意事项 (Sim2Real Safety & Calibration Checklist)

将下游系统由 PyBullet 物理仿真后端接入真实机械臂（例如通过 Franka Control Interface / FCI 或 KUKA Fast Robot Interface / FRI）时，必须严格遵守以下注意事项以防发生**撞机事故**：

### 1. 硬件级 E-Stop 与软件 Quick Stop 联动 (Hardwired Safety)
* **致命风险**：下游的 `risk_engine` 触发安全警报后，发送的是软件层面的控制指令拦截（Quick Stop 帧）。如果网络发生拥堵或实机驱动故障，软件指令无法到达伺服驱动器。
* **规范要求**：
  - 必须将 ROS 2 侧安全节点（如 `safety_monitor` 或 `risk_engine`）的数字量输出口（DO）或工业总线故障状态，通过硬件安全网关（如安全 PLC / 安全继电器）接入机器人的**物理控制柜安全回路 (Estop Circuit)**。
  - 一旦软件检测到严重的碰撞风险或看门狗超时，必须直接切断伺服主接触器（Cat. 0/1 停机），而非仅发送零速度指令。

### 2. 传感器噪声与漂移阈值重标定 (MMD/KL Recalibration)
* **现状/风险**：仿真的传感器数据（图像、关节角）是绝对干净的。实机部署时，相机的环境光变化、镜头畸变、电机的齿轮间隙（Backlash）以及传感器高频白噪声，会使得 MMD 和 KL 散度指标天然偏高。如果直接套用仿真的阈值，会导致系统频繁误触发安全保护并中断运行。
* **规范要求**：
  - 在正式运行策略前，进行“零动作对齐测试”与“示教轨迹标定”。
  - 记录机器人在正常无偏差状态下，真实反馈与基准状态之间的 MMD/KL 自然本底噪声，并在此基础上设置 $3\sigma$ 的动态安全阈值。

### 3. 关节空间奇异点与控制增益调谐 (Impedance Control Tuning)
* **安全风险**：阻抗控制器（`cartesian_impedance_controller`）的刚度和阻尼系数（$K_p$, $K_d$）在仿真中可以设得很大且不发生发散；在真实物理实体中，过高的刚度会引起关节高频共振，导致电机过热或保护关断，过低的刚度则会导致抓取力不足、控制发散或机械臂松垮下坠。
* **规范要求**：
  - 在切换至真机时，刚度系数必须**从极小值开始缓慢调大**（如先设为仿真的 $10\%$），并使用实时示波器观察关节电流与扭矩波动，确保系统处于过阻尼或临界阻阻尼状态，严禁发生震荡。
  - 在 MoveIt Servo 运动层中，必须激活奇异点阻尼减速机制（Singularity Avoidance），避免机械臂进入万向节锁死状态（Gimbal Lock），防止关节突发大扭矩运动。

### 4. 严苛的通信质量服务策略 (QoS Configuration)
* **网络风险**：实机运行通常采用有线网络连接主控机与机器人控制器。网络抖动或丢包会导致 `ros2_control` 周期中断。
* **规范要求**：
  - 关节状态（`/joint_states`）与控制指令的话题必须配置为 **Best Effort** 的可靠性策略（只保实时性，丢帧不补发，避免指令积压造成“追赶效应”）。
  - 安全心跳与 E-Stop 信号必须配置为 **Reliable** 和 **Transient Local**，确保所有节点均能实时且准确地收到安全状态变更。

---

## 四、跨机器部署补充规范（设计规划，非已实现）

> [!NOTE]
> 以下内容为从单机仿真向真实双机部署迁移时的技术规划，当前项目仅在单机仿真环境中完成验证，尚未接入真实 Franka Panda 硬件。

---

### 1. Franka Control Interface（FCI）网络与实时内核要求

真实 Panda 机械臂通过 **FCI（Franka Control Interface）** 与控制 PC 通信，对网络和操作系统有严格的实时性要求。

#### 网络配置

```
控制 PC（运行 ROS2 节点）
    ├── eth0 → 局域网 / 办公网络（普通流量）
    └── eth1 → 192.168.1.x（直连 Panda 控制柜，专用网口）
                         ↓ 1Gbps 直连网线（不经过交换机）
              Panda 控制柜（固定 IP: 192.168.1.1）
```

- Panda FCI 要求控制 PC 与控制柜之间的**往返延迟 < 1ms**
- 必须使用**专用网口直连**，不能共用办公网络，否则网络抖动会触发 FCI 通信超时保护（机械臂自动制动）
- 关闭控制 PC 上 eth1 的网卡节能（Energy Efficient Ethernet）：
  ```bash
  sudo ethtool -s eth1 speed 1000 duplex full autoneg off
  sudo ethtool --set-eee eth1 eee off
  ```

#### 实时内核（RT Kernel）要求

FCI 要求控制 PC 运行 **PREEMPT-RT 实时内核**，保证控制周期抖动 < 100μs：

```bash
# 验证当前内核是否为实时内核
uname -r
# 期望输出包含 "-rt" 或 "PREEMPT_RT"，例如：
# 5.15.0-76-generic-rt

# 检查实时调度权限
ulimit -r   # 应 > 0

# 设置 ROS2 控制节点的实时优先级（需 root 或 CAP_SYS_NICE）
sudo chrt -f 80 ros2 run controller_manager ros2_control_node
```

> [!WARNING]
> 不使用实时内核直接运行 FCI 会导致控制周期超时，Panda 控制柜会频繁触发 `communication_constraints_violation` 错误并强制停机。

---

### 2. vcan0 → 真实 CAN 总线切换

当前项目的 `canopen_hw_interface` 通过 `use_sim` 参数区分仿真和真机模式：

```cpp
// canopen_system.cpp — 参数控制仿真/真机分支
use_sim_ = (use_sim_text == "true");

if (use_sim_) {
    // 仿真：订阅 /sim/encoder_state，发布 /sim/joint_effort_cmd
} else {
    // 真机：open_can_socket() 连接真实 CAN 总线
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
}
```

#### 仿真 → 真机的切换步骤

**第一步：配置真实 CAN 总线接口**

```bash
# 加载 CAN 内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb    # USB-CAN 适配器（如 PEAK PCAN-USB）

# 启动真实 CAN 接口（替代 vcan0）
sudo ip link set can0 type can bitrate 1000000   # 1Mbit/s（CANopen 标准）
sudo ip link set can0 up

# 验证接口在线
ip link show can0
candump can0   # 应能看到驱动器心跳帧 701~707
```

**第二步：修改 launch 参数**

```bash
# 仿真模式（当前默认）
ros2 launch teleop_bringup full_system.launch.py use_sim:=true  can_interface:=vcan0

# 真机模式
ros2 launch teleop_bringup full_system.launch.py use_sim:=false can_interface:=can0
```

**第三步：验证 DS402 状态机上电序列**

真机启动时 `canopen_hw_interface` 会依次执行：

```
NMT Start → Switch On Disabled → Ready to Switch On → Switched On → Operation Enabled
```

通过 `candump can0` 观察各节点（COB-ID `0x701`~`0x707`）的心跳状态码应为 `0x05`（Operational），否则检查电源和节点 ID 配置。

> [!CAUTION]
> 真机首次上电前必须确认：急停按钮处于可触发状态、机械臂周围无人、阻抗控制刚度系数已降至仿真值的 10%。

---

### 3. 跨机器 ROS_DOMAIN_ID 与 DDS 配置

单机仿真时所有节点在同一台机器上，DDS 自动使用共享内存传输。真机部署时，控制 PC 和 Panda 控制柜（或上位机）是两台机器，DDS 退回 UDP 组播通信，需要统一以下配置。

#### ROS_DOMAIN_ID 统一

```bash
# 两台机器必须使用相同的 Domain ID，且在同一局域网子网
# 控制 PC（~/.bashrc 或 /etc/environment）
export ROS_DOMAIN_ID=42

# Panda 控制柜上位机（同样配置）
export ROS_DOMAIN_ID=42
```

Domain ID 决定 DDS 使用的 UDP 组播地址（`239.255.0.{domain_id}`），不同 Domain 的节点完全隔离，看不到彼此的话题。

#### 组播路由验证

```bash
# 验证两台机器的组播包能互达
# 在控制 PC 上
ping 239.255.0.42

# 检查网络接口是否支持组播
ip link show eth1 | grep MULTICAST

# 如果交换机过滤了组播，可切换为单播发现（FastDDS XML 配置）
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/unicast_discovery.xml
```

`unicast_discovery.xml` 示例（避免组播被网络设备过滤）：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.1.100</address>  <!-- 控制 PC IP -->
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>192.168.1.1</address>   <!-- Panda 控制柜 IP -->
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

#### 跨机器 QoS 注意事项

DDS 切换到 UDP 后，之前依赖共享内存的性能优势消失，需要重新评估：

| 话题 | 单机延迟 | 跨机器 UDP 延迟 | 处理方式 |
|------|---------|----------------|---------|
| `/joint_states` @ 1kHz | ~1μs | ~200μs | 可接受，保持 `BEST_EFFORT` |
| `/sim/encoder_state` @ 1kHz | ~1μs | ~200μs | 可接受，保持 `BEST_EFFORT` |
| `/safety/estop` | ~1μs | ~1ms | 必须 `RELIABLE + TRANSIENT_LOCAL` |
| `/bridge/camera/image` @ 30Hz | ~1ms | ~5~20ms | 考虑压缩或降帧率 |

> [!TIP]
> 跨机器部署时建议用 `ros2 topic delay /joint_states` 实时监测端到端延迟，目标保持 < 50ms（与 M4 验收标准一致）。若延迟持续超标，优先排查网络设备是否有 QoS 限速或组播过滤策略。
