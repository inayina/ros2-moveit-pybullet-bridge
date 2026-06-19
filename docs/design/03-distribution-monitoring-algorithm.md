# 03 · 分布监控算法详设

**文档版本**：v0.1  
**依赖**：[01 · 系统架构与需求](./01-system-architecture-and-requirements.md)、[02 · 接口设计](./02-interface-design.md)  
**实现包**：`dist_monitor`

---

## 1. 问题定义

### 1.1 输入

在时刻 \( t \)，系统维护两个滑动窗口：

- **Sim 窗口** \(\mathcal{W}_s\)：来自 `/bridge/sim/joint_states` 的最近 \(N\) 个样本
- **Real 窗口** \(\mathcal{W}_r\)：来自 `/bridge/real/joint_states` 的最近 \(N\) 个样本

每个样本为：

\[
\mathbf{x}_t = [q_1, \ldots, q_n, \dot{q}_1, \ldots, \dot{q}_n]^T \in \mathbb{R}^{2n}
\]

其中 \(n\) 为关节数。默认使用位置分量 \(\mathbf{q}_t \in \mathbb{R}^n\) 进行 KL 计算，完整 \(\mathbf{x}_t\) 用于 MMD。

### 1.2 输出

每 \(\Delta t = 0.1\) s（10Hz）发布一次 `DistributionMetrics`：

- 逐关节 KL 散度 \(\{D_{KL}^{(i)}\}_{i=1}^n\)
- 联合分布 MMD 统计量及 p-value
- `shift_detected` 布尔判定

### 1.3 设计目标

| 目标 | 指标 |
|------|------|
| 实时性 | 单次计算 < 50ms（6-DOF, N=500） |
| 灵敏度 | 阻尼 +20% 偏移，5s 内检出率 > 90% |
| 特异性 | 无偏移基线运行，误报率 < 5%（10min 窗口） |
| 可解释性 | 输出逐关节 KL，支持定位偏移关节 |

---

## 2. 数据预处理

### 2.1 时间对齐

Sim 与 Real 两路数据来自同一指令驱动，但 Real 侧有延迟噪声。对齐策略：

```
1. 以 Sim 时间戳为基准轴
2. Real 样本按最近邻匹配：|t_real - t_sim| < 20ms
3. 无法匹配的 Real 样本丢弃
4. 对齐后计算误差序列：ε_t = q_sim(t) - q_real(t)
```

**监控对象**：优先对**误差序列** \(\{\varepsilon_t\}\) 建分布，而非各自绝对位置。理由：两实例接收相同指令，绝对轨迹相近，误差分布对物理参数差异更敏感。

### 2.2 滑窗管理

```python
class SlidingWindow:
    """固定时长滑窗，默认 5s @ 100Hz → 最多 500 样本"""

    def __init__(self, duration_sec: float, max_freq_hz: float):
        self.duration = duration_sec
        self.max_samples = int(duration_sec * max_freq_hz)

    def push(self, timestamp: float, value: np.ndarray) -> None:
        ...

    def get_samples(self) -> np.ndarray:
        """返回 shape (N, D) 的数组"""
        ...

    def count(self) -> int:
        ...
```

**冷启动**：样本数 < `min_samples`（默认 50）时，输出 `shift_detected=false`，指标置零。

### 2.3 直方图分箱（KL 用）

对每一关节误差序列 \(\{\varepsilon^{(i)}_t\}\)：

1. 合并 Sim 对齐误差（即 \(\varepsilon = q_s - q_r\)）作为单一样本集
2. 用 **Freedman-Diaconis 规则**自动确定 bin 数：

\[
h = \frac{2 \cdot \mathrm{IQR}}{\sqrt[3]{N}}, \quad k = \lceil (\max - \min) / h \rceil
\]

3. 限制 \(k \in [10, 50]\)，避免稀疏或过密
4. 添加 Laplace 平滑：\(\hat{p}_i = \frac{c_i + \alpha}{N + \alpha k}\)，\(\alpha = 10^{-6}\)

---

## 3. KL 散度

### 3.1 定义

对离散分布 \(P\)（基线）和 \(Q\)（当前窗口），：

\[
D_{KL}(P \| Q) = \sum_i P(i) \log \frac{P(i)}{Q(i)}
\]

### 3.2 基线策略

| 模式 | 说明 | 适用 |
|------|------|------|
| **静态基线** | 系统启动后前 30s 无随机化采集为 \(P\) | 稳定实验环境 |
| **自适应基线** | \(P\) 为上一窗口的 Real 误差分布（随机游走基线） | 渐变漂移检测 |
| **Sim 参考基线** | \(P\)=零均值高斯（\(\sigma\) 从初始无扰动标定） | 绝对偏差检测 |

**默认**：静态基线。服务 `/monitor/reset_baseline` 可重新采集。

### 3.3 双窗口 KL 计算流程

本系统采用**误差分布对比**变体：

```
基线 P：无扰动阶段（或 reset 后）的 ε 分布直方图
当前 Q：当前滑窗内的 ε 分布直方图
```

```python
def kl_divergence(p: np.ndarray, q: np.ndarray) -> float:
    """
    p, q: 归一化概率分布（同长度直方图）
    """
    p = np.clip(p, 1e-10, None)
    q = np.clip(q, 1e-10, None)
    return float(np.sum(p * np.log(p / q)))
```

### 3.4 逐关节与聚合

\[
D_{KL}^{mean} = \frac{1}{n} \sum_{i=1}^{n} D_{KL}^{(i)}
\]

**判定**：

```
shift_detected_kl = (kl_divergence_mean > kl_threshold_mean)
```

默认 `kl_threshold_mean = 0.15`（需通过标定实验确认，见 §6）。

---

## 4. MMD（Maximum Mean Discrepancy）

### 4.1 定义

给定核函数 \(k(\cdot, \cdot)\)，两样本集 \(X = \{\mathbf{x}_i\}_{i=1}^m\)，\(Y = \{\mathbf{y}_j\}_{j=1}^n\)：

\[
\widehat{\mathrm{MMD}}^2 = \frac{1}{m^2}\sum_{i,i'} k(\mathbf{x}_i, \mathbf{x}_{i'}) + \frac{1}{n^2}\sum_{j,j'} k(\mathbf{y}_j, \mathbf{y}_{j'}) - \frac{2}{mn}\sum_{i,j} k(\mathbf{x}_i, \mathbf{y}_j)
\]

### 4.2 RBF 核

\[
k(\mathbf{x}, \mathbf{y}) = \exp\left(-\frac{\|\mathbf{x} - \mathbf{y}\|^2}{2\gamma^2}\right)
\]

**带宽 \(\gamma\) 选择**：

```
median heuristic: γ = median({||x_i - x_j|| : i < j}) / sqrt(2)
```

参数 `mmd_gamma` 可手动覆盖；默认 `1.0`，启动时用前 100 个样本自动估计。

### 4.3 实现

```python
def mmd_rbf(X: np.ndarray, Y: np.ndarray, gamma: float) -> float:
    """
    X: (m, D), Y: (n, D)
    返回 MMD 统计量（非负）
    """
    XX = rbf_kernel(X, X, gamma)
    YY = rbf_kernel(Y, Y, gamma)
    XY = rbf_kernel(X, Y, gamma)
    m, n = len(X), len(Y)
    mmd2 = XX.sum() / (m * m) + YY.sum() / (n * n) - 2 * XY.sum() / (m * n)
    return float(max(mmd2, 0.0))


def rbf_kernel(A: np.ndarray, B: np.ndarray, gamma: float) -> np.ndarray:
    sq_dists = np.sum(A**2, axis=1, keepdims=True) + \
               np.sum(B**2, axis=1) - 2 * A @ B.T
    return np.exp(-sq_dists / (2 * gamma**2))
```

### 4.4 置换检验（p-value）

\(H_0\)：\(X\) 与 \(Y\) 来自同一分布。

```
算法：
1. 合并 Z = X ∪ Y，计算 observed_mmd = MMD(X, Y)
2. 重复 B 次（默认 B=100）：
   a. 随机打乱 Z 的标签
   b. 拆分为 X', Y'（保持原 m, n）
   c. 计算 perm_mmd[b] = MMD(X', Y')
3. p_value = (count(perm_mmd >= observed_mmd) + 1) / (B + 1)
```

**判定**：

```
shift_detected_mmd = (p_value < 0.05) AND (mmd_statistic > mmd_threshold)
```

### 4.5 X 与 Y 的语义

| 集合 | 来源 | 含义 |
|------|------|------|
| \(X\) | Sim 窗口的 \(\mathbf{x}_t = [q, \dot{q}]\) | 理想仿真状态分布 |
| \(Y\) | Real 窗口的 \(\mathbf{x}_t\) | 扰动仿真状态分布 |

与 KL（基于误差）不同，MMD 直接比较两路**绝对状态联合分布**，对多维耦合偏移（如阻尼+摩擦同时变化）更敏感。

---

## 5. 综合判定逻辑

```python
def detect_shift(kl_mean, mmd_stat, mmd_p, cfg) -> tuple[bool, str]:
    kl_flag = cfg.use_kl and (kl_mean > cfg.kl_threshold_mean)
    mmd_flag = cfg.use_mmd and (mmd_p < 0.05) and (mmd_stat > cfg.mmd_threshold)

    if kl_flag and mmd_flag:
        return True, "both"
    if kl_flag:
        return True, "kl"
    if mmd_flag:
        return True, "mmd"
    return False, "none"
```

**风险引擎对接**：`distribution_shift` 维度得分：

\[
s_{D1} = \mathrm{clip}\left(\frac{1}{2}\left(\frac{D_{KL}^{mean}}{D_{KL}^{max}} + \frac{\mathrm{MMD}}{\mathrm{MMD}_{max}}\right), 0, 1\right)
\]

其中 \(D_{KL}^{max}\)、\(\mathrm{MMD}_{max}\) 来自标定实验（§6）。

---

## 6. 阈值标定流程

### 6.1 标定实验设计

```
Phase A（基线采集）：
  - randomization_strength = 0.0
  - 运行 SC-01 场景 3 次，每次 60s
  - 记录 KL_mean、MMD 的 [mean, std, P95, P99]

Phase B（灵敏度标定）：
  - 分别注入 +10%, +20%, +30% 单参数偏移
  - 每种偏移运行 10 次
  - 记录检出延迟 MTTD、检出率 Recall

Phase C（阈值确定）：
  - kl_threshold_mean = Phase A 的 P99 × 1.2
  - mmd_threshold = Phase A 的 P99 × 1.2
  - 验证 Phase B +20% 偏移 Recall > 90%
```

### 6.2 默认初始阈值（标定前占位）

| 参数 | 初始值 | 说明 |
|------|--------|------|
| `kl_threshold_mean` | 0.15 | 基于 6-DOF 经验值，需标定替换 |
| `mmd_threshold` | 0.05 | 同上 |
| `mmd_p_value_alpha` | 0.05 | 统计显著性水平 |
| `window_duration_sec` | 5.0 | 平衡灵敏度与稳定性 |
| `min_samples` | 50 | 冷启动保护 |

### 6.3 阈值配置文件

```yaml
# dist_monitor/config/thresholds.yaml
thresholds:
  kl_threshold_mean: 0.15
  mmd_threshold: 0.05
  mmd_p_value_alpha: 0.05

calibration:
  baseline_kl_p99: null      # 标定后填入
  baseline_mmd_p99: null
  calibration_date: null
  calibration_scenario: "SC-01"
```

---

## 7. 节点实现架构

```
dist_monitor_node
├── subscribers
│   ├── /bridge/sim/joint_states
│   └── /bridge/real/joint_states
├── core
│   ├── TimeAligner          # 时间戳对齐
│   ├── SlidingWindow (×2)   # Sim / Real 缓冲
│   ├── ErrorWindow          # 误差序列缓冲
│   ├── KLDivergence         # 逐关节 KL
│   ├── MMDTest              # MMD + 置换检验
│   └── ShiftDetector        # 综合判定
├── publishers
│   ├── /monitor/distribution_metrics
│   └── /monitor/tracking_error
└── services
    ├── /monitor/set_thresholds
    └── /monitor/reset_baseline
```

### 7.1 主循环伪代码

```python
class DistMonitorNode(Node):
    def __init__(self):
        self.sim_window = SlidingWindow(5.0, 100.0)
        self.real_window = SlidingWindow(5.0, 100.0)
        self.error_window = SlidingWindow(5.0, 100.0)
        self.baseline_histograms = None  # List[np.ndarray] per joint
        self.timer = self.create_timer(0.1, self.compute_and_publish)

    def on_sim_state(self, msg):
        self.sim_window.push(stamp, positions)

    def on_real_state(self, msg):
        self.real_window.push(stamp, positions)

    def compute_and_publish(self):
        aligned_sim, aligned_real = self.aligner.align(
            self.sim_window, self.real_window
        )
        if len(aligned_sim) < self.min_samples:
            self.publish_empty_metrics()
            return

        errors = aligned_sim - aligned_real
        self.error_window.push_batch(errors)

        kl_per_joint = []
        for j in range(n_joints):
            hist_q = histogram(self.error_window[:, j])
            hist_p = self.baseline_histograms[j]
            kl_per_joint.append(kl_divergence(hist_p, hist_q))

        mmd_stat = mmd_rbf(aligned_sim_full, aligned_real_full, self.gamma)
        mmd_p = permutation_test(aligned_sim_full, aligned_real_full, B=100)

        shift, method = detect_shift(
            np.mean(kl_per_joint), mmd_stat, mmd_p, self.cfg
        )
        self.publish_metrics(kl_per_joint, mmd_stat, mmd_p, shift, method)
```

---

## 8. 计算复杂度与优化

| 操作 | 复杂度 | N=500, D=12 | 优化 |
|------|--------|-------------|------|
| 直方图（KL） | O(N) per joint | < 1ms | 固定 bin 边界缓存 |
| MMD 核矩阵 | O(m² + n² + mn) | ~30ms | 子采样至 m,n ≤ 200 |
| 置换检验 B=100 | O(B × MMD) | ~50ms | 并行化 / 降低 B 至 50（开发模式） |

**子采样策略**：当 N > 200 时，均匀抽取 200 个样本计算 MMD，KL 仍用全量（计算轻量）。

---

## 9. 单元测试用例

| 测试 ID | 输入 | 期望输出 |
|---------|------|---------|
| UT-KL-01 | 两相同高斯样本 | \(D_{KL} \approx 0\) |
| UT-KL-02 | N(0,1) vs N(0.5,1) | \(D_{KL} > 0.1\) |
| UT-KL-03 | 空窗口 | `shift_detected=false` |
| UT-MMD-01 | 两相同均匀分布 | MMD ≈ 0, p > 0.05 |
| UT-MMD-02 | N(0,1) vs N(2,1) | p < 0.05 |
| UT-ALN-01 | Real 延迟 30ms | 对齐误差 < 0.001 rad |
| UT-DET-01 | KL > threshold, MMD 不显著 | method="kl" |
| UT-DET-02 | 两者均显著 | method="both" |

---

## 10. Ground Truth 注入验证协议

配合 `/bridge/inject_shift` 服务，标准验证流程：

```bash
# 1. 基线运行 60s
ros2 action send_goal /hoc/execute_scenario "{scenario_id: 'SC-01', random_seed: 42, randomization_strength: 0.0}"

# 2. 注入 +30% 阻尼，持续 30s
ros2 service call /bridge/inject_shift bridge_monitor_msgs/srv/InjectShift \
  "{parameter_name: 'joint_damping', delta_percent: 30.0, duration_sec: 30.0}"

# 3. 检查监控输出
ros2 topic echo /monitor/distribution_metrics --field shift_detected
```

**验收指标**：

| 指标 | 目标 |
|------|------|
| Recall（+20% 阻尼） | > 90% |
| MTTD（+20% 阻尼） | < 5s |
| FPR（无扰动 10min） | < 5% |

---

## 11. 与风险引擎的接口映射

| DistributionMetrics 字段 | risk_engine 输入 | 归一化方式 |
|--------------------------|-----------------|------------|
| `kl_divergence_mean` | `distribution_shift` 分量 | 除以 `kl_threshold_mean × 2` |
| `mmd_statistic` | `distribution_shift` 分量 | 除以 `mmd_threshold × 2` |
| `shift_detected` | 告警触发器 | 直接布尔 |
| `tracking_error`（另路） | `tracking_error` 分量 | RMSE / threshold |

---

**上一篇**：[02 · 接口设计](./02-interface-design.md)  
**下一篇**：[04 · 人机运维控制台设计](./04-hoc-console-design.md)
