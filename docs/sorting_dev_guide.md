# 下游仓库：多任务回放、监控与网页控制台联调指南 (Downstream Sorting Dev Guide)

本指南针对下游 `ros2-moveit-pybullet-bridge` 仓库，规范多目标任务在 **PyBullet 场景对齐、Policy 动态运行、以及 React HOC 网页下发指令** 时的调试流程与执行命令。

---

## 1. 本地环境构建

编译下游的桥接器、监控器和风险管理包：
```bash
colcon build --symlink-install --packages-select \
  bridge_monitor_msgs \
  pybullet_bridge \
  dist_monitor \
  risk_engine \
  hoc_console
```

---

## 2. 策略运行器服务化 (`risk_engine/risk_engine/policy_runner.py`)

在 `policy_runner` 中，除了加载神经网络权重进行推理之外，还需实现 `SetLanguageTask` ROS 2 服务：

```python
from bridge_monitor_msgs.srv import SetLanguageTask
import numpy as np

class PolicyRunnerNode(Node):
    def __init__(self):
        super().__init__('policy_runner_node')
        self.srv_set_task = self.create_service(
            SetLanguageTask, '/risk/set_language_task', self._handle_set_language_task)
        # 初始化 Policy 加载和 Text Embedder...
        
    def _handle_set_language_task(self, request, response):
        instruction = request.instruction
        self.get_logger().info(f"Received new sorting instruction: {instruction}")
        
        # 1. 转换为 Embedding 向量
        self.active_text_embedding = self.text_embedder.get_embedding(instruction)
        self.task_running = True
        
        response.success = True
        response.message = f"Activated policy with task: {instruction}"
        return response
```

---

## 3. 网页控制台 (HOC) 交互配置 (`hoc_console/`)

在 React 组件 `TaskControlPanel.jsx` 中，添加控制指令下发交互：

```javascript
import React, { useState } from 'react';

export function TaskControlPanel({ webSocket }) {
  const [instruction, setInstruction] = useState('pick up the red box and place it in the left bin');

  const sendTask = () => {
    // 通过 WebSocket 发送任务指令给 hoc_server
    webSocket.send(JSON.stringify({
      event: 'trigger_language_task',
      data: { instruction: instruction }
    }));
  };

  return (
    <div className="task-panel">
      <h3>具身任务下发控制板</h3>
      <select onChange={(e) => setInstruction(e.target.value)} value={instruction}>
        <option value="pick up the red box and place it in the left bin">抓取红方块放左箱</option>
        <option value="pick up the blue cylinder and place it in the right bin">抓取蓝圆柱放右箱</option>
        <option value="pick up the green sphere and place it in the left bin">抓取绿球放左箱</option>
      </select>
      <button onClick={sendTask}>启动神经网络策略</button>
    </div>
  );
}
```

---

## 4. 联调运行步骤

### 第一步：启动仿真与桥接（加载多目标模型）
```bash
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  model_path:=config/models/franka_panda_multi.xml
```

### 第二步：运行 Policy 推理与 WebSocket 转发节点
将中游导出的 Policy 放入指定目录，并拉起运行器：
```bash
ros2 run risk_engine policy_runner \
  --ros-args -p policy_path:=checkpoints/best_checkpoint.npz &

ros2 run hoc_console hoc_server
```

### 第三步：打开 HOC 网页端进行测试
1. 在浏览器中打开 `http://localhost:3000` (或启动本地 React 开发服务 `npm run dev`)。
2. 确认仪表盘已通过 WebSocket 联通。
3. 在“任务下发面板”中，选择 `"抓取蓝圆柱放右箱"`，点击启动。
4. **观察视觉指标**：在 PyBullet 窗口中观察机械臂是否朝圆柱运动；在网页仪表盘上实时观测追踪误差（Tracking Error）和 KL 散度的变化。
