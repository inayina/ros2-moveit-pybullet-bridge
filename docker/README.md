# Docker

基于 `ros:jazzy-ros-base` 构建 bridge 工作区，并挂载 `robot-arm-episode-data-lab` 的 LeRobot 导出目录。

## 前置

1. 在 episode-data-lab 中生成导出（若尚未有）：

```bash
cd ~/robot-sim-lab/robot-arm-episode-data-lab
python scripts/batch_collect.py --output dataset/v1 --num-episodes 2 --seed 42
python scripts/export_lerobot_style.py dataset/v1 --output dataset/v1/lerobot_export
```

2. 设置挂载源路径：

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
```

## 命令

```bash
# 构建镜像
docker compose build

# 冒烟：URDF 检查 + offline_compare + portfolio_demo（15s headless）
docker compose run --rm verify

# 使用 LeRobot 作为 Real 源的 portfolio 演示（headless，Ctrl+C 退出）
docker compose run --rm portfolio-demo
```

容器内路径固定为：

- `EPISODE_DATA_LAB_ROOT=/data/episode-data-lab`
- `LEROBOT_EXPORT=/data/episode-data-lab/dataset/v1/lerobot_export`

## GUI

Headless 验证默认 `sim_mode:=DIRECT`。需要 RViz / PyBullet GUI 时请在宿主机按 README 启动，或自行配置 X11 转发。

## 构建上下文

根目录 `.dockerignore` 排除文档、前端依赖与本机 colcon 产物，避免把 ~1 GB 无关文件打进 build context；仅保留 ROS 包源码与 `scripts/` 验证脚本。
