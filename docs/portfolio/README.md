# 作品集导出材料

本目录存放面向**集成交付工程师 / 技术评审**的可导出文档，与 `docs/design/` 详设文档互补：此处侧重**交付视角**（验收、部署、演示），design 侧重实现细节。

## 文件说明

| 文件 | 用途 | 推荐导出格式 |
|------|------|--------------|
| [SYSTEM_DESIGN_SPEC.md](./SYSTEM_DESIGN_SPEC.md) | 完整系统设计说明书（13 页等效） | PDF |
| [system-design-slides.md](./system-design-slides.md) | Marp 幻灯片源文件（13 页） | PDF / PPTX |

## 导出 PDF（推荐）

### 方式 A · Marp 幻灯片 → PDF（最快）

```bash
# 安装 Marp CLI（一次性）
npm install -g @marp-team/marp-cli

# 导出 PDF
cd docs/portfolio
marp system-design-slides.md --pdf -o system-design-slides.pdf

# 导出 PowerPoint（可选）
marp system-design-slides.md --pptx -o system-design-slides.pptx
```

### 方式 B · Pandoc 说明书 → PDF

```bash
# 需安装 pandoc + texlive（或 wkhtmltopdf）
cd docs/portfolio
pandoc SYSTEM_DESIGN_SPEC.md -o system-design-spec.pdf \
  --pdf-engine=xelatex -V CJKmainfont="Noto Sans CJK SC" \
  -V geometry:margin=2.5cm --toc
```

### 方式 C · VS Code / Cursor

1. 安装扩展 **Marp for VS Code**
2. 打开 `system-design-slides.md`
3. 右上角 **Export Slide Deck** → PDF

## 配图替换

幻灯片与说明书中标注了 `docs/assets/` 路径，导出前可将合成图替换为真实录屏截图：

| 占位 | 建议替换为 |
|------|-----------|
| `portfolio-overview.png` | 系统总览 |
| `m2-iiwa-pipeline.svg` | MoveIt 闭环 |
| `m4-monitor-metrics.png` | 监控指标 |
| `m5-hoc-dashboard.png` | HOC 浏览器截图 |

生成基础配图：`python3 scripts/generate_milestone_assets.py`

## 版本

与 [docs/design/README.md](../design/README.md) v0.7 对齐 · 2026-06-20
