# Microduck-vision

> _A tiny biped robot that moves using reinforcement learning policies._
> —— [Microduck](https://github.com/pollen-robotics/microduck)

## 快速开始

安装 rust node

```bash
pnpm install
pnpm dev
```

- 检查：`pnpm lint`、`pnpm test`；发布：`pnpm release` 将 `microduck-vision.exe`、运行所需 DLL 和 `resources/` 放到 `target/bin/`。分发时保留整个目录结构。
- 运行官方策略：在策略页面按需下载 [microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) 官方 ONNX。
- 运行官方策略：VLA预览中开启控制环

## 和官方的关系

官方代码在 `vendor/microduck` 和 `vendor/microduck_rl`，固定在发布版本。  
当前项目可以理解为同时支持模拟训练和真机控制的集成终端。

### 复用了哪些部分

- `microduck_rl` 的 MJCF 和网格：编译时直接从 pinned vendor 读取并复制到输出目录，3D 视图和仿真都使用这份构建资源。
- `duck-control` 核心loop(模块级复用) + MoJoCo 模拟计算逻辑。

### 演进了哪些部分

- 重实现loop让其完整的运行在电脑端 => https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/main/README.md

---

核心开发约定见 [AGENTS](AGENTS.md)

每个 crate 的作用自行查看`Cargo.toml`~`description`
