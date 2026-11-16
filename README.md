# Microduck-vision

> _A tiny biped robot that moves using reinforcement learning policies._
> —— [Microduck](https://github.com/pollen-robotics/microduck)

## 快速开始

安装 rust node

```bash
pnpm install
pnpm dev
```

- 检查：`pnpm lint`、`pnpm test`；打包：`pnpm release`。
- 运行官方策略：在策略页面按需下载 [microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) 官方 ONNX。
- 运行官方策略：VLA预览中开启控制环

## 和官方的关系

官方代码在 `vendor/microduck` 和 `vendor/microduck_rl`，固定在发布版本。

### 复用了哪些部分

- `microduck_rl` 的 MJCF 和网格：直接读取，3D 视图和仿真都用它。
- `duck-control` 的核心控制逻辑

### 自定义哪些部分

- 

---

核心开发约定见 [.rules](.rules)  

每个 crate 的作用自行查看`Cargo.toml`~`description`
