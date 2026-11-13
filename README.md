# 小鸭子

> *A tiny biped robot that moves using reinforcement learning policies.*
> —— [Microduck](https://github.com/pollen-robotics/microduck)

## 快速开始

需要 Rust 1.90 以上、Node.js 和 pnpm。

```bash
cd apps/desktop
pnpm install
pnpm tauri dev
```

- 检查：`pnpm lint`、`pnpm test`；打包：`pnpm release`。
- 运行官方策略：把 [microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) `v9` 下载到 `%APPDATA%\microduck\library\policies\official\`；仿真还需要按 `vendor/microduck_rl` 的说明启动 `duck-body`。

## 和官方的关系

官方代码在 `vendor/microduck` 和 `vendor/microduck_rl`，固定在发布版本。

### 复用了哪些部分

- `microduck_rl` 的 MJCF 和网格：直接读取，3D 视图和仿真都用它。
- 


---

约定见 [.rules](.rules)。每个 crate 是什么，看它 `Cargo.toml` 的 `description`；它和其他 crate 的边界写在下面的注释里。
