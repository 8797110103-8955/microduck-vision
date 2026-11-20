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

- 重实现loop
  - 官方其实有完备的逻辑，但是因为最终运行端不一致分散在不同的代码里
  - 纯Web版可参考 https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/main/README.md
  - 仓库本意是 学习 > 完备的把机器运行起来 -> 单终端MoJoCo跑训练、不同舵机测试、模拟环境看效果、实机接入
- 能重实现loop的基准
  - 动力学中影响身体平衡参数并不是所有都进入Token，而是留在loop内
  - ONNX输入输出本身是稳定的，只要做一层运动学的转义就好
  - 即：接受官方onnx + 非XL330 电机的鸭子可能抽搐

> 核心是：复现官方onnx + XL330 的运动轨迹数据

### ON MY WAY

> 单终端跑意味着：所有性能极限取决于我自己的显卡 -> 用轨迹数据当老师，尝试些复杂的机械控制方案

- [ ] DeepMimic

---

核心开发约定见 [AGENTS](AGENTS.md)

每个 crate 的作用自行查看`Cargo.toml`~`description`
