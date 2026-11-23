# Microduck-vision

> _A tiny biped robot that moves using reinforcement learning policies._
> —— [Microduck](https://github.com/pollen-robotics/microduck)

## 快速开始

安装 rust node

```bash
pnpm install
pnpm generate
pnpm dev
```

- 检查运行 `pnpm lint`、`pnpm test`。发布运行 `pnpm release`，EXE、DLL 和 `resources/` 会放到 Cargo `target-dir` 一级目录下的 `bin/`，默认为 `target/bin/`。分发时保留整个目录结构。
- 运行官方策略：在策略页面按需下载 [microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) 官方 ONNX。
- 运行官方策略：VLA预览中开启控制环
- 本地 RL 训练需要 `uv`。在训练页选择模拟机，并填写同版本 `vendor/microduck_rl` 的项目目录。桌面用该项目的锁文件准备 Python 环境，训练结果目录内的 `simulation-profile.json` 提供本次身体与资源。完成后的 `artifact.json` 记录 ONNX 路径、输入输出和执行指纹。

场景位置统一使用右手 Z 向上的世界坐标，单位为米；3D 显示时才转换成 Three.js 的 `(x, z, -y)`。每只模拟机的配置位置是场景放置偏移，运行中的平移和朝向只累计策略执行期间的实际仿真增量。开启空控制环、落地稳定、切换真机同步和重开控制环都不会归零场景位置；只有“归零”明确清除位移、朝向和累计距离。位移按机器 ID 保留，停止时写入配置目录的 `world-placements.json`。仿真状态和运行记录仍保留原始物理坐标、身体高度及姿态；真机没有定位反馈时保持已有场景位置。

## 和官方的关系

官方代码在 `vendor/microduck` 和 `vendor/microduck_rl`，固定在发布版本。  
当前项目可以理解为同时支持模拟训练和真机控制的集成终端。

### 复用了哪些部分

- `microduck_rl` 的 MJCF 和网格在编译时从 pinned vendor 读取。MuJoCo 从内存加载，3D 视图读取随应用分发的同源 STL。
- `duck-control` 核心loop(模块级复用) + MoJoCo 模拟计算逻辑。

### 演进了哪些部分

- 重实现loop
  - 官方其实有完备的逻辑，但是因为最终运行端不一致分散在不同的代码里
  - 纯Web版可参考 [microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/main/README.md)
  - 仓库本意是 学习 > 完备的把机器运行起来 -> 单终端MoJoCo跑训练、不同舵机测试、模拟环境看效果、实机接入
- 能重实现loop的基准
  - onnx 驱动只在意关节反馈，且本身输出稳定
  - 几何动力学完全由代码控制
    - 身体树统一 (舵机长宽高一致，舵机宽高的小变化被随机化参数吃掉了，重量独立计算，影响的是)
    -
  - 身体动力学完全取决于真实电机 [crates\servo](./crates/servo)
  - 即：
    - s2r 部分纯代码构建的模拟环境基本能还原任意电机条件下的官方场景
    - 接受官方onnx + 非XL330 电机的鸭子可能抽搐

> 核心是：复现官方onnx + XL330 的运动轨迹数据

### ON MY WAY

> 单终端跑意味着：所有性能极限取决于我自己的显卡 -> 用轨迹数据当老师，尝试些复杂的机械控制方案

- [ ] DeepMimic

---

核心开发约定见 [AGENTS](AGENTS.md)

每个 crate 的作用自行查看`Cargo.toml`~`description`
