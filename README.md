# Microduck-vision

> _A tiny biped robot that moves using reinforcement learning policies._
> —— [Microduck](https://github.com/pollen-robotics/microduck)

一个面向 MicroDuck 的桌面实验终端，把官方策略运行、MuJoCo 仿真、不同舵机测试、本地训练和实机控制放在同一个工作流里。  
核心目标是：**在官方已有成熟onnx的背景下，探索其他舵机 & 动力学条件 & 训练策略**

复现官方 ONNX + XL330 的运动轨迹是前期基准，HD1910 Compatible 接入官方 ONNX，用来验证仿真与执行流程可跨电机匹配，并为后续积累参考数据。

## 快速开始

当前桌面构建面向 Windows x64。准备 Rust 1.90 或更新版本、Node.js、pnpm、MSVC C++ 构建工具和 WebView2 运行时，然后在仓库根目录运行：

```bash
pnpm install
pnpm generate
pnpm dev
```

首次构建会下载锁定版本的 MuJoCo SDK 和 ONNX Runtime，需要网络连接。只运行桌面预览和官方 ONNX 策略，无需准备 Python 训练环境。

### 运行官方策略

1. 创建或选择一台模拟机，选择要测试的模型。复现官方行为时先使用 XL330。
2. 在“策略”页按需下载 [microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) 提供的官方 ONNX。
3. 回到 ONNX 预览，选择模拟机和已缓存的策略，设置运动指令后开启控制环。

接入实机时，先在机器与调试页面确认连接、关节映射、方向和零位，再在 ONNX 预览中切换真机同步、开启力矩和真机控制环。

### 本地训练

本地训练需要支持 CUDA 的 NVIDIA GPU 和驱动，应用启动训练时会调用 `uv` 准备训练环境。

```powershell
.\target\bin\microduck-vision.exe --help | Out-Host
```

### 检查与发布

```bash
pnpm release # 默认发布目录 `target/bin/`
```

## 和官方的关系

官方代码保存在 [vendor/microduck](vendor/microduck) 和 [vendor/microduck_rl](vendor/microduck_rl)，固定在发布版本并直接复用。桌面端围绕这些实现组织机器配置、执行、预览和训练流程。

### 直接复用

- `microduck_rl` 的 MJCF、网格和训练环境。MJCF 与网格在编译时从锁定的 vendor 版本读取；MuJoCo 从内存加载资源，3D 视图读取随应用分发的同源 STL。
- `microduck` 的标准关节定义、运动学，以及 `duck-control` 中的观测构建、策略执行等模块。
  - 官方原运行逻辑分散在机器人运行端、仿真和训练工具中
  - 项目把执行流程整理为桌面端可以共同使用的控制环：读取反馈、构建观测、执行策略、生成关节目标，再交给模拟或实机后端
  - 各包细节描述可查看crates/*/Cargo.toml.description
- 纯 Web 仿真也可参考官方的 [microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/main/README.md)

### 仿真与不同舵机

策略通过约定的观测和动作接口工作，观测包含关节位置与速度、机体角速度、重力方向、上一帧动作及运动指令；底层可以用仿真反馈或实机反馈提供这些数据。  
因此可以复用同一套策略执行流程，比较不同模型的表现；当前模型统一使用 vendor 提供的 `scene_backlash.xml` 和 `robot_groundcontact_backlash.xml`，包含每个舵机关节的被动背隙。  
模型差异集中在 [models](apps/body-runtime/src/models)：

| 模型              | 用途                                                                  |
| ----------------- | --------------------------------------------------------------------- |
| XL330             | 官方策略与运动轨迹的参考模型。                                        |
| HD1910 Compatible | 在 HD1910 参数上加入位置刚度补偿及响应拟合，尝试接近 XL330 的运动模型 |
| HD1910            | 使用 HD1910 电机参数，观察原生响应。                                  |

运动结果同时取决于机体几何、质量与惯量、接触、电机响应、反馈和控制时序。  
更换舵机后，机械尺寸、质量分布和动力学差异需要测量与验证，域随机化也需要覆盖相应变化。  
当前沿用官方机体几何与惯性数据，HD1910 兼容模型的轨迹拟合是仿真中的近似。复用官方 ONNX 并不意味着舵机的实机会产生相同运动，但实机验证几乎能做到大致相同；

## ON MY WAY

> 自训练 ONNX 默认加载 %APPDATA%/microduck-vision/policies/trained/****/manifest.json  
> aaa

- [x] DeepMimic 风格的运动模仿训练。
- [ ] DeepMimic -> Eureka

---

开发约定见 [AGENTS.md](AGENTS.md)。

# 附录

> Robot-learning techniques for PhysicalAI [原论文（arXiv v1）](https://arxiv.org/abs/2608.01851v1)

## 1.1 Code-as-policy

```mermaid
flowchart TB
    s3_1["1.1<br/>Code-as-policy"]
    s3_1_1["1.1.1<br/>Zero-shot synthesis"]
    s3_1_2["1.1.2<br/>Closed-loop self-repair"]
    s3_1_3["1.1.3<br/>Skill-library accumulation"]
    s3_1_4["1.1.4<br/>Evolutionary search"]
    s3_1_5["1.1.5<br/>Full self-improving loop"]
    s3_1 --- s3_1_1
    s3_1 --- s3_1_2
    s3_1 --- s3_1_3
    s3_1 --- s3_1_4
    s3_1 --- s3_1_5
    s3_1_1_works["Code-as-Policies: LLM programs for embodied control [144]<br/>ProgPrompt: situated robot task plans via LLMs [236]<br/>VoxPoser: composable 3D value maps [100]<br/>Instruct2Act: multimodal instructions to actions [97]<br/>ChatGPT for Robotics: design principles &amp; abilities [244]<br/>TidyBot: personalised robot assistance [265]<br/>RoboCodeX: multimodal code for behavior synthesis [186]<br/>RoboScript: code for free-form manipulation [27]<br/>Text2Motion: instructions to feasible plans [147]<br/>Demo2Code: demonstrations to synthesized code [251]<br/>Statler: state-maintaining LLMs for reasoning [285]<br/>Prompt2Walk: prompt a robot to walk with LLMs [257]<br/>RoboPro: video-instructed policy code generation [273]"]
    s3_1_1 --- s3_1_1_works
    s3_1_2_works["Inner Monologue: embodied reasoning via planning [102]<br/>DoReMi: recovering plan-execution misalignment [73]<br/>REFLECT: summarizing experiences for failure explanation [159]<br/>Code-as-Monitor: constraint-aware visual programming [306]<br/>AHA: VLM reasoning over manipulation failures [48]<br/>Introspective Planning: uncertainty vs. task ambiguity [145]"]
    s3_1_2 --- s3_1_2_works
    s3_1_3_works["RoboCoder: from basic skills to general tasks [137]<br/>DROC: distilling knowledge via language corrections [294]"]
    s3_1_3 --- s3_1_3_works
    s3_1_4_works["CaP-X: benchmarking &amp; improving coding agents [58]<br/>RoboEvolve: co-evolving planner-simulator [26]<br/>GEAR: policies via LLM evolutionary search [72]"]
    s3_1_4 --- s3_1_4_works
    s3_1_5_works["ASPIRE: agentic skills discovery for robotics [160]<br/>ENPIRE: real-world policy self-improvement [272]<br/>RoboClaw: scalable long-horizon robotic tasks [139]"]
    s3_1_5 --- s3_1_5_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1,s3_1_1 blue;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_1_works blueLight;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_2 teal;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_2_works tealLight;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_3 green;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_3_works greenLight;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_4 yellow;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_4_works yellowLight;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_5 red;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_1_5_works redLight;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
```

## 1.2 End-to-end VLA

```mermaid
flowchart TB
    s3_2["1.2<br/>End-to-end VLA"]
    s3_2_works["RT-1: robotics transformer for real-world control [17]<br/>RT-2: VLA models transfer web knowledge [18]<br/>Octo: open-source generalist robot policy [66]<br/>OpenVLA: open-source VLA model [120]<br/>RoboFlamingo: VLM foundation models as imitators [142]<br/>CogACT: synergizing cognition and action [138]<br/>SpatialVLA: spatial representations for VLA [207]<br/>π₀: VLA flow model for general control [15]<br/>π₀.₅: VLA with open-world generalisation [14]<br/>GR00T N1: foundation model for humanoid robots [194]<br/>Gemini Robotics: bringing AI into the physical world [65]"]
    s3_2 --- s3_2_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_2 teal;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_2_works tealLight;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
```

## 1.3 Reward synthesis

```mermaid
flowchart TB
    s3_3["1.3<br/>Reward synthesis"]
    s3_3_works["Eureka: human-level reward design via LLMs [163]<br/>DrEureka: LLM-guided sim-to-real transfer [164]<br/>Text2Reward: reward shaping with LLMs [275]<br/>Language-to-Rewards: for robotic skill synthesis [288]<br/>Eurekaverse: environment curriculum generation [146]<br/>RoboGen: automated learning via generative sim [256]<br/>Auto MC-Reward: automated dense reward design [136]"]
    s3_3 --- s3_3_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_3 orange;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_3_works orangeLight;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
```

## 1.4 Skill libraries

```mermaid
flowchart TB
    s3_4["1.4<br/>Skill libraries"]
    s3_4_1["1.4.1<br/>Unsupervised latent-RL"]
    s3_4_2["1.4.2<br/>Skill-space &amp; hierarchical RL"]
    s3_4_3["1.4.3<br/>LLM &amp; code libraries"]
    s3_4_4["1.4.4<br/>Open-world LLM agents"]
    s3_4 --- s3_4_1
    s3_4 --- s3_4_2
    s3_4 --- s3_4_3
    s3_4 --- s3_4_4
    s3_4_1_works["DIAYN: learning skills without a reward function [50]<br/>DADS: dynamics-aware discovery of skills [229]<br/>LSD: Lipschitz-constrained skill discovery [200]<br/>CIC: contrastive intrinsic control [129]<br/>METRA: scalable unsupervised RL, metric-aware [202]"]
    s3_4_1 --- s3_4_1_works
    s3_4_2_works["SPiRL: accelerating RL with learned skill priors [206]<br/>OPAL: offline primitive discovery [6]<br/>PARROT: data-driven behavioral priors [235]<br/>SkiMo: skill-based model-based RL [231]"]
    s3_4_2 --- s3_4_2_works
    s3_4_3_works["Voyager: open-ended embodied agent with LLMs [250]<br/>LOTUS: continual imitation via skill discovery [248]<br/>LRLL: bootstrapping composable skills [242]<br/>BOSS: learning new tasks with LLM guidance [297]<br/>SPRINT: pre-training via instruction relabeling [295]<br/>Uni-Skill: self-evolving skill repository [274]<br/>SkillFlow: lifelong skill discovery &amp; evolution [300]"]
    s3_4_3 --- s3_4_3_works
    s3_4_4_works["GITM: capable agents with text-based memory [309]<br/>JARVIS-1: memory-augmented multimodal agents [259]<br/>ExpeL: LLM agents are experiential learners [301]<br/>Optimus-1: hybrid multimodal memory agents [143]<br/>Odyssey: Minecraft agents with open-world skills [157]"]
    s3_4_4 --- s3_4_4_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_4,s3_4_1,s3_4_2,s3_4_3,s3_4_4 purple;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_4_1_works,s3_4_2_works,s3_4_3_works,s3_4_4_works purpleLight;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
```

## 1.5 Sim-to-real & transfer

```mermaid
flowchart TB
    s3_5["1.5<br/>Sim-to-real &amp; transfer"]
    s3_5_works["Open X-Embodiment: datasets and RT-X models [195]<br/>CrossFormer: one policy for many embodiments [44]<br/>RoboCat: self-improving generalist agent [16]<br/>Mirage: zero-shot transfer via cross-painting [29]"]
    s3_5 --- s3_5_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_5 pink;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_5_works pinkLight;
```

## 1.6 Benchmarks & simulators

```mermaid
flowchart TB
    s3_6["1.6<br/>Benchmarks &amp; simulators"]
    s3_6_works["LIBERO: knowledge transfer for lifelong learning [149]<br/>robosuite: modular simulation framework [310]<br/>BEHAVIOR-1K: 1000 everyday household activities [134]<br/>Meta-World: benchmark for multi-task &amp; meta RL [286]<br/>RLBench: robot learning benchmark &amp; environment [105]<br/>ManiSkill2: unified benchmark for manipulation [71]<br/>CALVIN: language-conditioned long-horizon tasks [178]"]
    s3_6 --- s3_6_works

    classDef blue fill:#93C5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef blueLight fill:#DBEAFE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef teal fill:#5EEAD4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef tealLight fill:#CCFBF1,stroke:#333333,color:#111111,stroke-width:1px;
    classDef green fill:#86EFAC,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_6 green;
    classDef greenLight fill:#DCFCE7,stroke:#333333,color:#111111,stroke-width:1px;
    class s3_6_works greenLight;
    classDef yellow fill:#FCD34D,stroke:#333333,color:#111111,stroke-width:1px;
    classDef yellowLight fill:#FEF3C7,stroke:#333333,color:#111111,stroke-width:1px;
    classDef red fill:#FCA5A5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef redLight fill:#FEE2E2,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orange fill:#FDBA74,stroke:#333333,color:#111111,stroke-width:1px;
    classDef orangeLight fill:#FFEDD5,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purple fill:#C4B5FD,stroke:#333333,color:#111111,stroke-width:1px;
    classDef purpleLight fill:#EDE9FE,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pink fill:#F9A8D4,stroke:#333333,color:#111111,stroke-width:1px;
    classDef pinkLight fill:#FCE7F3,stroke:#333333,color:#111111,stroke-width:1px;
```