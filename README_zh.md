# hurodes

hurodes（Humanoid Robot Description）是一个用于人形机器人模型描述、转换与处理的 Python 工具包。项目提出了自定义的 HRDF（Humanoid Robot Description Format）格式，为 MJCF、URDF 等主流机器人描述格式之间的互转提供了统一的中间桥梁，并封装了生成器、解析器及常用脚本，帮助用户高效完成机器人模型的创建、转换与批量处理。

---

## 核心特性

- **HRDF 统一中间格式**：采用结构化的 HRDF 目录（CSV + YAML + Mesh）描述机器人信息，便于批量编辑与分析，且实现更全面的机器人描述
- **灵活的生成器/解析器**：支持 MJCF ⇆ HRDF ⇆ URDF 的双向转换，满足多格式协同需求
- **多机器人合并**：通过名称前缀机制，可将多个机器人模型自动合并到同一个 MJCF 文件，支持协作/群体仿真
- **脚本化批处理**：内置命令行脚本，轻松完成格式互转、模型合并等常见任务
- **模块化设计**：清晰的包结构，方便二次开发和功能扩展
- **丰富的接口**：内置大量接口，为下游任务（例如强化学习训练、动作重映射、实物部署）等提供支持，避免手动填写带来的错误
---

## 安装

本项目基于 Python 3.9 及以上版本。

### 方式一：从 PyPI 安装（推荐）

```bash
# 从 PyPI 安装最新稳定版本
pip install hurodes

# 或安装包含开发依赖的版本
pip install hurodes[dev]
```

### 方式二：从源代码安装

```bash
# 克隆仓库
git clone https://github.com/ZyuonRobotics/humanoid-robot-description.git
cd humanoid-robot-description

# 正常安装
pip install .

# 开发者安装（包含测试依赖）
pip install -e .[dev]
```

---

## 快速开始

以下示例演示了主要脚本的模型转换与可视化用法：


```bash
# 解析 URDF 或 MJCF 为 HRDF（通过 format_type 选择 'urdf' 或 'mjcf'）
hurodes-parse path/to/robot.urdf your_robot_name --format_type urdf

# 从 HRDF 生成 MJCF 并可视化
hurodes-generate your_robot_name

# 合成多个机器人（HRDF）为一个 MJCF 并可视化
hurodes-generate-composite robot1,robot2
```

---


## HRDF 格式简介

HRDF 采用目录结构存储机器人信息：

```
assets/robots/your_robot_name/
├── actuator.csv     # 执行器参数
├── body.csv         # 刚体信息
├── collision.csv    # 碰撞体参数
├── joint.csv        # 关节信息
├── mesh.csv         # 网格文件索引
├── meshes/          # 具体网格资源
└── meta.yaml        # 元信息 (树结构、地面参数等)
```

- **中间桥梁**：在 MJCF、URDF 等格式互转时充当统一数据载体。
- **结构化存储**：CSV/YAML 文件便于批量读取、分析、版本控制。
- **可扩展**：目录结构清晰，易于新增属性或支持新格式。
- **项目缓存**：运行时生成的临时数据会存放于用户主目录的 `~/.hurodes`，无需手动维护。
