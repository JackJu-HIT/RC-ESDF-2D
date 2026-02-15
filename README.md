# RC-ESDF-2D
🚀 **A high-performance C++ implementation of the Robo-centric ESDF algorithm for any-shape robotic planning.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++: 14/17](https://img.shields.io/badge/C++-14%2F17-blue.svg)](https://isocpp.org/)
[![Origin: FAST-Lab](https://img.shields.io/badge/Origin-FAST--Lab-red.svg)](https://github.com/ZJU-FAST-Lab)

## 📖 简介 (Introduction)

**RC-ESDF-2D** 是基于浙大高飞团队（FAST-Lab）研究成果的高效复现版本。该库实现在机器人中心坐标系（Body Frame）下构建欧几里得符号距离场（2D ESDF），专为复杂形状机器人的局部路径规划（如 TEB, MPC）提供核心支撑。

本项目复现自以下学术论文：
> **Robo-centric ESDF: A Fast and Accurate Whole-body Collision Evaluation Tool for Any-shape Robotic Planning**, *Weijia Zhou, Fei Gao, et al.*

---

## ✨ 核心特性 (Features)

*   **论文算法复现**: 复现了论文中提出的机器人中心 ESDF 构建逻辑，适用于任意形状的多边形足迹（Any-shape Footprint）。
*   **机器人中心坐标系 (Robo-Centric)**: 所有计算均在 Body Frame 下实时生成，无需全局地图，天然适配动态环境避障。
*   **高速 $O(1)$ 查询**: 基于双线性插值（Bilinear Interpolation），单次查询耗时仅约 **2.4 μs**（测试环境：普通移动端 CPU），满足极致的实时性需求。
*   **解析梯度 (Analytic Gradient)**: 提供连续、平滑的一阶解析梯度，确保基于梯度的优化器（如 g2o, Ceres, NLopt）能够快速且稳定地收敛。
*   **可视化辅助**: 内置基于 OpenCV 的诊断工具，可直观对比物理轮廓（Yellow Box）与离散场（SDF Field）的对齐准确度。
*   **轻量化设计**: 仅依赖 Eigen3 核心库，易于集成到现有的 ROS 或嵌入式导航系统中。

---

## 📊 可视化效果 (Visualization)

![ESDF Visualization](https://github.com/JackJu-HIT/RC-ESDF-2D/blob/master/files/RC-ESDF%20Normalized.png) 

通过内置的 `visualizeEsdf()` 函数，您可以清晰地观察：
*   🔴 **红色区域**: 机器人内部 ($dist < 0$)。
*   🟢 **绿色区域**: 机器人外部安全区 ($dist > 0$)。
*   🟨 **黄色轮廓**: 输入的原始多边形物理边界。
*   ⚪ **白色箭头**: 解析梯度向量 $\nabla D$（始终指向最短路径脱离碰撞的方向）。

---

## 🚀 快速开始 (Quick Start)

### 依赖 (Dependencies)
*   [Eigen3](http://eigen.tuxfamily.org/) (核心计算)
*   [OpenCV](https://opencv.org/) (可选，仅用于可视化调试)
*   CMake (>= 3.10)

### 编译与运行 (Build)
```bash
git clone https://github.com/JackJu-HIT/RC-ESDF-2D.git
cd RC-ESDF-2D
mkdir build && cd build
cmake ..
make
./test_rc_esdf
```

### 核心代码示例 (Basic Usage)
```cpp
#include "rc_esdf.h"

RcEsdfMap esdf;
// 初始化地图：10m x 10m, 分辨率 0.1m
esdf.initialize(10.0, 10.0, 0.1); 

// 定义机器人多边形顶点 (Body Frame)
std::vector<Eigen::Vector2d> footprint;
footprint.push_back({0.7, 0.3});
footprint.push_back({-0.7, 0.3});
footprint.push_back({-0.7, -0.3});
footprint.push_back({0.7, -0.3});

// 离线/启动时生成 SDF 场
esdf.generateFromPolygon(footprint);

// 在线查询：输入障碍物在 Body Frame 的坐标
double dist;
Eigen::Vector2d grad;
if (esdf.query(Eigen::Vector2d(0.4, 0.2), dist, grad)) {
    if (dist < 0) {
        // 发生碰撞！利用 -grad 方向将机器人推离障碍物
    }
}
```

---

## 🛠 应用场景 (Applications)
*   **TEB Local Planner**: 增强碰撞检测逻辑，为非圆形状机器人提供更精确的代价约束。
*   **轨迹优化 (Trajectory Optimization)**: 在 MPC 或 EGO-Planner 框架中作为硬约束或惩罚项。
*   **势场法导航**: 生成高质量、无震荡的斥力场。

---



## 📄 协议 (License)
本项目基于 [MIT License](LICENSE) 开源。
```
