# RC-ESDF-2D
🚀 A high-performance, robo-centric 2D Signed Distance Field implementation for real-time collision avoidance and local trajectory optimization.

# RC-ESDF: Robo-Centric 2D Signed Distance Field

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++: 14/17](https://img.shields.io/badge/C++-14%2F17-blue.svg)](https://isocpp.org/)

**RC-ESDF** 是一个高效、轻量级的机器人中心欧几里得符号距离场 (2D ESDF) 实现库。它专为地面机器人的局部规划（如 TEB, MPC）设计，支持实时的高速距离查询和解析梯度计算。

![ESDF Visualization](https://your-image-link-here.com/demo.png) 
*(这里放你最后运行出来的彩色梯度图，非常吸引人)*

## ✨ 特性 (Features)

*   **机器人中心坐标系 (Robo-Centric)**: 所有的计算都在 Body Frame 下进行，非常适合动态障碍物避障和局部优化。
*   **高速查询**: 基于双线性插值 (Bilinear Interpolation) 的 $O(1)$ 时间复杂度查询。
*   **解析梯度 (Analytic Gradient)**: 提供连续、平滑的梯度场，助力基于梯度的优化器（如 g2o, Ceres）快速收敛。
*   **可视化调试**: 集成基于 OpenCV 的诊断工具，直观查看距离场分布、像素网格对齐和梯度方向。
*   **零依赖 (除核心库外)**: 仅依赖 Eigen3，保持极高的可移植性。

## 🚀 快速开始 (Quick Start)

### 依赖 (Dependencies)
*   Eigen3 (必选)
*   OpenCV (可选，仅用于可视化)
*   CMake (>= 3.10)

### 编译 (Build)
```bash
mkdir build && cd build
cmake ..
make
```

### 简单示例 (Example)
```cpp
#include "rc_esdf.h"

RcEsdfMap esdf;
esdf.initialize(10.0, 10.0, 0.1); // 10m x 10m, 0.1m resolution

// 定义机器人多边形
std::vector<Eigen::Vector2d> footprint = {{0.5, 0.3}, {-0.5, 0.3}, {-0.5, -0.3}, {0.5, -0.3}};
esdf.generateFromPolygon(footprint);

// 在线查询
double dist;
Eigen::Vector2d grad;
if (esdf.query(Eigen::Vector2d(0.4, 0.2), dist, grad)) {
    // 处理碰撞或更新代价函数
}
```

## 📊 可视化说明 (Visualization)
仓库提供的诊断工具可以显示：
*   **红色区域**: 机器人本体内部 ($dist < 0$)。
*   **绿色区域**: 外部安全区域 ($dist > 0$)。
*   **黄色框**: 输入的物理轮廓。
*   **白色箭头**: 距离场梯度 $\nabla D$（指向安全方向）。

## 🛠 应用场景 (Applications)
*   **TEB Local Planner**: 作为插件替换原有的简单碰撞检查，提供平滑推力。
*   **MPC 避障**: 在预测控制中加入距离约束。
*   **虚拟人工势场法**: 生成高质量的引力/斥力场。

## 📄 开源协议 (License)
本项目采用 [MIT License](LICENSE) 协议。
```
