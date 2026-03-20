# 🧭 Pathfinding Algorithms Collection
> A* · D* Lite · Random Map · Visualization

一个基于 Python 实现的**路径规划算法学习项目**，包含 A* 与 D* Lite 核心算法，支持随机迷宫生成与动态可视化，便于算法对比与学习。

---

## ✨ 核心特性
- **双算法实现**：经典 A* 算法 + 动态环境 D* Lite 算法
- **启发式切换**：A* 支持曼哈顿距离 / 欧氏距离两种启发式函数
- **随机地图**：可生成复杂障碍物地图，支持固定随机种子 (Seed) 复现实验
- **动态可视化**：基于 Matplotlib 实时绘制搜索过程与最终路径
- **代码清晰**：模块化设计，便于扩展更多路径规划算法

---

## 🛠️ 环境要求
- Python 3.13+
- Matplotlib 3.8+

## 🚀 快速开始
1.  **运行 A* 算法**
     ```bash
    python Dstar_Lite.py

3.  **运行 D Lite 算法**
     ```bash
    python Dstar_Lite.py

5.  **自定义配置**
    ```bash
    修改起点 / 终点：在对应算法主程序中修改 start 和 goal 元组
    固定随机地图：在 env.py 的 Env 类初始化中传入 seed=整数（如 seed=42）

## 📂 文件结构
| 📄 文件 | 📝 功能描述 |
|:-------|:-----------|
| `env.py` | 环境定义、障碍物生成、随机地图逻辑 |
| `plotting.py` | 可视化模块：绘制地图、搜索过程与路径动画 |
| `A_pratice_original.py` | A* 算法核心实现与主入口 |
| `Dstar_Lite.py` | D* Lite 算法核心实现与主入口 |

## 📝 项目说明
本项目为路径规划算法的学习与实践代码，通过对比 A* 与 D* Lite 算法，理解静态与动态环境下的路径规划差异，适合用于算法面试准备与学术学习。
