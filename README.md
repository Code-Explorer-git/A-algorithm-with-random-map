# A* Pathfinding with Random Map
#A*算法
一个基于 Python 的 A* 路径规划算法实现，支持随机生成复杂迷宫地图，并通过 Matplotlib 动态可视化搜索过程。

## 特性
1. 双启发式函数支持：可切换 曼哈顿距离 (Manhattan) 或 欧氏距离 (Euclidean)
2. 随机地图生成器：内置算法生成复杂障碍物，每次运行体验不同
3. 可复现性：支持固定随机种子 (Seed)，方便调试和对比实验
4. 动态可视化：实时展示探索过程和最终路径

## 环境要求
1. Python 3.13
2. Matplotlib

## 安装依赖
```bash
pip install matplotlib
```

## 快速开始
### 1. 运行默认配置
直接运行主程序即可看到随机地图和路径规划动画：
```bash
python A_pratice_original.py
```

### 2. 修改配置（可选）
- 修改起点 / 终点：在 `A_pratice_original.py` 的 main 函数中修改 start 和 goal 元组。
- 固定随机地图：在 `env.py` 的 Env 类初始化中传入 seed=整数（例如 seed=42），即可复现同一张地图。

## 文件说明
| 文件 | 功能描述 |
|------|----------|
| `env.py` | 环境定义、障碍物生成逻辑、随机地图生成器 |
| `plotting.py` | 基于 Matplotlib 的可视化工具，负责绘制地图、探索过程和路径 |
| `A_pratice_original.py` | A* 算法核心实现与主程序入口 |

---

## 关于本项目
本项目为路径规划算法的学习练习代码，基于经典的 A* (A-Star) 算法实现。
