# A* Pathfinding with Random Map
#A算法 / D Lite 算法一个基于 Python 的路径规划算法实现，包含经典 A* 算法与 D* Lite 算法，支持随机生成复杂迷宫地图，并通过 Matplotlib 动态可视化搜索过程。
特性
双算法支持：A* (A-Star) 算法 + D* Lite 算法
A* 双启发式函数：可切换 曼哈顿距离 (Manhattan) 或 欧氏距离 (Euclidean)
随机地图生成器：内置算法生成复杂障碍物，每次运行体验不同
可复现性：支持固定随机种子 (Seed)，方便调试和对比实验
动态可视化：实时展示两种算法的探索过程和最终路径
环境要求
Python 3.13
Matplotlib
安装依赖
bash
运行
pip install matplotlib
快速开始
1. 运行 A* 算法（默认配置）
bash
运行
python A_pratice_original.py
2. 运行 D* Lite 算法
bash
运行
python Dstar_Lite.py
3. 修改配置（可选）
修改起点 / 终点：在对应算法主程序中修改 start 和 goal 元组。
固定随机地图：在 env.py 的 Env 类初始化中传入 seed = 整数（例如 seed=42），即可复现同一张地图。
文件说明
表格
文件	功能描述
env.py	环境定义、障碍物生成逻辑、随机地图生成器
plotting.py	基于 Matplotlib 的可视化工具，负责绘制地图、探索过程和路径
A_pratice_original.py	A* 算法核心实现与主程序入口
Dstar_Lite.py	D* Lite 算法核心实现与主程序入口
关于本项目
本项目为路径规划算法的学习练习代码，实现了经典的 A* (A-Star) 算法与 D* Lite 算法，适用于静态环境下的最优路径规划学习与演示。
