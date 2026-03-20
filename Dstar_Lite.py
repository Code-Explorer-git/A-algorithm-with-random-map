import heapq
import math
import matplotlib.pyplot as plt
from env import Env
from plotting import Plotting


class DStarLite:
    def __init__(self, s_start, s_goal, env):
        """
        D* Lite 算法初始化
        :param s_start: 起点 (x, y)
        :param s_goal: 终点 (x, y)
        :param env: 环境实例（包含地图、移动方向等）
        """
        self.s_start = s_start  # 起点
        self.s_goal = s_goal  # 终点
        self.env = env  # 环境
        self.motions = env.motions  # 8方向移动规则
        self.obs = env.obs  # 障碍物集合

        # 核心代价参数
        self.g = {}  # 节点到目标的实际代价（反向搜索，初始无穷大）
        self.rhs = {}  # 一步前瞻代价（rhs(s) = min(g(s') + cost(s,s'))）
        self.U = []  # 优先级队列（heapq最小堆实现）
        self.km = 0.0  # 启发式偏移量：仅当起点移动时更新，障碍更新不修改！

        # 初始化所有节点的g和rhs为无穷大，终点的rhs=0（反向搜索的起点）
        for x in range(env.x_range):
            for y in range(env.y_range):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')
        self.rhs[self.s_goal] = 0.0

        # 终点加入优先级队列
        heapq.heappush(self.U, (self.calculate_key(self.s_goal), self.s_goal))

    def calculate_key(self, s):
        """计算优先级队列的key值（核心排序依据）"""
        min_g_rhs = min(self.g[s], self.rhs[s])
        key1 = min_g_rhs + self.heuristic(s, self.s_start) + self.km
        key2 = min_g_rhs
        return (key1, key2)

    def heuristic(self, s1, s2):
        """启发式函数：8方向移动用切比雪夫距离，比欧几里得更适配网格"""
        dx = abs(s1[0] - s2[0])
        dy = abs(s1[1] - s2[1])
        return max(dx, dy)

    def cost(self, s1, s2):
        """计算两节点间移动代价：障碍为无穷大，正交1，斜向√2"""
        if s2 in self.obs or s1 in self.obs:
            return float('inf')
        dx, dy = abs(s1[0] - s2[0]), abs(s1[1] - s2[1])
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_neighbors(self, s):
        """获取节点的8方向有效邻居（不越界）"""
        x, y = s
        neighbors = []
        for dx, dy in self.motions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range:
                neighbors.append((nx, ny))
        return neighbors

    def update_vertex(self, s):
        """更新节点的rhs值，并维护优先级队列（移除过期数据）"""
        if s != self.s_goal:
            # 重新计算rhs：所有邻居的g值+移动代价的最小值
            self.rhs[s] = min(
                self.g[s_prime] + self.cost(s, s_prime)
                for s_prime in self.get_neighbors(s)
            )

        # 移除队列中该节点的旧数据（避免过期值干扰）
        self.U = [(k, node) for k, node in self.U if node != s]
        heapq.heapify(self.U)

        # 节点状态不一致（g≠rhs），加入队列等待处理
        if self.g[s] != self.rhs[s]:
            heapq.heappush(self.U, (self.calculate_key(s), s))

    def compute_shortest_path(self):
        """增量式最短路径计算（D* Lite核心逻辑）"""
        while self.U and (
                # 队列头部节点优先级更高，或起点状态不一致
                self.U[0][0] < self.calculate_key(self.s_start) or
                self.rhs[self.s_start] != self.g[self.s_start]
        ):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)

            # 旧key过期，重新加入队列
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            # 过一致状态：g > rhs，更新g为rhs，同步更新邻居
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            # 欠一致状态：g < rhs，重置g为无穷大，重新更新自身和邻居
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.get_neighbors(u):
                    self.update_vertex(s)

    def find_path(self):
        """从起点生成到目标的正向路径，优化了路径查找逻辑"""
        path = [self.s_start]
        s_current = self.s_start
        max_step = self.env.x_range * self.env.y_range  # 防止死循环
        step = 0

        while s_current != self.s_goal and step < max_step:
            step += 1
            # 选择代价最小的邻居作为下一个节点
            neighbors = self.get_neighbors(s_current)
            # 过滤掉障碍和代价无穷大的节点
            valid_neighbors = [s for s in neighbors if
                               self.cost(s_current, s) < float('inf') and self.rhs[s] < float('inf')]

            if not valid_neighbors:
                print("路径中断：无有效邻居，无法到达目标！")
                return []

            # 选择总代价最小的邻居
            s_next = min(valid_neighbors, key=lambda s: self.rhs[s] + self.cost(s_current, s))
            path.append(s_next)
            s_current = s_next

        if s_current != self.s_goal:
            print("路径生成失败：超出最大步数，无法到达目标！")
            return []

        return path

    def update_obstacles(self, new_obs):
        # 1. 新增障碍到环境和障碍物集合
        for obs in new_obs:
            if 0 <= obs[0] < self.env.x_range and 0 <= obs[1] < self.env.y_range:
                self.obs.add(obs)
        self.env.update_obs(self.obs)

        # 2. 仅更新障碍的邻居节点（增量更新，核心优势）
        update_nodes = set()
        for obs in new_obs:
            for s in self.get_neighbors(obs):
                update_nodes.add(s)

        for s in update_nodes:
            self.update_vertex(s)

        # 3. 增量更新最短路径
        self.compute_shortest_path()

    def move_start(self, new_start):
        """模拟机器人移动，更新起点并调整km"""
        old_start = self.s_start
        self.s_start = new_start
        # 起点移动时更新km，抵消启发式的变化
        self.km += self.heuristic(old_start, new_start)


def main():
    # 1. 初始化环境：seed固定地图，random_map=False用原版简单地图
    # 修改前（固定地图）
    env = Env(seed=10, random_map=False)

    ## 修改后（每次随机新地图）
    #env = Env(seed=None, random_map=True)
    s_start = (5, 5)  # 起点
    s_goal = (45, 25)  # 终点

    # 2. 交互式选择模式
    print("=" * 50)
    print("          D* Lite 算法测试工具")
    print("=" * 50)
    print("请选择运行模式：")
    print("1 - 静态环境路径规划（无动态障碍）")
    print("2 - 动态环境路径规划（新增障碍后重规划）")
    print("0 - 退出程序")
    print("=" * 50)

    while True:
        try:
            choice = int(input("请输入选择（0/1/2）："))
            if choice not in [0, 1, 2]:
                print("输入错误！请输入 0、1 或 2")
                continue
            break
        except ValueError:
            print("输入错误！请输入整数 0、1 或 2")

    # 3. 根据选择执行对应逻辑
    if choice == 0:
        print("程序已退出")
        return

    # 初始化D* Lite和可视化工具
    d_star_lite = DStarLite(s_start, s_goal, env)
    plotting = Plotting(s_start, s_goal, env.obs)

    if choice == 1:
        # 静态环境路径规划
        print("\n=== 静态环境路径规划 ===")
        d_star_lite.compute_shortest_path()
        path_static = d_star_lite.find_path()
        if path_static:
            print(f"静态路径生成成功，路径长度: {len(path_static)}")
        else:
            print("静态路径生成失败（无法到达目标）")
        plotting.animation(path_static, [], "D* Lite (Static Env)")

    elif choice == 2:
        # 第一步：生成静态初始路径
        print("\n=== 第一步：生成静态初始路径 ===")
        d_star_lite.compute_shortest_path()
        path_static = d_star_lite.find_path()
        if not path_static:
            print("初始静态路径生成失败（无法到达目标），终止动态测试")
            return
        print(f"初始静态路径长度: {len(path_static)}")
        plotting.animation(path_static, [], "D* Lite (Static Env - Before Dynamic)")
        plt.close()  # 关闭静态窗口，准备动态展示

        # 第二步：动态添加障碍并重规划【已调整障碍位置，不会完全隔断地图】
        print("\n=== 第二步：动态添加障碍，路径重规划 ===")
        # 【关键修改】调整障碍位置，仅阻断原路径，保留其他通道
        new_obs = set()
        # 仅阻断原路径的核心通道，不封死整个地图
        for x in range(10, 21):
            new_obs.add((x, 16))  # 原路径的上方通道
        for y in range(10, 16):
            new_obs.add((21, y))  # 原路径的右侧通道
        print(f"新增障碍数量：{len(new_obs)} 个")

        # 执行动态更新
        d_star_lite.update_obstacles(new_obs)
        path_dynamic = d_star_lite.find_path()

        # 结果展示
        if path_dynamic:
            print(f"动态重规划路径生成成功，路径长度: {len(path_dynamic)}")
        else:
            print("动态路径生成失败（无法到达目标）")

        # 可视化动态路径
        plotting.update_obs(d_star_lite.obs)
        plotting.animation(path_dynamic, [], "D* Lite (Dynamic Env - After Update)")

    print("\n 测试完成！")


if __name__ == "__main__":
    main()