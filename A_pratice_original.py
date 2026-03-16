import math
import heapq
import env
import plotting


class Astar:
    def __init__(self, start, goal, heuristic_type="manhattan"):
        self.start = start
        self.goal = goal
        self.heuristic_type = heuristic_type

        self.OPEN = []
        self.CLOSE = []
        self.g = dict()
        self.PARENT = dict()

        # 移除seed参数，与原Env类兼容
        self.Env = env.Env()
        self.motion = self.Env.motions
        self.obstacles = self.Env.obs
        self.x_range = self.Env.x_range
        self.y_range = self.Env.y_range

    def get_neighbor(self, node):
        return [(node[0] + dx, node[1] + dy) for dx, dy in self.motion]

    def is_collision(self, current, neighbor):
        if current in self.obstacles or neighbor in self.obstacles:
            return True

        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]

        if dx != 0 and dy != 0:
            check1 = (current[0] + dx, current[1])
            check2 = (current[0], current[1] + dy)
            if check1 in self.obstacles or check2 in self.obstacles:
                return True

        return False

    def cost(self, s_start, s_goal):
        """
        计算移动代价 (仅使用欧氏距离)
        :param s_start: 起始节点
        :param s_goal: 目标节点
        :return: 移动代价，若碰撞则返回无穷大
        """
        if self.is_collision(s_start, s_goal):
            return math.inf

        # 直接返回欧氏距离，不再进行类型判断
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def heuristic(self, node):
        # 仅保留曼哈顿距离和欧氏距离
        if self.heuristic_type == "manhattan":
            return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])
        elif self.heuristic_type == "euclidean":
            return math.hypot(node[0] - self.goal[0], node[1] - self.goal[1])
        else:
            raise ValueError(f"不支持的启发函数类型：{self.heuristic_type}，可选值：manhattan/euclidean")

    def f_value(self, node):
        return self.g[node] + self.heuristic(node)

    def extract_path(self):
        path = [self.goal]
        current = self.goal
        while True:
            current = self.PARENT[current]
            path.append(current)
            if current == self.start:
                break
        return path[::-1]

    def search(self):
        self.PARENT[self.start] = self.start
        self.g[self.start] = 0
        self.g[self.goal] = math.inf

        heapq.heappush(self.OPEN, (self.f_value(self.start), self.start))

        while self.OPEN:
            _, current_node = heapq.heappop(self.OPEN)
            self.CLOSE.append(current_node)

            if current_node == self.goal:
                break

            for neighbor in self.get_neighbor(current_node):
                new_g = self.g[current_node] + self.cost(current_node, neighbor)
                if neighbor not in self.g:
                    self.g[neighbor] = math.inf
                if new_g < self.g[neighbor]:
                    self.g[neighbor] = new_g
                    self.PARENT[neighbor] = current_node
                    heapq.heappush(self.OPEN, (self.f_value(neighbor), neighbor))

        return self.extract_path(), self.CLOSE


def main():
    start = (5, 5)
    goal = (45, 25)

    # 仅保留manhattan和euclidean选项
    astar = Astar(start, goal, heuristic_type="euclidean")
    # 移除第三个参数，与原Plotting类兼容
    plot = plotting.Plotting(start, goal, astar.obstacles)

    path, visited = astar.search()
    plot.animation(path, visited, "A*")


if __name__ == "__main__":
    main()