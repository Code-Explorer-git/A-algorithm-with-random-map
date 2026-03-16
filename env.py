import random


class Env:
    def __init__(self, seed=None, random_map=True):
        """
        :param seed: 随机种子，填入整数即可固定地图，None则每次随机
        :param random_map: 是否使用随机地图，False则使用原版简单地图
        """
        self.x_range = 51
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

        # 内部设置随机种子
        self.seed = seed
        self.random_map = random_map
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        初始化障碍物：根据参数选择随机或固定
        """
        x = self.x_range
        y = self.y_range
        obs = set()

        # 1. 必须有的边界围墙
        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))
        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        if not self.random_map:
            # --- 模式A：原版固定地图 ---
            for i in range(10, 21):
                obs.add((i, 15))
            for i in range(15):
                obs.add((20, i))
            for i in range(15, 30):
                obs.add((30, i))
            for i in range(16):
                obs.add((40, i))
            return obs
        else:
            # --- 模式B：复杂随机地图 (默认) ---
            if self.seed is not None:
                random.seed(self.seed)

            # 保护起点(5,5)和终点(45,25)附近区域不生成障碍
            protected = set()
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    protected.add((5 + dx, 5 + dy))
                    protected.add((45 + dx, 25 + dy))

            # 1. 随机生成 8-12 个矩形障碍物
            num_rects = random.randint(8, 12)
            for _ in range(num_rects):
                # 随机矩形大小和位置
                w = random.randint(3, 8)
                h = random.randint(3, 12)
                x0 = random.randint(5, x - w - 5)
                y0 = random.randint(5, y - h - 5)

                # 填充矩形
                for i in range(w):
                    for j in range(h):
                        px = x0 + i
                        py = y0 + j
                        if (px, py) not in protected:
                            obs.add((px, py))

            # 2. 随机生成 3-5 道长墙 (增加迷宫感)
            num_walls = random.randint(3, 5)
            for _ in range(num_walls):
                is_horizontal = random.choice([True, False])
                if is_horizontal:
                    wx = random.randint(5, x - 20)
                    wy = random.randint(5, y - 5)
                    length = random.randint(10, 25)
                    for i in range(length):
                        px = wx + i
                        py = wy
                        if (px, py) not in protected:
                            obs.add((px, py))
                else:
                    wx = random.randint(5, x - 5)
                    wy = random.randint(5, y - 20)
                    length = random.randint(10, 15)
                    for i in range(length):
                        px = wx
                        py = wy + i
                        if (px, py) not in protected:
                            obs.add((px, py))

            return obs