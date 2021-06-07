"""
    A class containing a great deal of path planning algorithms is used to calculates the best path and distance from start to end.
    TODO:
        路径规划算法:
            func:
                Dijkstra
                BFS
                DFS
                A-Star
                DA-Star
                Faa(No Barriers)
                JPS(Jump Search)
            arg:
                start: (x, y)
                end: (x, y)
                rect: [(x1, y1), (x2, y2), ..., (xn, yn)]
"""

import math
import bisect


class PPA(object):
    """
        路径规划算法集合(PPA)
        Contains:
            Dijkstra
            BFS
            DFS
            A-Star
            DA-Star
            Faa(No Barriers)
            JPS(Jump Search)
    """
    def __init__(self, queue):
        self.queue = queue
        self.start = None
        self.end = None
        self.size = None
        self.barriers = None
        self.sqrt = None
        self.d = lambda a, b: math.pow(abs(a[0] - b[0]) ** self.sqrt + abs(a[1] - b[1]) ** self.sqrt, 1 / self.sqrt)
        self.g = lambda a: math.pow(abs(a[0] - self.start[0]) ** self.sqrt + abs(a[1] - self.start[1]) ** self.sqrt,
                                    1 / self.sqrt)
        self.h = lambda a: math.pow(abs(a[0] - self.end[0]) ** self.sqrt + abs(a[1] - self.end[1]) ** self.sqrt,
                                    1 / self.sqrt)
        self.directions = {1: [[-1, 0], [1, 0], [0, -1], [0, 1]],
                           2: [[-1, 1], [-1, 0], [-1, -1], [1, 0], [1, 1], [0, 1], [1, -1], [0, -1]]}

    def set(self, size, start, end, barriers, sqrt=1):
        """
            设置寻路算法的参数:
                size: 迷宫大小
                start: 起点
                end: 终点
                barriers: 障碍物
                sqrt: 距离sqrt(次方)
        """
        self.start = start
        self.end = end
        self.size = size
        self.barriers = [[0] * self.size[1] for _ in range(self.size[0])]
        for barrier in barriers:
            self.barriers[math.floor(barrier[0])][math.floor(barrier[1])] = 1
        self.sqrt = sqrt

    def dijkstra(self):
        """
            func: 单源dijkstra算法
            step:
                step0: 开始循环
                for:
                    for:
                        step1: 判断是否出界, 是否为障碍物
                        step2: 得到从起始点到该点的距离(父节点到该节点距离)
                            1) line(父) + g(point)
                        step3: 添加到备选列表中standby中

                    step5: 选择standby中最小的节点min
                    step4: 判断在line中是否有该节点，如果无该节点，则添加进入line，如果有则比较:
                        if > : => 不管
                        if < : => 替换父节点
                    step5: 如果该节点为end则退出循环

                step6: 计算路径
        """
        line = {self.start: [None, 0]}
        standby = []
        vis = [[0] * self.size[1] for _ in range(self.size[0])]

        current = self.start

        while current != self.end:
            x = math.floor(current[0])
            y = math.floor(current[1])
            if vis[x][y] != -1 and current and self.barriers[x][y] != 1:
                points = []
                for direction in self.directions[self.sqrt]:
                    point = (current[0] + direction[0], current[1] + direction[1])
                    p_x = math.floor(point[0])
                    p_y = math.floor(point[1])
                    if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] \
                            and self.barriers[p_x][p_y] != 1 and vis[p_x][p_y] != -1:
                        bisect.insort(standby, [line[current][1] + self.d(point, current), tuple(current), point])

                        points.append(point)
                if points != []:
                    self.queue.put([points, 0])

            best = standby[0]
            standby = standby[1:]

            if best[0] < line.get(best[2], [None, math.inf])[1]:
                line[best[2]] = [best[1], best[0]]
                current = best[2]
            vis[x][y] = -1

        path = [self.end]

        while path[-1] != self.start:
            path.append(line.get(path[-1])[0])

        self.queue.put([path[::-1], 1])

        return path[::-1]

    def bfs(self):
        """
            func: bfs(广度优先算法)
            step:
                step1: 初始化vis和path
                step2: 进行循环, 运行子算法__bfs
                    if 路径集的最后一个的最后一个点为终点 => break
                    else: => continue
                step3: 在所有到达终点的路径中找出最优, 即为最优(根据bfs特性此最优为全局最优)
                step4: 返回最优路径
        """
        vis = [[0] * self.size[1] for _ in range(self.size[0])]
        path = [[self.start]]

        while path[-1][-1] != self.end:
            path = self.__bfs(path=path, vis=vis)

        m_path = [None, math.inf]

        for each in path[:-1]:
            if each[-1] == self.end:
                d = 0
                for i in range(len(each) - 1):
                    d += self.d(each[i], each[i+1])
                if d < m_path[1]:
                    m_path = [each, d]

        self.queue.put([m_path[0], 1])

        return m_path[0]

    def __bfs(self, path, vis):
        """
        func: bfs子算法
        step:
            for:
                step1: 循环path当中的所有路径集去遍历新的适合的路径
                    if 有路径已经到达终点, 则置flag为True => 添加一个结束判断符
                    else: pass
                step2: 返回新的路径集合
        """
        flag = False
        new_path = []
        points = []
        xys = set()
        for each in path:
            x, y = math.floor(each[-1][0]), math.floor(each[-1][1])
            xys.add((x, y))
            if vis[x][y] != 1 and self.barriers[x][y] != 1:
                for direction in self.directions[self.sqrt]:
                    point = (each[-1][0] + direction[0], each[-1][1] + direction[1])
                    p_x, p_y = math.floor(point[0]), math.floor(point[1])
                    if point == self.end:
                        flag = True
                    if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] \
                            and self.barriers[p_x][p_y] != 1 and vis[p_x][p_y] != -1:
                        new = each.copy()
                        new.append(point)
                        new_path.append(new)
                        points.append(point)
            for xy in xys:
                vis[xy[0]][xy[1]] = -1

        self.queue.put([list(set(points)), 0])

        if flag is True:
            new_path.append([self.end])

        return new_path

    def dfs(self):
        """
            func: dfs(深度优先算法)
            step:
                step1: 运行子算法__dfs
                step2: 返回子算法得到的最优路径
        """
        return self.__dfs([self.start])

    def __dfs(self, path):
        """
            func: __dfs(dfs子算法)
            step:
                step1: 判断路径最后一个点是否为终点
                    if True: => 返回路径
                    else: => pass
                for: 对各个方向进行一个循环
                    step2: 判断点是否合法
                        if True: => 递归 __dfs(path)
                        else False: => continue
                    step3: 判断first是否返回了路径
                        if True: => 返回路径集合(沿着递归方向)
                        else: continue
        """
        if path[-1] == self.end:
            self.queue.put([path, 1])
            return path

        for direction in self.directions[self.sqrt]:
            point = (path[-1][0] + direction[0], path[-1][1] + direction[1])
            p_x, p_y = math.floor(point[0]), math.floor(point[1])
            if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] and self.barriers[p_x][p_y] != 1 and point not in path:
                path.append(point)
                self.queue.put([[point], 0])
                first = self.__dfs(path.copy())
                if first is not None:
                    return first
                path.pop()

    def best_fs(self):
        """
            func: best_first_search(最佳优先算法)
            step1: 初始化
            while: 当前点不为终点进行
                for: 对于当前节点的各个方向进行循环
                    step2: 判断新点是否符合
                        if True:
                            step3: 将点按启发函数g(x)的值由小到大加入到standby列表中
                        else: continue
                    step4: 从standby列表中弹出第一个点(启发值最小点)加入到最优路径中
                    step5: 替换当前节点
            step5: 返回得到的路径(无障碍物时必最优)
        """

        vis = [[0] * self.size[1] for _ in range(self.size[0])]

        current = self.start
        path = [current]
        while current != self.end:
            standby = []
            middle = []
            x = math.floor(current[0])
            y = math.floor(current[1])
            for direction in self.directions[self.sqrt]:
                point = (current[0] + direction[0], current[1] + direction[1])
                p_x, p_y = math.floor(point[0]), math.floor(point[1])
                if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] and self.barriers[p_x][p_y] != 1 \
                        and vis[p_x][p_y] != -1:
                    bisect.insort(standby, [self.h(point), tuple(point)])
                    middle.append(tuple(point))
            if middle != []:
                self.queue.put([middle, 0])
            vis[x][y] = -1
            current = standby[0][1]
            path.append(current)

        self.queue.put([path, 1])
        return path

    def astar(self):
        """
            func: A-star(A*算法)
            step:
                step0: 初始化
                while: 同上
                    for: 同上
                        step1: 判断是否出界, 是否为障碍物
                        step2: 得到从起始点到该点的距离(父节点到该节点距离)
                            1) line(父) + g(point) + h(point)
                        step3: 添加到备选列表中standby中

                    step5: 选择standby中最小的节点min
                    step4: 判断在line中是否有该节点，如果无该节点，则添加进入line，如果有则比较:
                        if > : => 不管
                        if < : => 替换父节点
                    step6: 替换当前节点

                step6: 返回最优路径
        """
        line = {self.start: [None, 0]}
        standby = []
        vis = [[0] * self.size[1] for _ in range(self.size[0])]

        current = self.start

        while current != self.end:
            x = math.floor(current[0])
            y = math.floor(current[1])
            if vis[x][y] != -1 and self.barriers[x][y] != 1:
                points = []
                for direction in self.directions[self.sqrt]:
                    point = (current[0] + direction[0], current[1] + direction[1])
                    p_x = math.floor(point[0])
                    p_y = math.floor(point[1])
                    if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] \
                            and self.barriers[p_x][p_y] != 1 and vis[p_x][p_y] != -1:
                        h = self.h(point)
                        bisect.insort(standby,
                                      [line[current][1] + self.d(point, current) + h, h, tuple(current), point])
                        points.append(point)
                if points != []:
                    self.queue.put([points, 0])

            best = standby[0]
            standby = standby[1:]

            if best[0] <= line.get(best[3], [None, math.inf])[1]:
                line[best[3]] = [best[2], best[0] - best[1]]
                current = best[3]
            vis[x][y] = -1
        path = [self.end]

        while path[-1] != self.start:
            path.append(line.get(path[-1])[0])

        self.queue.put([path, 1])
        return path[::-1]

    def dastar(self):
        """
            func: Double A-star(DA*算法)
            step:
                step0: 初始化
                for:
                    step1: 对起点后终点同时同步进行A*寻路算法
                    step2: 判断两边当前的搜索点是否在对方的已搜索列表中
                        if True: => break
                        else: => continue

                step3:  返回最优路径

        """

        line_1 = {self.start: [None, 0]}
        line_2 = {self.end: [None, 0]}

        standby_1 = []
        standby_2 = []

        vis_1, vis_2 = [[[0] * self.size[1] for _ in range(self.size[0])] for __ in range(2)]

        current_1 = self.start
        current_2 = self.end

        while True:
            x_1, y_1 = math.floor(current_1[0]), math.floor(current_1[1])
            x_2, y_2 = math.floor(current_2[0]), math.floor(current_2[1])

            points = []
            for direction in self.directions[self.sqrt]:
                if vis_1[x_1][y_1] != -1 and self.barriers[x_1][y_1] != 1:
                    point = (current_1[0] + direction[0], current_1[1] + direction[1])
                    px_1, py_1 = math.floor(point[0]), math.floor(point[1])
                    if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] and self.barriers[px_1][py_1] != 1 \
                            and vis_1[px_1][py_1] != -1:
                        h = self.h(point)
                        bisect.insort(standby_1,
                                      [line_1[current_1][1] + self.d(point, current_1) + h, h, tuple(current_1), point])
                        points.append(point)

                if vis_2[x_2][y_2] != -1 and self.barriers[x_2][y_2] != 1:
                    point = (current_2[0] + direction[0], current_2[1] + direction[1])
                    px_2, py_2 = math.floor(point[0]), math.floor(point[1])
                    if 0 <= point[0] < self.size[0] and 0 <= point[1] < self.size[1] and self.barriers[px_2][py_2] != 1\
                            and vis_2[px_2][py_2] != -1:
                        h = self.g(point)
                        bisect.insort(standby_2,
                                      [line_2[current_2][1] + self.d(point, current_2) + h, h, tuple(current_2), point])
                        points.append(point)

            if points != []:
                self.queue.put([points, 0])

            best_1, best_2 = standby_1[0], standby_2[0]
            standby_1, standby_2 = standby_1[1:], standby_2[1:]

            if best_1[0] < line_1.get(best_1[3], [None, math.inf])[1]:
                line_1[best_1[3]] = [best_1[2], best_1[0] - best_1[1]]
                current_1 = best_1[3]

            if best_2[0] < line_2.get(best_2[3], [None, math.inf])[1]:
                line_2[best_2[3]] = [best_2[2], best_2[0] - best_2[1]]
                current_2 = best_2[3]

            vis_1[x_1][y_1] = vis_2[x_2][y_2] = -1

            for current in [current_1, current_2]:
                if current in line_1 and current in line_2:
                    path_1 = [current]
                    path_2 = [current]

                    while path_1[-1] != self.start:
                        path_1.append(line_1.get(path_1[-1])[0])

                    while path_2[-1] != self.end:
                        path_2.append(line_2.get(path_2[-1])[0])

                    path = path_1[::-1]
                    path.extend(path_2[1:])

                    self.queue.put([path, 1])

                    return path

    def faa(self):
        """
            func: faa(根据if条件判断找到无障碍时的最短路径算法(仅适用于无障碍路径))
            step:
                step1: 初始化
                step2: 根据当前点与终点的位置关系来进行移动
                    criterion(准则):
                        1. 对于曼哈顿距离: 先上下移动, 后左右移动
                        2. 对于欧氏距离: 先对角方向移动, 后左右移动
                step3: 返回路径
        """
        point = list(self.start)
        path = [self.start]

        while tuple(point) != self.end:
            if point[0] == self.end[0]:
                if point[1] < self.end[1]:
                    point[1] += 1
                elif point[1] > self.end[1]:
                    point[1] -= 1
            elif point[1] == self.end[1]:
                if point[0] < self.end[0]:
                    point[0] += 1
                elif point[0] > self.end[0]:
                    point[0] -= 1
            if self.sqrt == 2:
                if point[1] > self.end[1]:
                    if point[0] < self.end[0]:
                        point[0] += 1
                    else:
                        point[0] -= 1
                    point[1] -= 1
                elif point[1] < self.end[1]:
                    if point[0] < self.end[0]:
                        point[0] += 1
                    else:
                        point[0] -= 1
                    point[1] += 1
            else:
                if point[1] > self.end[1]:
                    point[1] -= 1
                elif point[1] < self.end[1]:
                    point[1] += 1

            self.queue.put([[tuple(point)], 0])

            path.append(tuple(point))

        self.queue.put([path, 1])

        return path

    def jps(self):
        pass
