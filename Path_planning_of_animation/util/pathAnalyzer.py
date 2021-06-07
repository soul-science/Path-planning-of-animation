"""
    A class, a multiple thread, is used to analyze the path of mazes created by users according to the algorithm.
"""

from threading import Thread
import numpy as np
from .ppa import PPA
import time


class PathAnalyzer(Thread):
    """
        根据路径规划算法计算路径, 并且用管道来传送路径信息
    """
    def __init__(self, queue, size, start, end, barriers, sqrt, method):
        super().__init__()
        self.queue = queue
        self.method = method
        self.alg = PPA(queue)
        self.alg.set(size, start, end, barriers, sqrt)
        self.distance = lambda a, b: np.emath.power(abs(a[0] - b[0]) ** sqrt + abs(a[1] - b[1]) ** sqrt, 1 / sqrt)

    def run(self) -> None:
        t = time.time()
        length = 0
        path = eval("self.alg." + self.method + "()")
        t = time.time() - t
        for i in range(len(path) - 1):
            length += self.distance(path[i], path[i + 1])

        self.queue.put([[self.method, t, length, path], 2])