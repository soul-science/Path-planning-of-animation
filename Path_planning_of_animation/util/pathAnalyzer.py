"""
    Module: mazeGenerator
    Author: ShaoHaozhou
    motto: Self-discipline, self-improvement, self-love
    Date: 2021/6/18
    Introduce: Two classes, a multiple thread and a multiprocess,
     used to analyze the path of mazes created by users according to the algorithm.
    介绍:
        SingleRunner: 用来运行单次的路径规划
        PathAnalyzer: 作为主程序的子进程进行整个路径规划工作
    初衷:
        为了提高效率才搞了这两个类，，本来搞得多线程，结果一卡一卡的；然后又换成多进程，又不能同时进行了；最后结合起来还阔以。
"""

from multiprocessing import Process
from threading import Thread

import numpy as np
from .ppa import PPA
from .threadkiller import stop_thread
import time


class SingleRunner(Thread):
    """
        单个线程, 运行PPA类进行一次性的路径规划
    """
    def __init__(self, send, size, start, end, barriers, sqrt, method):
        super().__init__()
        self.send = send
        self.method = method
        self.sqrt = sqrt
        self.alg = PPA(send)
        self.alg.set(size, start, end, barriers, sqrt)

    def distance(self, a, b):
        return np.emath.power(abs(a[0] - b[0]) ** self.sqrt + abs(a[1] - b[1]) ** self.sqrt, 1 / self.sqrt)

    def run(self) -> None:
        t = time.time()
        length = 0
        path = eval("self.alg." + self.method + "()")
        t = time.time() - t
        for i in range(len(path) - 1):
            length += self.distance(path[i], path[i + 1])

        self.send.put([[self.method, t, length, path], 2])


class PathAnalyzer(Process):
    """
        根据路径规划算法计算路径, 并且用管道来传送路径信息
    """

    def __init__(self, send, receive):
        super().__init__()
        self.send = send
        self.receive = receive
        self.runner = None

    def do(self, size, start, end, barriers, sqrt, method):
        self.runner = SingleRunner(self.send, size, start, end, barriers, sqrt, method)
        self.runner.start()

    def run(self) -> None:
        while True:
            if self.receive.empty() is False:
                pop = self.receive.get_nowait()
                while not self.send.empty():
                    self.send.get_nowait()
                if self.runner is not None and self.runner.is_alive():
                    stop_thread(self.runner)
                if pop[-1] == 1:
                    self.do(*pop[0])

