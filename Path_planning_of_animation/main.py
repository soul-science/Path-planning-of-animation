"""
    主程序 py文件

    TODO(刚开始的思路):
        迷宫生成器(mazeGenerator.py):
            func1: 制作一个n*n的方形矩阵
            func2: 阔以通过输入设置起点和终点
            func3: 阔以通过点击来设置障碍物
        寻路测试器(pathAnalyzer.py):
            func1: 包含迷宫生成器
            func2: 阔以设置寻路算法
            func3: 根据算法模拟寻路过程，并且进行算法分析
        路径规划算法集合(ppa.py):
            contains:
                Dijkstra
                BFS(Best First Search)
                BFS
                DFS
                A-Star
                DA-Star
                Faa(No Barriers)
                JPS(Jump Search)
        python库:
            GUI: tkinter, matplotlib
            Logic: math, bisect, numpy
            Image: PIL
            Task: threading, multiprocessing
            others: time, win32api, ctypes, inspect
        思路:
            top1: 使用tkinter 与 matplotlib 配合完成迷宫的绘制(包含障碍物的设置和清除，起点和终点的设置)，路径规划动画化的推演 ✔
            top2: 对灰度图片的推演 ✔
            top3: 一些自创算法(目前只有faa这一个,,,害) ❌
        待做:
            1. 迷宫生成器
            2. 继续优化迷宫的速度和承载最大迷宫的size
        [注意]: 迷宫不要开太大，控制在: width~(0, 100], height~(0, 100] (太多cpu要炸)
"""

from util.mazeGenerator import MazeGenerator


if __name__ == '__main__':
    maze_generator = MazeGenerator()
    maze_generator.loop()
