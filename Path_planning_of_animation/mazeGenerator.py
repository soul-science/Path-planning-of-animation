"""
    TODO:
        迷宫生成器
            func1: 制作一个n*n的方形矩阵
            func2: 阔以通过输入设置起点和终点
            func3: 阔以通过点击来设置障碍物
        寻路测试器:
            func1: 包含迷宫生成器
            func2: 阔以设置寻路算法
            func3: 根据算法模拟寻路过程，并且进行算法分析
        选择库:
            GUI: Tkinter and Matplotlib
            Analysis: PPA
            Image: PIL, Numpy

    初步思路:
        top1: 使用tkinter 与 matplotlib 配合完成迷宫的绘制(包含障碍物的设置和清除，起点和终点的设置)，路径规划动画化的推演
        top2: 对灰度图片的推演.
        top3: 没想到
"""

import ctypes
import inspect
import time

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, font
import matplotlib.pyplot as plt
from matplotlib.pylab import mpl
import matplotlib

from PIL import Image, ImageTk
import numpy as np

from threading import Thread
from tqueue import Queue

from ppa import PPA

mpl.rcParams['font.sans-serif'] = ['SimHei']  # 中文显示
mpl.rcParams['axes.unicode_minus'] = False  # 负号显示
plt.style.use('seaborn-whitegrid')

font_label = {
    'family': 'Times New Roman',
    'weight': 'normal',
    'size': 9,
}

matplotlib.use("TkAgg")
plt.ion()
plt.show(block=False)


def mypause(interval):
    backend = plt.rcParams['backend']

    if backend in matplotlib.rcsetup.interactive_bk:
        figManager = matplotlib._pylab_helpers.Gcf.get_active()
        if figManager is not None:
            canvas = figManager.canvas
            if canvas.figure.stale:
                canvas.draw()
            canvas.start_event_loop(interval)
            return


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


class Maze(object):
    def __init__(self, parent, width, height, num, grid_off=False):
        self.parent = parent
        self.width = width
        self.height = height
        self.queue = Queue()
        self.start = None
        self.end = None
        self.barriers = []
        self.barriers_pos = []
        self.gone = []
        self.flag = False
        self.num = num
        self.grid_off = grid_off

        self.analyzer = None

        self.fig = plt.figure(num=num, figsize=(self.width // 3, self.height // 3))
        self.ax = self.fig.add_subplot(111)
        self.fig.canvas.mpl_connect('close_event', self.close)
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.mpl_connect('motion_notify_event', self.notify)
        self.fig.canvas.mpl_connect('scroll_event', self.scroll)
        self.fig.canvas.manager.window.protocol("WM_DELETE_WINDOW", self.close)

    def initialize(self):
        self.ax.format_coord = lambda x, y: ''
        self.ax.set_title('maze_{num}'.format(num=self.num))
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_xticks((0, self.width))
        self.ax.set_yticks((0, self.height))

        if self.grid_off is False:
            miloc = plt.MultipleLocator(1)
            self.ax.xaxis.set_minor_locator(miloc)
            self.ax.yaxis.set_minor_locator(miloc)
            self.ax.grid(axis='both', which='minor')
        else:
            self.ax.grid(False)

        plt.show()

    def set(self, barriers=None):
        self.barriers = barriers

    def draw_circle(self, point, color):
        circle = plt.Circle(point, 0.43, color=color)
        self.ax.add_artist(circle)

    def draw_line(self, points, color):
        x = []
        y = []
        line = plt.Line2D([], [], color=color)
        self.ax.add_line(line)
        for point in points:
            x.append(point[0])
            y.append(point[1])
            line.set_data(x, y)

        plt.show()

    def draw_rect(self, point, color):
        rect = plt.Rectangle(np.floor(point), width=1, height=1, color=color)
        self.ax.add_artist(rect)

    @staticmethod
    def cmp(a, b):
        return a if a is None or b[1] > a[1] else [a[0], a[1] - 1]

    @staticmethod
    def cmp_(a, b):
        return 0 if a is None or b[1] > a[1] else -1

    def onclick(self, event):
        if event.xdata is not None and event.ydata is not None:
            if event.button == 1:
                if self.start is not None:
                    self.end = self.cmp(self.end, self.start)
                    for i in range(len(self.barriers)):
                        self.barriers[i], self.barriers_pos[i] = self.cmp([self.barriers[i], self.barriers_pos[i]],
                                                                          self.start)
                    for i in range(len(self.gone)):
                        self.gone[i] = self.cmp(self.gone[i], self.start)

                    self.ax.artists.pop(self.start[1])

                self.start = [tuple(np.floor([event.xdata, event.ydata]) + 0.5), len(self.ax.artists)]
                self.draw_circle(self.start[0], 'b')

            elif event.button == 3:
                if self.end is not None:
                    self.start = self.cmp(self.start, self.end)
                    for i in range(len(self.barriers)):
                        self.barriers[i], self.barriers_pos[i] = self.cmp([self.barriers[i], self.barriers_pos[i]],
                                                                          self.end)
                        for i in range(len(self.gone)):
                            self.gone[i] = self.cmp(self.gone[i], self.end)
                    self.ax.artists.pop(self.end[1])

                self.end = [tuple(np.floor([event.xdata, event.ydata]) + 0.5), len(self.ax.artists)]
                self.draw_circle(self.end[0], 'g')

        plt.show()

    def notify(self, event):
        if event.button == 2:
            point = tuple(np.floor([event.xdata, event.ydata]) + 0.5)
            try:
                index = self.barriers.index(point)
            except ValueError:
                index = -1

            if index == -1:
                self.barriers.append(point)
                self.barriers_pos.append(len(self.ax.artists))
                self.draw_rect(point, 'k')
            plt.show()

    def scroll(self, event):
        point = tuple(np.floor([event.xdata, event.ydata]) + 0.5)

        try:
            index = self.barriers.index(point)
        except ValueError:
            index = -1

        if index != -1:
            self.start = self.cmp(self.start, [self.barriers[index], self.barriers_pos[index]])
            self.end = self.cmp(self.end, [self.barriers[index], self.barriers_pos[index]])
            for i in range(len(self.gone)):
                self.gone[i] = self.cmp(self.gone[i], [self.barriers[index], self.barriers_pos[index]])
            for i in range(len(self.barriers)):
                if i != index:
                    self.barriers[i], self.barriers_pos[i] = self.cmp([self.barriers[i], self.barriers_pos[i]],
                                                                      [self.barriers[index], self.barriers_pos[index]])
            self.ax.artists.pop(self.barriers_pos[index])
            del self.barriers[index], self.barriers_pos[index]

            plt.show()

    def clear(self):
        start = 0
        end = 0
        barriers = [0] * len(self.barriers)
        if self.gone != []:
            for each in self.gone:
                start += self.cmp_(self.start, each)
                end += self.cmp_(self.end, each)
                for i in range(len(self.barriers)):
                    barriers[i] += self.cmp_([self.barriers[i], self.barriers_pos[i]], each)

        self.start[1] += start
        self.end[1] += end
        self.barriers_pos = [self.barriers_pos[i] + barriers[i] for i in range(len(barriers))]

        while self.gone != []:
            m = self.gone.index(max(self.gone, key=lambda x: x[1]))
            self.ax.artists.pop(self.gone.pop(m)[1])

        self.ax.lines = []

        plt.show()

    def draw(self):
        if not self.queue.empty():
            points, flag = self.queue.get()
            if flag == 0:
                for point in points:
                    if point != self.start[0] and point != self.end[0] and point not in [each[0] for each in self.gone]:
                        self.gone.append([point, len(self.ax.artists)])
                        self.draw_circle(point, 'y')
            elif flag == 1:
                self.draw_line(points, 'r')

            else:
                self.parent.logs_add(*points)

        mypause(0.001)

    def loop(self):
        self.initialize()
        while self.flag is False:
            self.draw()

    def close(self, event=1):
        self.flag = True
        if self.analyzer is not None and self.analyzer.is_alive():
            stop_thread(self.analyzer)
        plt.close(fig=self.num)

    def run(self, method, sqrt):
        self.analyzer = PathAnalyzer(
            queue=self.queue,
            size=(self.width, self.height),
            start=self.start[0], end=self.end[0],
            barriers=self.barriers,
            sqrt=sqrt, method=method)
        self.analyzer.start()


class MazeGenerator(object):
    normal = {'__module__', '__init__', 'set', '__dict__', '__weakref__', '__doc__', '_PPA__bfs', '_PPA__dfs'}

    def __init__(self):
        self.main = tk.Tk()  # 创建主窗口
        self.width, self.height = 350, 450  # 获取此时窗口大小
        self.methods = tuple(PPA.__dict__.keys() - MazeGenerator.normal)

        self.func_value = tk.StringVar()
        self.func_value.set(self.methods[0])
        self.distance_value = tk.IntVar()
        self.distance_value.set(1)

        self.w = tk.IntVar()
        self.w.set(10)
        self.h = tk.IntVar()
        self.h.set(10)

        self.logs_box = tk.Listbox(self.main, width=35, height=4, relief="solid", border=1)
        self.maze = None
        self.num = 0

    def initialize(self):
        self.main.title("Path planning of animation")  # 窗口标题
        self.main.iconbitmap("./favicon.ico")
        self.main.withdraw()  # 隐藏窗口
        self.main.update_idletasks()  # 刷新窗口
        self.main.geometry('%dx%d+%d+%d' % (self.width, self.height,
                                            (self.main.winfo_screenwidth() - self.width) * 0.5,
                                            (self.main.winfo_screenheight() - self.height) * 0.3))  # 窗口位置居中

        self.logs_box.place(x=50, y=330)

        favicon = ImageTk.PhotoImage(file="./favicon.ico")
        ico_label = tk.Label(self.main, image=favicon)
        ico_label.image = favicon

        ft = font.Font(family='华文隶书', size=13, weight=font.BOLD, slant=font.ITALIC, underline=1)
        title_label = tk.Label(self.main, text="Path planning of animation", font=ft)

        func_label = ttk.Label(self.main, text="激活函数：")
        func_choice = ttk.Combobox(self.main, textvariable=self.func_value)
        func_choice["values"] = self.methods
        func_choice.current(0)

        distance_label = ttk.Label(self.main, text="距离方式：")
        distance_choice = ttk.Combobox(self.main, textvariable=self.distance_value)
        distance_choice["values"] = [1, 2]
        distance_choice.current(0)

        height_label = ttk.Label(self.main, text="迷宫高度：")
        height_entry = tk.Entry(self.main, width=5, relief="solid", textvariable=self.h)

        width_label = ttk.Label(self.main, text="迷宫宽度：")
        width_entry = tk.Entry(self.main, width=5, relief="solid", textvariable=self.w)

        maze_btn = ttk.Button(self.main, text="创建空白迷宫", width=33, command=self.create_maze)
        image_btn = ttk.Button(self.main, text="载入图片(迷宫)", width=33, command=self.create_image_maze)
        clear_btn = ttk.Button(self.main, text="清除迷宫", width=33, command=self.clear_maze)
        run_btn = ttk.Button(self.main, text="运行路径规划算法", width=33, command=self.run_method)

        ico_label.place(x=50, y=5)
        title_label.place(x=95, y=15)
        func_label.place(x=55, y=50)
        func_choice.place(x=130, y=50)
        distance_label.place(x=55, y=90)
        distance_choice.place(x=130, y=90)
        height_label.place(x=55, y=130)
        height_entry.place(x=130, y=130)
        width_label.place(x=180, y=130)
        width_entry.place(x=255, y=130)
        maze_btn.place(x=55, y=170)
        image_btn.place(x=55, y=210)
        clear_btn.place(x=55, y=250)
        run_btn.place(x=55, y=290)

        self.main.resizable(0, 0)  # 阻止GUI大小调整
        self.main.deiconify()  # 显示窗口

    def create_maze(self):
        if self.maze is not None and plt.fignum_exists(self.num):
            self.maze.close()
            plt.close(fig=self.num)

        self.num += 1
        self.maze = Maze(self, self.w.get(), self.h.get(), self.num)
        self.maze.loop()

    def create_image_maze(self):
        if self.maze is not None and plt.fignum_exists(self.num):
            self.maze.close()
            plt.close(fig=self.num)

    def logs_add(self, func, time_, length, path):
        self.logs_box.insert(
            "end",
            "maze(func: {func}, time: {time}, length: {length}, path: {path})".format(
                func=func,
                time=time_,
                length=length,
                path=path,
            ))
        self.logs_box.see("end")

    def clear_maze(self):
        if self.maze is not None and plt.fignum_exists(self.num):
            self.maze.clear()

    def run_method(self):
        if self.maze is not None and plt.fignum_exists(self.num):
            if self.maze.analyzer is not None and self.maze.analyzer.is_alive():
                stop_thread(self.maze.analyzer)
            self.maze.run(method=self.func_value.get(), sqrt=self.distance_value.get())

    def quit(self):
        if messagebox.askokcancel("提示", "确定要关闭窗口吗?"):
            self.main.destroy()

    def run(self):
        self.initialize()

    def loop(self):
        self.run()
        self.main.mainloop()  # 显示主窗口


class PathAnalyzer(Thread):
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


if __name__ == '__main__':
    generator = MazeGenerator()
    generator.loop()
