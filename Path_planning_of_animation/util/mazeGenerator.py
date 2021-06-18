"""
    Module: mazeGenerator
    Author: ShaoHaozhou
    motto: Self-discipline, self-improvement, self-love
    Date: 2021/6/18
    Introduce: A class called Maze Generator, having all functions in order to realize the visual path optimization,
        includes all packages written by author.
    介绍: 一个称为迷宫生成器的类，具有实现可视化的路径优化的所有功能。
"""


import tkinter as tk
from tkinter import ttk, filedialog, messagebox, font
import matplotlib.pyplot as plt
from matplotlib.pylab import mpl
import matplotlib

from PIL import Image, ImageTk
import numpy as np


from .pathAnalyzer import PathAnalyzer
from .processkiller import stop_process
from .ppa import PPA
from multiprocessing import Queue


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


class Maze(object):
    def __init__(self, parent, width, height, num, img=None, grid_off=False):
        self.parent = parent
        self.width = width
        self.height = height
        self.send, self.receive = parent.send, parent.receive
        self.start = None
        self.end = None
        self.barriers = []
        self.barriers_pos = []
        self.gone = []
        self.flag = False
        self.num = num
        self.img = img
        self.grid_off = grid_off

        self.fig = plt.figure(num=num, figsize=(self.width // 3, self.height // 3))
        self.fig.canvas.manager.set_window_title('maze_{num}'.format(num=self.num))
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

        if not self.grid_off:
            xmiloc = plt.MultipleLocator(1)
            ymiloc = plt.MultipleLocator(1)
            self.ax.xaxis.set_minor_locator(xmiloc)
            self.ax.yaxis.set_minor_locator(ymiloc)
            self.ax.grid(axis='both', which='minor')
        else:
            self.ax.grid(False)

        if self.img is not None:
            for i in range(self.height):
                for j in range(self.width):
                    if self.img[i][j] != 0:
                        rgb = self.img[i][j] / 255
                        point = tuple(np.floor([j, self.height - i]) + 0.5)
                        self.barriers.append(point)
                        self.barriers_pos.append(len(self.ax.artists))
                        self.draw_rect(point, (rgb, rgb, rgb))

        plt.show()

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

    def scroll(self, event):
        point = tuple(np.floor([event.xdata, event.ydata]) + 0.5)

        try:
            index = self.barriers.index(point)
        except ValueError:
            index = -1

        if index != -1:
            print(point)
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
        if not self.send.empty():
            points, flag = self.send.get_nowait()
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

    def close(self, event=0):
        self.flag = True
        self.receive.put(["stop_thread", 0])
        plt.close(fig=self.num)

    def run(self, sqrt, method):
        self.receive.put([[(self.width, self.height), self.start[0], self.end[0], self.barriers, sqrt, method], 1])


class MazeGenerator(object):
    normal = {
        '__module__', '__init__', 'set', '__dict__', '__weakref__', '__doc__',
        '_PPA__bfs', '_PPA__dfs', 'd', 'g', 'h', '_PPA__search_jump', '_PPA__horizon_do',
        '_PPA__get_enforce_neighbor', '_PPA__corner_do'}

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

        self.send, self.receive = Queue(), Queue()

        self.analyzer = PathAnalyzer(
            send=self.send,
            receive=self.receive
        )
        self.analyzer.start()

    def initialize(self):
        self.main.title("Path planning of animation")  # 窗口标题
        self.main.iconbitmap("./util/favicon.ico")
        self.main.protocol("WM_DELETE_WINDOW", self.quit)
        self.main.withdraw()  # 隐藏窗口
        self.main.update_idletasks()  # 刷新窗口
        self.main.geometry('%dx%d+%d+%d' % (self.width, self.height,
                                            (self.main.winfo_screenwidth() - self.width) * 0.5,
                                            (self.main.winfo_screenheight() - self.height) * 0.3))  # 窗口位置居中

        self.logs_box.place(x=50, y=330)

        favicon = ImageTk.PhotoImage(file="./util/favicon.ico")
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

        self.num += 1
        self.maze = Maze(self, self.w.get(), self.h.get(), self.num)
        self.maze.loop()

    def create_image_maze(self):
        if self.maze is not None and plt.fignum_exists(self.num):
            self.maze.close()
        img = Image.open(filedialog.askopenfilename(title=u'选择图片')).convert('L')
        img = np.array(img.resize((img.size[0]//2, img.size[1]//2)))
        self.num += 1
        self.maze = Maze(self, img.shape[1], img.shape[0], self.num, img=img)
        self.maze.loop()


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
            try:
                self.maze.run(method=self.func_value.get(), sqrt=self.distance_value.get())
            except Exception:
                messagebox.showerror(title="错误", message="请检查起点和终点是否放置!")
        else:
            messagebox.showwarning(title="警告", message="还未创建迷宫, 请先创建!")

    def quit(self):
        if messagebox.askokcancel("提示", "确定要关闭窗口吗?"):
            if self.maze is not None and plt.fignum_exists(self.num):
                self.maze.close()
            stop_process(self.analyzer.pid)
            self.main.destroy()

    def run(self):
        self.initialize()

    def loop(self):
        self.run()
        self.main.mainloop()  # 显示主窗口


if __name__ == '__main__':
    generator = MazeGenerator()
    generator.loop()
