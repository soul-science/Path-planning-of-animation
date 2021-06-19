# Path_planning_of_animation —— by Shao

### **TODO**(刚开始的思路):

####     迷宫生成器(mazeGenerator.py):

#####         **func1: 制作一个n*n的方形矩阵**

#####         **func2: 阔以通过输入设置起点和终点**

#####         **func3: 阔以通过点击来设置障碍物**

####     寻路测试器(pathAnalyzer.py):

#####         **func1: 包含迷宫生成器**

#####         **func2: 阔以设置寻路算法**

#####         **func3: 根据算法模拟寻路过程，并且进行算法分析**

####     路径规划算法集合(ppa.py):

#####         **contains:**

#####             **Dijkstra**

#####             **BFS(Best First Search)**

#####             **BFS**

#####             **DFS**

#####             **A-Star**

#####             **DA-Star**

#####             **Faa(No Barriers)**

#####             **JPS(Jump Search)**

####     python库:

#####         **GUI: tkinter, matplotlib**

#####         **Logic: math, bisect, numpy**

#####         **Image: PIL**

#####         **Task: threading, multiprocessing**

#####         **Others: time, win32api, ctypes, inspect**

####     思路:

​        **top1: 使用tkinter 与 matplotlib 配合完成迷宫的绘制(包含障碍物的设置和清除，起点和终点的设置)，路径规划动画化的推演 ✔**

#####         **top2: 对灰度图片的推演 ✔**

#####         **top3: 一些自创算法(目前只有faa这一个,,,害) ❌**

####     待做:

#####         **1. 迷宫生成器**

#####         **2. 继续优化迷宫的速度和承载最大迷宫的size**

#### 	迷宫操作:

##### 		**1. 迷宫鼠标左键点击设置起点，右键设置终点**

##### 		**2. 按住鼠标滑轮在迷宫中移动设置障碍物，在障碍物上滚动滑轮则取消障碍物**

##### 		**3. 距离方式：1为曼哈顿距离，2为欧氏距离**

##### 		**4. 目前如果找不到路径不会通知(可以在地图中看出来)，待改进**

###### [注意]: 迷宫不要开太大，控制在: width~(0, 100], height~(0, 100] (太多cpu要炸)

### 程序主界面(理工男UI)

<img src="https://github.com/soul-science/Path-planning-of-animation/blob/main/images/image1.png?raw=true" width="300" height="450" alt="image1.png" style="float: left;" />

### 演示动画
<img src="https://github.com/soul-science/Path-planning-of-animation/blob/main/images/video1.gif?raw=true" width="739" height="481" alt="video1.png" style="float: left;" />
<!-- ![video1.gif](https://github.com/soul-science/Path-planning-of-animation/blob/main/images/video1.gif?raw=true) -->

