import heapq

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = sys.maxsize
        self.parent = None

import numpy as np
import sys
import time
from matplotlib.patches import Rectangle

class RandomMap:
    def __init__(self, size=50):
        self.size = size
        self.obstacle = size//8
        self.GenerateObstacle()

    def GenerateObstacle(self):
        self.obstacle_point = set()
        self.obstacle_point.add((self.size//2, self.size//2))
        self.obstacle_point.add((self.size//2, self.size//2-1))

        # Generate an obstacle in the middle
        for i in range(self.size//2-4, self.size//2):
            self.obstacle_point.add((i, self.size-i))
            self.obstacle_point.add((i, self.size-i-1))
            self.obstacle_point.add((self.size-i, i))
            self.obstacle_point.add((self.size-i, i-1))

        for i in range(self.obstacle-3):
            x = np.random.randint(0, self.size)
            y = np.random.randint(0, self.size)
            self.obstacle_point.add((x, y))

            if (np.random.rand() > 0.5): # Random boolean
                for l in range(self.size//4):
                    self.obstacle_point.add((x, y+l))
                    pass
            else:
                for l in range(self.size//4):
                    self.obstacle_point.add((x+l, y))
                    pass

    def IsObstacle(self, i ,j):
        return (i,j) in self.obstacle_point


import glob
from PIL import Image

class AStar:
    def __init__(self, map):
        self.map=map
        self.open_set = []
        self.close_set = set()

    def BaseCost(self, p):
        x_dis = p.x
        y_dis = p.y
        # Distance to start point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def HeuristicCost(self, p):
        x_dis = self.map.size - 1 - p.x
        y_dis = self.map.size - 1 - p.y
        # Distance to end point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def TotalCost(self, p):
        return self.BaseCost(p) + self.HeuristicCost(p)

    def IsValidPoint(self, x, y):
        if x < 0 or y < 0 or x >= self.map.size or y >= self.map.size:
            return False
        return not self.map.IsObstacle(x, y)

    def IsInOpenList(self, p):
        for s in self.open_set:
            _, _, op = s
            if op.x == p.x and p.y == op.y: return True
        return False

    def IsInCloseList(self, p):
        return (p.x,p.y) in self.close_set

    def IsStartPoint(self, p):
        return p.x == 0 and p.y ==0

    def IsEndPoint(self, p):
        return p.x == self.map.size-1 and p.y == self.map.size-1

    def pushQueue(self, queue, point):
        heapq.heappush(queue, [self.TotalCost(point), hash(point), point])

    def RunAndSaveImage(self, ax, plt):
        start_time = time.time()

        start_point = Point(0, 0)
        start_point.cost = 0

        self.pushQueue(self.open_set, start_point)

        while True:
            if (not self.open_set):
                print('No path found, algorithm failed!!!')
                return
            _, _, p = heapq.heappop(self.open_set)
            rec = Rectangle((p.x, p.y), 1, 1, color='c')
            ax.add_patch(rec)
            self.SaveImage(plt)

            if self.IsEndPoint(p):
                self.BuildPath(p, ax, plt, start_time)

                # Create the frames
                frames = []
                imgs = glob.glob("./pics/*.png")
                for i in imgs:
                    new_frame = Image.open(i)
                    frames.append(new_frame)

                # Save into a GIF file that loops forever
                frames[0].save('png_to_gif.gif', format='GIF',
                               append_images=frames[1:],
                               save_all=True,
                               duration=20, loop=0)
                return

            self.close_set.add((p.x, p.y))

            x, y = p.x, p.y
            xCands = [x-1, x, x+1]
            yCands = [y-1, y, y+1]
            for r in xCands:
                for c in yCands:
                    if r == x and c == y:   continue
                    self.ProcessPoint(r, c, p)

    def SaveImage(self, plt):
        millis = int(round(time.time() * 1000))
        filename = './pics/' + str(millis) + '.png'
        plt.savefig(filename)

    def ProcessPoint(self, x, y, parent):
        p = Point(x, y)
        if not self.IsValidPoint(x, y) or self.IsInCloseList(p):
            return # Do nothing for invalid point, Do nothing for visited point
        print('Process Point [', p.x, ',', p.y, ']', ', cost: ', p.cost)
        if not self.IsInOpenList(p):
            p.parent = parent
            p.cost = self.TotalCost(p)
            self.pushQueue(self.open_set, p)

    def BuildPath(self, p, ax, plt, start_time):
        path = []
        while True:
            path.append(p) # Insert first
            if self.IsStartPoint(p):    break
            else:   p = p.parent
        for p in path[::-1]:
            rec = Rectangle((p.x, p.y), 1, 1, color='g')
            ax.add_patch(rec)
            plt.draw()
            self.SaveImage(plt)
        end_time = time.time()
        print('===== Algorithm finish in', int(end_time-start_time), ' seconds')

import matplotlib.pyplot as plt

plt.figure(figsize=(5, 5))

map = RandomMap()

ax = plt.gca()
ax.set_xlim([0, map.size])
ax.set_ylim([0, map.size])

for i in range(map.size):
    for j in range(map.size):
        if map.IsObstacle(i,j):
            rec = Rectangle((i, j), width=1, height=1, color='gray')
            ax.add_patch(rec)
        else:
            rec = Rectangle((i, j), width=1, height=1, edgecolor='gray', facecolor='w')
            ax.add_patch(rec)

rec = Rectangle((0, 0), width = 1, height = 1, facecolor='b')
ax.add_patch(rec)

rec = Rectangle((map.size-1, map.size-1), width = 1, height = 1, facecolor='r')
ax.add_patch(rec)

plt.axis('equal')
plt.axis('off')
plt.tight_layout()
#plt.show()

a_star = AStar(map)
a_star.RunAndSaveImage(ax, plt)
