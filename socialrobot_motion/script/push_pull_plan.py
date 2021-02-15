from six import with_metaclass 

from socialrobot_motion.srv import *
from std_msgs.msg import *

import numpy as np
import math
from heapq import *

class Singleton(type):
    '''
    for singleton pattern
    '''
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class AStar():
    def __init__(self):
        self._start = None
        self._goal = None
        self._robot_pos = None        
        self._size = (0, 10)
        
        self._map_data = {}
        self.__path = []

        self._neighbors = ((0,1),(0,-1),(1,0),
                        (-1,0),(1,1),(1,-1),
                        (-1,1),(-1,-1))


    def get_heuristic(self, a, b):
        return (math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2))


    def run(self, map_data, start, goal):
        self._start = start
        self._goal = goal
        self._map_data = map_data

        close_set = set()
        came_from = {}
        gscore = {self._start:0}
        fscore = {self._start:self.get_heuristic(self._start, self._goal)}
        oheap = []

        heappush(oheap, (fscore[self._start], self._start))
    
        while oheap:

            current = heappop(oheap)[1]

            if current == self._goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in self._neighbors:
                self._neighbor = current[0] + i, current[1] + j            
                tentative_g_score = gscore[current] + self.get_heuristic(current, self._neighbor)
                if 0 <= self._neighbor[0] < self._map_data.shape[0]:
                    if 0 <= self._neighbor[1] < self._map_data.shape[1]:                
                        if self._map_data[self._neighbor[0]][self._neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                
                if self._neighbor in close_set and tentative_g_score >= gscore.get(self._neighbor, 0):
                    continue
                
                if  tentative_g_score < gscore.get(self._neighbor, 0) or self._neighbor not in [i[1]for i in oheap]:
                    came_from[self._neighbor] = current
                    gscore[self._neighbor] = tentative_g_score
                    fscore[self._neighbor] = tentative_g_score + self.get_heuristic(self._neighbor, self._goal)
                    heappush(oheap, (fscore[self._neighbor], self._neighbor))
                
        return False


class PushPlanner(with_metaclass(Singleton)):
    def __init__(self, **params):
        pass


    def mapMsgToArray(msg):
        return np.array([])



        
    def callback_plan_grasp(self, req):
        res = MotionPlanResponse()
        start = []
        goal = []

        grid_map = req.gridMap
        astar = AStar()
        astar.run(grid_map, start, goal)

        return res


class DragPlanner(with_metaclass(Singleton)):
    def __init__(self, **params):
        pass


    def callback_plan_grasp(self, req):
        res = MotionPlanResponse() 

        return res