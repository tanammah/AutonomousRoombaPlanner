from Roomba import Roomba
from state import State
import math
import numpy as np

roomba = Roomba()

# maps = ['map' + str(i) + '.npz' for i in range(1,10)]

# for i in range(len(maps)):
# 	print("-------------- Map {} --------------------------".format(maps[i]))
# 	d = np.load(maps[i])
# 	obstacle_grid = d['gridmap']
# 	startx, starty, start_theta = d['start']
# 	goalx, goaly, goal_theta = d['goal']
# 	startx = int(startx)
# 	starty = int(starty)
# 	goalx = int(goalx)
# 	goaly = int(goaly)
# 	start = startx, starty, math.radians(start_theta)
# 	goal = goalx, goaly, math.radians(goal_theta)
# 	roomba.setStart(start)
# 	roomba.setGoal(goal)
# 	roomba.setMap(obstacle_grid)
# 	path = roomba.findPath()

with open('results.txt', 'r') as f:
    res_high = []
    res_low = []
    for line in f.readlines():
        if 'expansions' not in line:
            continue
        if 'high' in line and 'epsilon=5' in line:
            res_high.append(float(line[line.index(':')+1:]))
        if 'low' in line and 'epsilon=5)' in line:
            res_low.append(float(line[line.index(':')+1:]))
    
    print(sum(res_high)/len(res_high))
    print(sum(res_low)/len(res_low))
