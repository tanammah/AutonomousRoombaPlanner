from Roomba import Roomba
from State import State
import math
from queue import PriorityQueue


roomba = Roomba()
roomba.setStart((4, 5, 0))
roomba.setMap(...) # input map here
roomba.setGoal((12, 50, 7*math.pi/4))

path = roomba.findPath()
print("path is ", path)

