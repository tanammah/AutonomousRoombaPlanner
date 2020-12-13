from Roomba import Roomba
from State import State
import math
from queue import PriorityQueue


roomba = Roomba()
roomba.setStart((4, 5, 0))
roomba.setGoal((12, 8, 7*math.pi/4))

path = roomba.findPath()
print("path is ", path)


"""
pq = PriorityQueue()
s = State(1,1,1)
s2 = State(2,2,2)
pq.put((1,s))
pq.put((1,s2))
"""

