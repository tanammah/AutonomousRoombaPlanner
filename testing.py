from Roomba import Roomba
from state import State
import math
import numpy as np

roomba = Roomba()
roomba.setStart((4, 5, 0))
roomba.setMap(np.array([[False]*80 for i in range(80)])) # input map here
roomba.setGoal((12, 50, 7*math.pi/4))

path = roomba.findPath()
print("path is ", path)

