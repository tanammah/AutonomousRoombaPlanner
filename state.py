from utils import *
import math

class State:
    "This class represents the search state that will be used for ARA* search"
    def __init__(self, x, y, theta, parent=None, parent_action=None, g=float('inf')):
        self.x = x
        self.y = y
        self.theta = theta % (2*math.pi)
        self.g = g
        self.h = float('inf')
        self.parent = parent
        self.parent_action = parent_action

    def __eq__(self, other):
        if not isinstance(other, State):
            return False
        return (self.x == other.x) and (self.y == other.y) and (almostEqual(self.theta, other.theta))

    def __hash__(self):
        deg = round(math.degrees(self.theta))
        return hash((self.x, self.y, deg))

    def __lt__(self, other):
        return self.g < other.g

    def setG(self, g):
        self.g = g
    def setH(self, h):
        self.h = h
    def setParent(self, parent):
        self.parent = parent
    def setParentAction(self, parent_action):
        self.parent_action = parent_action
