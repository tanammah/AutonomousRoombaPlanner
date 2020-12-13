import math
import time
from utils import *
from motion_primitives import *
from queue import PriorityQueue

class Roomba:
    def __init__(self):
        self.map = None
        self.start = None # tuple: (x, y, theta)
        self.goal = None  # tuple: (x, y, theta)
        self.lattice_resolution_dict = self.generateResolutionDict()
        self.max_search_time = 2

        # ARA* related member variables
        self.incons = set()
        self.closed = dict()
        self.open = PriorityQueue()
        self.open_states = dict()
        self.epsilon_init_val = 50
        self.epsilon_decrement = 2
        self.epsilon = self.epsilon_init_val
        self.goal_state = None

    def setMap(self, map_data):
        self.map = map_data

    def setStart(self, start):
        x, y, theta = start
        theta = theta % (2*math.pi)
        self.start = (x, y, theta)

    def setGoal(self, goal):
        x, y, theta = goal
        theta = theta % (2*math.pi)
        self.goal = (x, y, theta)

    def generateResolutionDict(self):
        res_dict = dict()

        res_dict['low'] = [ForwardAction(), ForwardLeftSlightAction(), ForwardRightSlightAction(), BackwardAction()]
        res_dict['high'] = [ForwardAction(), ForwardLeftSlightAction(), ForwardLeftSharpAction(), 
                                ForwardRightSlightAction(), ForwardRightSharpAction(), BackwardAction(), 
                                BackwardLeftSlightAction(), BackwardRightSlightAction()]
        return res_dict

    def findPath(self):
        start_time = time.time()

        start_state = State(self.start[0], self.start[1], self.start[2], g=0)
        start_state.h = self.heuristicFunc(start_state)

        self.goal_state = State(self.goal[0], self.goal[1], self.goal[2], g=float('inf'))

        # initialize data structures
        self.incons = set()
        self.closed = dict()
        self.open = PriorityQueue()
        self.open_states = dict()

        self.epsilon = self.epsilon_init_val

        self.open.put((self.fvalue(start_state), start_state))
        self.open_states[start_state] = start_state

        self.improvePath() 

        curr_time = time.time()

        # path should be found at this point. return a solution if no more time is left
        if ((curr_time - start_time >= self.max_search_time)):
            return self.getSolution()

        while (self.epsilon > 1): # TODO change this condition to use epsilon prime instead (read algorithm)
            self.decreaseEpsilon()

            self.updateOpen()
            self.incons = set()
            self.closed = dict()

            self.improvePath()

            if (self.goal_state and (self.goal_state.g < float('inf'))):
                print("path found for epsilon = {}".format(self.epsilon))

            curr_time = time.time()

            if ((curr_time - start_time >= self.max_search_time)):
                return self.getSolution()

        return self.getSolution()

    def getSolution(self):
        """ returns path if solution is found else None"""
        if self.goal_state and (self.goal_state.g < float('inf')):
            return self.backtrack(self.goal_state)
        return None
    def improvePath(self):
        while (not self.open.empty()):
            min_state = self.open.get()[1]

            if (min_state in self.closed): continue # state could be a dummy node

            if (self.fvalue(self.goal_state) < self.fvalue(min_state)):
                break

            if self.checkGoal(min_state):
                self.goal_state = min_state
                return

            self.closed[min_state] = min_state

            for successor in self.getSuccessors(min_state):
                if successor in self.closed:
                    closed_successor = self.closed[successor]
                    if (successor.g < closed_successor.g):
                        self.addToIncons(successor)
                        self.closed[successor] = successor
                else:
                    if ((successor in self.open_states) and (successor.g >= self.open_states[successor].g)):
                        continue

                    # state not discovered or has been discovered before but is more optimal path than old discovery
                    self.open_states[successor] = successor
                    self.open.put((self.fvalue(successor), successor))

    def updateOpen(self):
        """updates the open list with incons states and updates priorities. Also filters out dummy nodes and updates self.open_states"""
        state_buffer = list(self.incons)

        while (not self.open.empty()):
            state = self.open.get()[1]

            if (state.g > self.open_states[state].g): continue # skip dummy nodes!

            state_buffer.append(state)

        self.open_states = dict()
        for state in state_buffer:
            self.open.put((self.fvalue(state), state))
            self.open_states[state] = state

                    

    def addToIncons(self, state):
        """adds state to incons while replacing an old entry with an updated (better g value) state"""
        if state in self.incons: self.incons.remove(state)
        self.incons.add(state)

    def checkGoal(self, state):
        goal_x = self.goal[0]
        goal_y = self.goal[1]
        goal_theta = self.goal[2]
        return (state.x == goal_x) and (state.y == goal_y) and (almostEqual(goal_theta, state.theta))

    def getSuccessors(self, state):
        sx, sy, stheta = state.x, state.y, state.theta
        successors = []

        for action in self.lattice_resolution_dict['high']:
            transitions = action.get_transitions(state)

            dx = 0
            dy = 0
            dtheta = 0

            for transition in transitions:
                dx += transition[0]
                dy += transition[1]
                dtheta += transition[2]

            successor_x = sx + dx
            successor_y = sy + dy
            successor_theta = stheta + dtheta

            if (successor_theta < 0):
                successor_theta = 2*math.pi + successor_theta
            elif (successor_theta > 2*math.pi):
                successor_theta = successor_theta % (2*math.pi)

            cost = state.g + action.cost
            successor = State(successor_x, successor_y, successor_theta, parent=state, parent_action=action, g=cost)
            successor.setH(self.heuristicFunc(successor))

            if (self.checkValidState(successor)):
                successors.append(successor)
        return successors

    def backtrack(self, state):
        """returns a sequence of steps to get to given state from the start state"""
        steps = []
        while (state.parent is not None):
            action = state.parent_action
            transitions = action.get_transitions(state.parent)
            steps = transitions + steps
            state = state.parent
        return steps

    def checkValidState(self, state):
        return True # TODO: change this to actually reference the map and also check bounds

    def heuristicFunc(self, state):
        goal_x = self.goal[0]
        goal_y = self.goal[1]
        return math.sqrt((state.x - goal_x)**2 + (state.y - goal_y)**2)

    def fvalue(self, state):
        return state.g + self.epsilon * state.h

    def decreaseEpsilon(self):
        self.epsilon -= self.epsilon_decrement
        if (self.epsilon < 1): self.epsilon = 1
