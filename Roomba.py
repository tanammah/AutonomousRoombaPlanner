import math
import time
import numpy as np
from utils import *
from motion_primitives import *
from queue import PriorityQueue

class Roomba:
    def __init__(self):
        self.map = None # numpy array
        self.start = None # tuple: (x, y, theta)
        self.goal = None  # tuple: (x, y, theta)
        self.lattice_resolution_dict = self.generateResolutionDict()
        self.max_search_time = 1 # seconds
        self.resolution = "high"
        self.heuristic_map = None # numpy array 
        self.heuristic_maps = dict() # dict of numpy arrays 
        self.heat_zone_threshold = 4 # radius about start and goal endpoints within which all primitives are used even in low resolution

        # ARA* related member variables
        self.incons = set()
        self.closed = dict()
        self.open = PriorityQueue()
        self.open_states = dict()
        self.epsilon_init_val = 5
        self.epsilon_decrement = 2
        self.epsilon = self.epsilon_init_val
        self.goal_state = None

    def setMap(self, map_data):
        self.map = map_data
        self.heuristic_map = np.zeros(map_data.shape)
        self.heuristic_maps['low'] = np.array([[100000000]*len(map_data[0]) for i in range(len(map_data))], dtype=float)
        self.heuristic_maps['high'] = np.array([[100000000]*len(map_data[0]) for i in range(len(map_data))], dtype=float)
        self.updateHeuristicMapLatticeGraph()

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
        res_dict['high'] = [ForwardAction(), ForwardShortAction(), ForwardLeftSlightAction(), ForwardLeftSharpAction(), 
                                ForwardRightSlightAction(), ForwardRightSharpAction(), BackwardAction(), 
                                BackwardLeftSlightAction(), BackwardRightSlightAction()]
        return res_dict

    def findPath(self):
        start_time = time.time()

        for resolution in ['low', 'high']:
            self.resolution = resolution
            start_state = State(self.start[0], self.start[1], self.start[2], g=0)
            start_state.h = self.heuristicFunc(start_state)

            self.goal_state = State(self.goal[0], self.goal[1], self.goal[2], g=float('inf'), h=0)

            # initialize data structures
            self.incons = set()
            self.closed = dict()
            self.open = PriorityQueue()
            self.open_states = dict()

            self.epsilon = self.epsilon_init_val

            self.open.put((self.fvalue(start_state), start_state))
            self.open_states[start_state] = start_state

            self.improvePath()

            if (self.goal_state.g == float('inf')): # if no solution is found in lower res, skip to higher res
                continue 

            if (self.goal_state.g < float('inf')):
                    cost = self.getPathCost(self.goal_state)
                    print("solution cost for {} resolution (epsilon={}): {}".format(self.resolution, self.epsilon, cost))

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

                if (self.goal_state.g < float('inf')):
                    cost = self.getPathCost(self.goal_state)
                    print("solution cost for {} resolution (epsilon={}): {}".format(self.resolution, self.epsilon, cost))

                curr_time = time.time()

                if ((curr_time - start_time >= self.max_search_time)):
                    return self.getSolution()
            print("\n")

        return self.getSolution()

    def findPathNoReset(self, resolution='low'):
        start_time = time.time()

        self.resolution = resolution
        start_state = State(self.start[0], self.start[1], self.start[2], g=0)
        start_state.h = self.heuristicFunc(start_state)

        self.goal_state = State(self.goal[0], self.goal[1], self.goal[2], g=float('inf'), h=0)

        # initialize data structures
        self.incons = set()
        self.closed = dict()
        self.open = PriorityQueue()
        self.open_states = dict()

        self.epsilon = self.epsilon_init_val

        self.open.put((self.fvalue(start_state), start_state))
        self.open_states[start_state] = start_state

        self.improvePath()

        if (self.goal_state.g == float('inf')): # if no solution is found in lower res, skip to higher res
            if resolution == 'high':
                return None
            return findPathNoReset('high') 

        if (self.goal_state.g < float('inf')):
                cost = self.getPathCost(self.goal_state)
                print("solution cost for {} resolution (epsilon={}): {}".format(self.resolution, self.epsilon, cost))

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

            if (self.goal_state.g < float('inf')):
                cost = self.getPathCost(self.goal_state)
                print("solution cost for {} resolution (epsilon={}): {}".format(self.resolution, self.epsilon, cost))

            curr_time = time.time()

            if ((curr_time - start_time >= self.max_search_time)):
                return self.getSolution()

            if (self.epsilon <= 1 and self.resolution == 'low'):
                self.epsilon = self.epsilon_init_val
                self.resolution = 'high'
                print("doing no reset")
            print("\n")

        return self.getSolution()

    def getSolution(self):
        """ returns path if solution is found else None"""
        if self.goal_state and (self.goal_state.g < float('inf')):
            return self.backtrack(self.goal_state)
        return None

    def improvePath(self):
        self.expansions_count = 0
        while (not self.open.empty()):
            min_state = self.open.get()[1]

            if (min_state in self.closed): continue # state could be a dummy node

            if (self.fvalue(self.goal_state) < self.fvalue(min_state)):
                break

            self.expansions_count += 1
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
            if ((state.g + state.h) >= self.goal_state.g): continue # prune states that can never produce a more optimal solution

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
        motion_primitives = None
        successors = []

        if (self.resolution == "high"):
            motion_primitives = self.lattice_resolution_dict['high']
        else: # low resolution
            state_loc = state.x, state.y
            start_loc = self.start[:2]
            goal_loc = self.goal[:2]

            if ((euclid(state_loc, start_loc) < self.heat_zone_threshold) or (euclid(state_loc, goal_loc) < self.heat_zone_threshold)):
                motion_primitives = self.lattice_resolution_dict['high']
            else:
                motion_primitives = self.lattice_resolution_dict['low']


        for action in motion_primitives:
            valid_action = True
            transitions = action.get_transitions(state)

            dx = 0
            dy = 0
            dtheta = 0

            curr_x = sx
            curr_y = sy
            curr_theta = stheta

            for transition in transitions:
                dx += transition[0]
                dy += transition[1]
                dtheta += transition[2]

                temp_x = sx + dx
                temp_y = sy + dy
                temp_theta = (stheta + dtheta) % (2*math.pi)

                if (not self.checkValidLocation((temp_x, temp_y))):
                    valid_action = False
                    break

                # diagonal transition checking
                if not self.checkValidTransition(curr_x, curr_y, transition[0], transition[1]):
                    valid_action = False
                    break

                curr_x += dx
                curr_y += dy
                curr_theta += dtheta

            if not valid_action:
                continue

            successor_x = sx + dx
            successor_y = sy + dy
            successor_theta = stheta + dtheta

            successor_theta = successor_theta % (2*math.pi) # normalize so thetas stay within {0 -> 2pi}

            cost = state.g + action.get_cost(state)
            successor = State(successor_x, successor_y, successor_theta, parent=state, parent_action=action, g=cost)
            successor.setH(self.heuristicFunc(successor))
            successors.append(successor)
        return successors

    def backtrack(self, state):
        """returns a sequence of steps to get to given state from the start state"""
        tot_cost = 0
        steps = []
        while (state.parent is not None):
            action = state.parent_action
            transitions = action.get_transitions(state.parent)
            tot_cost += action.get_cost(state.parent)
            steps = transitions + steps
            state = state.parent
        return self.convertStepsToPositions(steps)

    def checkValidTransition(self, curr_x, curr_y, dx, dy):
        """ Checks that a diagonal transition is valid i.e no obstacles obstructing transition"""
        if (dx == 0) or (dy == 0):
            return True

        curr_col, curr_row = curr_x, curr_y

        col_1, row_1 = curr_col, curr_row + dy
        col_2, row_2 = curr_col + dx, curr_row

        if (self.checkWithinBounds((col_1, row_1)) and self.checkWithinBounds((col_2, row_2))):
            if self.map[row_1][col_1] or self.map[row_2][col_2]:
                return False
        return True

    def getPathCost(self, state):
        tot_cost = 0
        while (state.parent is not None):
            action = state.parent_action
            transitions = action.get_transitions(state.parent)
            tot_cost += action.get_cost(state.parent)
            state = state.parent
        return tot_cost

    def convertStepsToPositions(self, steps):
        positions = []
        curr_col = self.start[0]
        curr_row = self.start[1]
        curr_theta = self.start[2]
        for step in steps:
            curr_col += step[0]
            curr_row += step[1]
            curr_theta += step[2]
            positions.append((curr_col, curr_row, curr_theta))
        return positions

    def checkValidState(self, state):
        return self.checkValidLocation((state.x, state.y))

    def checkValidLocation(self, loc):
        if (not self.checkWithinBounds(loc)):
            return False
            
        col, row = loc
        return not self.map[row][col]

    def checkWithinBounds(self, loc):
        col, row = loc
        map_height, map_width = self.map.shape

        if (row < 0) or (row >= map_height) or (col < 0) or (col >= map_width):
            return False

        return True

    def heuristicFunc(self, state):
        goal_x = self.goal[0]
        goal_y = self.goal[1]
        #return self.heuristic_maps[self.resolution][state.y][state.x]
        return math.sqrt((state.x - goal_x)**2 + (state.y - goal_y)**2)

    def fvalue(self, state):
        return state.g + self.epsilon * state.h

    def decreaseEpsilon(self):
        self.epsilon -= self.epsilon_decrement
        if (self.epsilon < 1): self.epsilon = 1

    def updateHeuristicMapFullyConnected(self):
        """Updates the heuristic map for all obstacle-free map coordinates"""
        t1 = time.time()
        start = (self.goal[0], self.goal[1])

        dx = [-1, -1, 0, 1, 1, 1, 0, -1]
        dy = [0, -1, -1, -1, 0, 1, 1, 1]

        heap = PriorityQueue()
        heap_states = dict()
        closed = set()

        heap.put((0, start))
        heap_states[start] = 0

        while (not heap.empty()):
            min_state_cost , min_state = heap.get()

            if min_state in closed: continue # might be a dummy node!

            self.heuristic_map[min_state[1]][min_state[0]] = min_state_cost
            closed.add(min_state)
            del heap_states[min_state]

            x, y = min_state
            for i in range(len(dx)):
                x_prime = x + dx[i]
                y_prime = y + dy[i]
                neighbour = (x_prime, y_prime)
                neighbour_cost = min_state_cost + euclid(min_state, neighbour)
                if not (self.checkValidLocation(neighbour) and self.checkValidTransition(x, y, dx[i], dy[i])): continue
                if neighbour in closed: continue
                if ((neighbour in heap_states) and (heap_states[neighbour] < neighbour_cost)): continue
                heap.put((neighbour_cost, neighbour))
                heap_states[neighbour] = neighbour_cost
        t2 = time.time()
        print("it took {} seconds to update learn heuristic map".format(t2-t1))

    def updateHeuristicMapLatticeGraph(self):
        """Updates the heuristic map for all obstacle-free map coordinates"""
        t1 = time.time()

        movements = dict()
        for res in ['low', 'high']:
            motion_primitives = self.lattice_resolution_dict[res]
            d = dict()
            for action in motion_primitives:
                for i in range(8):
                    theta = i*math.pi/4
                    state = State(0, 0, theta)
                    transitions = action.get_transitions(state)
                    cost = action.get_cost(state)
                    # Multiply by -1 to invert actions
                    if len(transitions) > 1:
                        dx = -1*(transitions[0][0] + transitions[1][0])
                        dy = -1*(transitions[0][1] + transitions[1][1])  
                        if (dx,dy) not in d or d[(dx,dy)] > cost:
                            d[(dx,dy)] = cost
                    else:
                        dx = -1*transitions[0][0]
                        dy = -1*transitions[0][1]
                        if (dx,dy) not in d or d[(dx,dy)] > cost:
                            d[(dx, dy)] = cost
        
            movements[res] = set()
            for (dx,dy) in d:
                movements[res].add((dx,dy,d[(dx,dy)]))

        for res in ['low', 'high']:
            start = (self.goal[0], self.goal[1])
            print(start)

            heap = PriorityQueue()
            heap_states = dict()
            closed = set()

            heap.put((0, start))
            heap_states[start] = 0

            while (not heap.empty()):
                min_state_cost , min_state = heap.get()

                if min_state in closed: continue # might be a dummy node!

                self.heuristic_maps[res][min_state[1]][min_state[0]] = min_state_cost
                closed.add(min_state)
                del heap_states[min_state]

                x, y = min_state

                if euclid(min_state, start) < self.heat_zone_threshold:
                    moves = movements["high"]
                else:
                    moves = movements[res]

                for move in moves:
                    (dx, dy, move_cost) = move
                    x_prime = x + dx
                    y_prime = y + dy
                    neighbour = (x_prime, y_prime)
                    neighbour_cost = min_state_cost + move_cost
                    if not (self.checkValidLocation(neighbour)): continue
                    if neighbour in closed: continue
                    if ((neighbour in heap_states) and (heap_states[neighbour] < neighbour_cost)): continue
                    heap.put((neighbour_cost, neighbour))
                    heap_states[neighbour] = neighbour_cost
        
        t2 = time.time()
        print("it took {} seconds to update learn heuristic map".format(t2-t1))

