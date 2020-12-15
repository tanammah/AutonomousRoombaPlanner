import math
from state import State

class Action:
    def __init__(self):
        self.min_displacement = 1.45

    def get_transitions(self, init_state):
        """
        This method takes in an initial state (x, y, theta) and outputs a list of transitions
        of the form (dx, dy, dtheta) indicating the transitions involved in taking the action
        """
        raise NotImplementedError("Transition function not implemented")

    def get_cost(self):
        return self.cost

class ForwardAction(Action):
    def __init__(self):
        self.name = "Forward"
        self.cost = 2
        self.dtheta = 0 # net change in theta after taking action
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]

class ForwardShortAction(Action):
    def __init__(self):
        self.name = "Forward_Short"
        self.cost = 1
        self.dtheta = 0
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = end_theta

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta

        return [(dx_1, -dy_1, dtheta_1)]

class ForwardLeftSlightAction(Action):
    def __init__(self):
        self.name = "Forward_Left_Slight"
        self.cost = 2.4
        self.dtheta = math.pi/4
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]

class ForwardLeftSharpAction(Action):
    def __init__(self):
        self.name = "Forward_Left_Sharp"
        self.cost = 1.4
        self.dtheta = math.pi/4
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(end_theta))
        y_1 = round(y + self.min_displacement * math.sin(end_theta))
        theta_1 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta

        return [(dx_1, -dy_1, dtheta_1)]

class ForwardRightSlightAction(Action):
    def __init__(self):
        self.name = "Forward_Right_Slight"
        self.cost = 2.4
        self.dtheta = -math.pi/4
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]

class ForwardRightSharpAction(Action):
    def __init__(self):
        self.name = "Forward_Right_Sharp"
        self.cost = 1.4
        self.dtheta = -math.pi/4
        super().__init__()

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(end_theta))
        y_1 = round(y + self.min_displacement * math.sin(end_theta))
        theta_1 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta

        return [(dx_1, -dy_1, dtheta_1)]

class BackwardAction(Action):
    def __init__(self):
        self.name = "Backward"
        self.cost = 3
        self.dtheta = 0 # net change in theta after taking action
        super().__init__()
        self.min_displacement = -1.45

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]

class BackwardLeftSlightAction(Action):
    def __init__(self):
        self.name = "Backward_Left_Slight"
        self.cost = 3.8
        self.dtheta = -math.pi/4
        super().__init__()
        self.min_displacement = -1.45

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]

class BackwardRightSlightAction(Action):
    def __init__(self):
        self.name = "Backward_Right_Slight"
        self.cost = 3.8
        self.dtheta = math.pi/4
        super().__init__()
        self.min_displacement = -1.45

    def get_transitions(self, init_state):
        x, y, theta = init_state.x, init_state.y, init_state.theta

        end_theta = theta + self.dtheta # final theta at end state (after having taken action)

        x_1 = round(x + self.min_displacement * math.cos(theta))
        y_1 = round(y + self.min_displacement * math.sin(theta))
        theta_1 = theta + self.dtheta/2

        x_2 = round(x_1 + self.min_displacement * math.cos(end_theta))
        y_2 = round(y_1 + self.min_displacement * math.sin(end_theta))
        theta_2 = end_theta 

        dx_1 = x_1 - x
        dy_1 = y_1 - y
        dtheta_1 = self.dtheta/2

        dx_2 = x_2 - x_1
        dy_2 = y_2 - y_1
        dtheta_2 = self.dtheta/2

        return [(dx_1, -dy_1, dtheta_1), (dx_2, -dy_2, dtheta_2)]



