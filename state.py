
class State:
    def __init__(self, x, y, theta, parent, parent_action):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        self.parent_action = parent_action

    def set_parent(self, parent):
        self.parent = parent

    def set_parent_action(self, parent_action):
        self.parent_action = parent_action
