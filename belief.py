import pprint

class BeliefState():
    def __init__(self, arm: str, hand_empty:bool,
                 robotAt: str, objects:list, 
                 surfaces:list, configs:dict, 
                 belief: dict, visibility: dict,
                 visited: set, known: set):
        self.robotAt = robotAt
        self.arm = arm
        self.hand_empty = hand_empty                 
        self.objects = objects
        self.surfaces = surfaces
        self.configs = configs
        self.belief = belief
        self.visibility = visibility
        self.visited = visited
        self.known = known

    def print(self):
        pprint.pprint(self.belief)