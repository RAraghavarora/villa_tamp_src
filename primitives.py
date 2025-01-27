import geometry_msgs.msg
from itertools import count
import copy


class Command(object):
    def __init__(self):
        self.commands = []


# class Conf(object):
#     def __init__(self, robot, joints, values, moveit_plan=None):
#         self.robot = robot
#         self.joints = joints
#         self.values = values
#         self.moveit_plan = moveit_plan

#     def assign(self):
#         robot_state = RobotState()
#         robot_state.joint_state.name = self.joints
#         robot_state.joint_state.position = self.values
#         return robot_state

class Conf(object):
    num = count()
    def __init__(self, robot_state):
        self.robot_state = robot_state
        self.index = next(self.num)
    def iterate(self):
        yield self
    def __repr__(self):
        index = self.index
        return 'q{}'.format(index)


class Trajectory(Command):
    def __init__(self, path, moveit_plan=None):
        super(Trajectory, self).__init__()
        self.path = path
        self.moveit_plan = moveit_plan

class Commands(object):
    def __init__(self, state, savers=None, commands=None):
        # super(Commands, self).__init__()
        self.state = state
        self.savers = tuple(savers)
        self.commands = tuple(commands)

    def assign(self):
        for saver in self.savers:
            saver.restore()
        return copy.copy(self.state)

    def apply(self, state, **kwargs):
        for command in self.commands:
            for result in command.apply(state, **kwargs):
                yield result

    def __repr__(self):
        return "c{}".format(id(self) % 1000)

class State(object):
    def __init__(self, attachments={}, cleaned=set(), cooked=set()):
        pass

class PoseStamped(geometry_msgs.msg.PoseStamped):
    def __init__(self):
        super(PoseStamped, self).__init__()
        self.moveit_grasp = None
        self.header.frame_id = "map"