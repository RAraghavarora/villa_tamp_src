import geometry_msgs.msg
from moveit_commander import RobotState

class Command(object):
    def __init__(self):
        self.commands = []

class State(object):
    def __init__(self):
        pass

class Pose(object):
    def __init__(self):
        self.position = geometry_msgs.msg.Point()
        self.orientation = geometry_msgs.msg.Quaternion()

class Conf(object):
    def __init__(self, robot, joints, values, moveit_plan=None):
        self.robot = robot
        self.joints = joints
        self.values = values
        self.moveit_plan = moveit_plan

    def assign(self):
        robot_state = RobotState()
        robot_state.joint_state.name = self.joints
        robot_state.joint_state.position = self.values
        return robot_state

class Trajectory(Command):
    def __init__(self, path, moveit_plan=None):
        super(Trajectory, self).__init__()
        self.path = path
        self.moveit_plan = moveit_plan

class Commands(Command):
    def __init__(self, state, savers=None, commands=None):
        super(Commands, self).__init__()
        self.state = state
        self.savers = savers or []
        self.commands = commands or []

class Problem(object):
    def __init__(self, robot):
        self.robot = robot