from primitives import Trajectory, Commands, State, Conf, PoseStamped as OurPoseStamped
from geometry_msgs.msg import PoseStamped
from tf import transformations as T
import numpy as np
from utils import compute_grasp_poses, odom_to_map
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import moveit_commander

def get_grasp_gen(scene):
    def gen(obj):
        bbox = Marker()
        bbox.header.frame_id = "map"
        # bbox.pose = odom_to_map(scene.get_object_poses([obj])[obj]).pose
        bbox.pose.position.x = -0.431 - 0.2
        bbox.pose.position.y = -1.46
        bbox.pose.position.z = 0.45 + 0.1
        if obj == "ycb_003_cracker_box":
            bbox.scale.x = 0.06  # cracker box dimensions
            bbox.scale.y = 0.16
            bbox.scale.z = 0.21
        else:
            bbox.scale.x = 0.1
            bbox.scale.y = 0.1
            bbox.scale.z = 0.1
        grasps = compute_grasp_poses(
            bbox,
            top=False, 
            side=True,
            relative=False,
            side_up=True,
            rigid=True,
            off_center=False,
            pre_grasp=0.2
        )
        return [(g,) for g in grasps]
    return gen

def pick_motion(whole_body):
    def gen_pick(a ,o ,p ,g ,q1):
        q1_config = q1.robot_state
        robot_state = moveit_commander.RobotState()
        robot_state.joint_state = q1_config
        whole_body.set_start_state(robot_state)
        whole_body.set_pose_target(g)
        plan = whole_body.plan()
        if not plan[0]:
            yield None
        else:
            yield (plan[1],) 
    return gen_pick


# def get_pose_gen(whole_body, table_pose):
#     def gen(obj):

#                 yield (place_pose,)
#     return gen
