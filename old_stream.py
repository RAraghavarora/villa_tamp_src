import math
import random
import moveit_commander
import copy
from visualization_msgs.msg import Marker
from primitives import Trajectory, Commands, Conf, State
from primitives import PoseStamped as OurPoseStamped
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from moveit_msgs.msg import Constraints, PositionConstraint, Grasp
from shape_msgs.msg import SolidPrimitive
import rospy
from tf import transformations as T
import numpy as np
from moveit_commander.exception import MoveItCommanderException 
from utils import (
    make_gripper_posture,
    make_gripper_translation,
    pose_to_matrix,
    matrix_to_pose,
    make_pose,
    odom_to_map,
    compute_grasp_poses
)
from sensor_msgs.msg import JointState


def get_base_motion_gen(move_group, robot_state):
    def fn(bq1, bq2):
        move_group.set_start_state(robot_state)
        # Set joint target for base motion
        joint_values = [
            bq2.values[0],  # x
            bq2.values[1],  # y
            bq2.values[2],  # theta
        ]
        move_group.set_joint_value_target("world_joint", joint_values)

        # Plan the motion
        plan = move_group.plan()
        if not plan[0]:
            rospy.logwarn("Failed to find a plan")
            return None

        # Create trajectory path for PDDLStream
        path = [bq1, bq2]  # Simplified path

        bt = Trajectory(path, moveit_plan=plan[1])
        cmd = Commands(State(), savers=[], commands=[bt])
        return (cmd,)

    return fn


def get_arm_motion_gen(move_group, robot_state):
    """Generate arm motions for HSR using MoveIt"""
    # HSR arm joints: 'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 
    #                 'wrist_flex_joint', 'wrist_roll_joint'
    arm_joints = [
        'arm_lift_joint',
        'arm_flex_joint',
        'arm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
        'wrist_ft_sensor_frame_joint'
    ]

    def fn(arm, aq1, aq2):
        move_group.set_start_state(robot_state)
        # Set joint targets for arm motion
        joint_values = dict(zip(arm_joints, aq2.values))

        for arm in arm_joints:
            try:
                move_group.set_joint_value_target(arm, joint_values[arm])
            except MoveItCommanderException:
                rospy.logwarn(f"Failed to set joint value target for {arm}")
                continue

        # Plan the motion
        plan = move_group.plan()
        if not plan[0]:
            rospy.logwarn("Failed to find an arm motion plan")
            return None

        # Create trajectory path for PDDLStream
        path = [aq1, aq2]  # Simplified path

        at = Trajectory(path, moveit_plan=plan[1])
        cmd = Commands(State(), savers=[], commands=[at])
        return (cmd,)

    return fn

def get_grasp_gen(whole_body, obj_pose, hsrb_gripper):
    def gen(obj):
        map_frame_pose = obj_pose.pose
        bbox = Marker()
        bbox.header.frame_id = "map"
        bbox.pose.position = map_frame_pose.position
        bbox.pose.orientation = map_frame_pose.orientation
        bbox.scale.x = 0.06
        bbox.scale.y = 0.16
        bbox.scale.z = 0.21
        grasps = compute_grasp_poses(
            bbox,
            top=False,
            side=True,
            relative=False,
            side_up=True,
            rigid=True,
            off_center=False,
            pre_grasp=0.2,
        )
        random.shuffle(grasps)
        # TODO: We can move this part to post_process once the streams are changed to remove move_base and move_arm actions
        working_grasp = None
        hsrb_gripper.command(1.0)

        for grasp in grasps:
            whole_body.set_pose_target(grasp)
            plan = whole_body.plan()
            if plan[0]:
                rospy.loginfo("Executing a pre-grasp")
                working_grasp = grasp
                whole_body.execute(plan[1])
                close_grasps = compute_grasp_poses(
                    bbox,
                    top=False,
                    side=True,
                    relative=False,
                    side_up=True,
                    rigid=True,
                    off_center=False,
                    pre_grasp=0.07,
                )
                for grsp in close_grasps:
                    whole_body.set_pose_target(grsp)
                    plan = whole_body.plan()
                    if plan[0]:
                        rospy.loginfo("Executing final grasp")
                        whole_body.execute(plan[1])
                        hsrb_gripper.apply_force(0.2)
                        break
                break

        if working_grasp is None:
            rospy.logwarn("Failed to find a working grasp")
            raise NotImplementedError("No plan for any grasp found")
        else:
            rospy.loginfo("Found and executed a working grasp, shutting down now")
            raise NotImplementedError("Grasp executed")

    return gen


def get_ik_fn(whole_body, arm_group, robot_state):
    # Doesn't do anything yet

    def fn(arm, obj, obj_pose, grasp):
        whole_body.set_pose_target(grasp)
        plan = whole_body.plan()
        if plan[0]:

            #TODO: Change the stream.pddl to remove arm and base actions
            # We technically have to yeild base conf and arm conf, but those are redundant at this point. 

            yield (None, None)
        else:
            rospy.logwarn("Failed to find an IK solution for the received grasp")


    return fn
