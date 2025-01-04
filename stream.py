from primitives import Trajectory, Commands, State, Pose
from geometry_msgs.msg import PoseStamped
import rospy
from tf import transformations as T
import numpy as np
from moveit_commander.exception import MoveItCommanderException

def get_base_motion_gen(move_group, custom_limits={}, collisions=True):
    def fn(bq1, bq2):
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


def get_arm_motion_gen(move_group, custom_limits={}, collisions=True):
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

# def get_arm_motion_gen(move_group):
    def fn(aq1, aq2):
        target_pose = PoseStamped()
        target_pose.header.frame_id = move_group.get_planning_frame()
        target_pose.pose.position.x = aq2.position.x
        target_pose.pose.position.y = aq2.position.y
        target_pose.pose.position.z = aq2.position.z
        target_pose.pose.orientation = aq2.orientation

        move_group.set_pose_target(target_pose)
        plan = move_group.plan()

        if not plan[0]:
            rospy.logwarn("Failed to find a joint motion plan")
            return None

        path = [aq1, aq2]  # Simplified path
        at = Trajectory(path, moveit_plan=plan[1])
        cmd = Commands(State(), savers=[], commands=[at])
        return (cmd,)

    return fn


def get_grasp_gen():
    def gen(obj):
        grasps = []
        
        # Top-down grasp
        grasp = Pose()
        grasp.position.z = 0.1  # Grasp from 10cm above
        quat = T.quaternion_from_euler(0, np.pi/2, 0)  # Vertical approach
        grasp.orientation.x = quat[0]
        grasp.orientation.y = quat[1]
        grasp.orientation.z = quat[2]
        grasp.orientation.w = quat[3]
        grasps.append(grasp)
        
        # Side grasp
        grasp = Pose()
        grasp.position.x = 0.1  # Grasp from 10cm to the side
        quat = T.quaternion_from_euler(0, 0, np.pi/2)  # Horizontal approach
        grasp.orientation.x = quat[0]
        grasp.orientation.y = quat[1]
        grasp.orientation.z = quat[2]
        grasp.orientation.w = quat[3]
        grasps.append(grasp)
        
        for grasp in grasps:
            yield (grasp,)
            
    return gen