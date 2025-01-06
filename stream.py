import random
from primitives import Trajectory, Commands, State, Pose, Conf
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


def get_grasp_gen(move_group):
    GRASP_HEIGHT = 0.1
    SIDE_GRASP_OFFSET = 0.1

    def gen(obj):
        try:
            obj_pose_stamped = move_group.get_current_pose(obj)
        except:
            rospy.logwarn("Failed to get object pose")
            return None
        
        grasps = []
        
        # Top-down grasp
        top_grasp = Pose()
        top_grasp.position.x = obj_pose_stamped.pose.position.x
        top_grasp.position.y = obj_pose_stamped.pose.position.y
        top_grasp.position.z = obj_pose_stamped.pose.position.z + GRASP_HEIGHT
        quat = T.quaternion_from_euler(0, np.pi/2, 0)
        top_grasp.orientation.x = quat[0]
        top_grasp.orientation.y = quat[1]
        top_grasp.orientation.z = quat[2]
        top_grasp.orientation.w = quat[3] + GRASP_HEIGHT
        grasps.append(top_grasp)
        
        # Side grasp
        for angle in [0, np.pi/2, -np.pi/2]:
            side_grasp = Pose()
            side_grasp.position.x = obj_pose_stamped.pose.position.x + SIDE_GRASP_OFFSET*np.cos(angle)
            side_grasp.position.y = obj_pose_stamped.pose.position.y + SIDE_GRASP_OFFSET*np.sin(angle)
            side_grasp.position.z = obj_pose_stamped.pose.position.z
            quat = T.quaternion_from_euler(0, 0, angle + np.pi)
            side_grasp.orientation.x = quat[0]
            side_grasp.orientation.y = quat[1]
            side_grasp.orientation.z = quat[2]
            side_grasp.orientation.w = quat[3]
            grasps.append(side_grasp)

        random.shuffle(grasps)
        return (grasps[0],)

        return [(g,) for g in grasps]
            
    return gen

def get_ik_fn(move_group):
    def fn(arm, obj, obj_pose, grasp):
        try:
            # target_pose -> End-effector pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = move_group.get_planning_frame()

            target_pose.pose.position.x = obj_pose.position.x + grasp.position.x
            target_pose.pose.position.y = obj_pose.position.y + grasp.position.y
            target_pose.pose.position.z = obj_pose.position.z + grasp.position.z
            target_pose.pose.orientation = grasp.orientation

            # base_conf -> base pose
            angle_to_obj = np.arctan2(obj_pose.position.y, obj_pose.position.x)
            DESIRED_DISTANCE = 0.5
            base_x = obj_pose.position.x - DESIRED_DISTANCE*np.cos(angle_to_obj)
            base_y = obj_pose.position.y - DESIRED_DISTANCE*np.sin(angle_to_obj)
            base_theta = angle_to_obj
            base_conf = Conf(move_group, ['world_joint'], [base_x, base_y, base_theta])

            # IK for arm?
            move_group.clear_pose_targets()
            move_group.set_pose_target(target_pose)
            move_group.set_joint_value_target("world_joint", base_conf.values)
            plan = move_group.plan()
            if not plan[0]:
                rospy.logwarn("Failed to find an IK solution")
                return None
            
            joint_goal = move_group.get_joint_value_target()

            arm_joints = [
                'arm_lift_joint',
                'arm_flex_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint',
                'wrist_ft_sensor_frame_joint'
            ]
            arm_conf = Conf(move_group, arm_joints, joint_goal)
            return (arm_conf, base_conf)
            
            


            breakpoint()

            plan = move_group.plan()
            if not plan[0]:
                rospy.logwarn("Failed to find an IK solution")
                return None

            joint_goal = move_group.get_joint_value_target()
            move_group.clear_pose_targets()
            arm_joints = [
                'arm_lift_joint',
                'arm_flex_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint',
                'wrist_ft_sensor_frame_joint'
            ]
            arm_conf = Conf(move_group, arm_joints, joint_goal)
            return (arm_conf, base_conf)
        # try:
        #     joint_goal = move_group.get_joint_value_target()
        #     move_group.clear_pose_targets()
        #     q = Conf(move_group, move_group.get_active_joints(), joint_goal)
        #     return (q,)
        except MoveItCommanderException as e:
            rospy.logwarn(f"Failed to find an IK solution: {e}")
            return None
        finally:
            move_group.clear_pose_targets()

    return fn