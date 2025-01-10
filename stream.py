import random
from primitives import Trajectory, Commands, State, Pose, Conf
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
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


def get_grasp_gen(move_group, scene):
    GRASP_STANDOFF = 0.1

    def gen(obj):
        try:
            object_pose_dict = scene.get_object_poses([obj])
        except:
            rospy.logwarn("Failed to get object pose")
            return None
        
        obj_pose = object_pose_dict[obj] # Object pose not needed because we calculate related to the object
        if obj == 'ycb_003_cracker_box':
            width, length, height = 0.06, 0.16, 0.21
        else:
            width = length = height = 0.1
        rospy.loginfo(f"Generating grasp poses for {obj}")
        grasp_dimensions = (width, length, height)
        grasps = []
        # Top-down grasp
        # from primitives import Trajectory, Commands, State, Pose, Conf
        from geometry_msgs.msg import Pose
        # top_grasp = Pose()
        # top_grasp.position.x = obj_pose.position.x
        # top_grasp.position.y = obj_pose.position.y
        # top_grasp.position.z = obj_pose.position.z + GRASP_HEIGHT
        # quat = T.quaternion_from_euler(0, np.pi/2, 0)
        # top_grasp.orientation.x = quat[0]
        # top_grasp.orientation.y = quat[1]
        # top_grasp.orientation.z = quat[2]
        # top_grasp.orientation.w = quat[3] + GRASP_HEIGHT
        # grasps.append(top_grasp)
        
        # height_ratios = [0.5, 0.6, 1]
        # Side grasps relative to the object
        rospy.loginfo(f"Object pose: {obj_pose}")
        for angle in [0, np.pi/2, np.pi, -np.pi/2]:
            # height_ratio = random.choice(height_ratios)
            height_ratio = 1
            side_grasp = Pose()
            x_offset = np.cos(angle) * (width / 2 + GRASP_STANDOFF)
            y_offset = np.sin(angle) * (width / 2 + GRASP_STANDOFF)
            z_offset = height_ratio * height - height / 2

            side_grasp.position = Point(x_offset, y_offset, z_offset)

            roll = 0
            pitch = 0
            yaw = angle

            quat = T.quaternion_from_euler(roll, pitch, yaw)
            side_grasp.orientation = Quaternion(*quat)
            # yield side_grasp
            rospy.loginfo(f"Generated Side grasp: {side_grasp}")
            grasps.append(side_grasp)

        #     side_grasp.position.x = obj_pose.position.x + SIDE_GRASP_OFFSET*np.cos(angle)
        #     side_grasp.position.y = obj_pose.position.y + SIDE_GRASP_OFFSET*np.sin(angle)
        #     side_grasp.position.z = obj_pose.position.z + GRIPPER_LENGTH/2
        #     quat = T.quaternion_from_euler(0, 0, angle + np.pi)
        #     side_grasp.orientation.x = quat[0]
        #     side_grasp.orientation.y = quat[1]
        #     side_grasp.orientation.z = quat[2]
        #     side_grasp.orientation.w = quat[3]
        #     grasps.append(side_grasp)

        # random.shuffle(grasps)

        # for grasp in grasps:
        #     target_pose = PoseStamped()
        #     target_pose.header.frame_id = move_group.get_planning_frame()
        #     target_pose.pose.position = grasp.position
        #     target_pose.pose.orientation = grasp.orientation
        #     move_group.set_end_effector_link("hand_palm_link")
        #     move_group.set_pose_target(target_pose, "hand_palm_link") #TODO: works with whole_body, not with move_group?
            
        #     plan = move_group.plan()
        #     if plan[0]:
        #         return (grasp,)
        #     else:
        #         rospy.logwarn(plan[-1])

        random.shuffle(grasps)
        return [(grasp,) for grasp in grasps]

        # for grasp in grasps:
        #     yield (grasp,)
        # return [(g,) for g in grasps]

        # rospy.logwarn("Failed to find a grasp")
        # return None
                   
    return gen


def get_ik_fn(whole_body, arm_group):
    DESIRED_DISTANCE = 0.5
    def fn(arm, obj, obj_pose, grasp):
        try:
            obj_matrix = pose_to_matrix(obj_pose)
            grasp_matrix = pose_to_matrix(grasp)
            world_grasp_matrix = np.dot(obj_matrix, grasp_matrix) # grasp_pose -> world
            world_grasp_pose = matrix_to_pose(world_grasp_matrix)

            angle_to_obj = np.arctan2(world_grasp_pose.position.y, world_grasp_pose.position.x)
            base_x = world_grasp_pose.position.x - DESIRED_DISTANCE*np.cos(angle_to_obj)
            base_y = world_grasp_pose.position.y - DESIRED_DISTANCE*np.sin(angle_to_obj)
            base_theta = angle_to_obj
            whole_body.clear_pose_targets()
            whole_body.set_pose_reference_frame("map")
            whole_body.set_joint_value_target("world_joint", [base_x, base_y, base_theta])

            plan = whole_body.plan()
            if plan[0]:
                base_conf = Conf(whole_body, ["world_joint"], [base_x, base_y, base_theta], moveit_plan=plan[1])
                
                target_pose = PoseStamped()
                target_pose.header.frame_id = 'map'
                target_pose.pose = world_grasp_pose.pose
                whole_body.set_pose_target(target_pose, "hand_palm_link")
                plan = whole_body.plan()
                if plan[0]:
                    arm_conf = Conf(whole_body, arm_group.get_active_joints(), [plan[1].joint_trajectory.points[-1].positions[i] for i in range(6)], moveit_plan=plan[1])
                    yield (arm_conf, base_conf)
                else:
                    rospy.logwarn("Failed to find an IK arm plan")
                    return None
            else:
                rospy.logwarn("Failed to find IK base plan")
                return None


        except MoveItCommanderException as e:
            rospy.logwarn(f"Failed to find an IK solution: {e}")
            return None
        finally:
            whole_body.clear_pose_targets()

    return fn

# def get_ik_fn(whole_body, arm_group):
#     move_group = whole_body
#     HAND_TF = "hand_palm_link"
#     DESIRED_DISTANCE = 0.5

#     def fn(arm, obj, obj_pose, grasp):
#         # grasp_pose will be relative to the object
#         try:
#             rospy.loginfo(f"Object Pose: {obj_pose}")
#             rospy.loginfo(f"Grasp Pose (in object frame): {grasp}")
#             obj_matrix = pose_to_matrix(obj_pose)
#             grasp_matrix = pose_to_matrix(grasp)

#             world_grasp_matrix = np.dot(obj_matrix, grasp_matrix) # grasp_pose -> world
#             world_grasp_pose = matrix_to_pose(world_grasp_matrix)
#             rospy.loginfo(f"Grasp Pose (in world frame): {world_grasp_pose}")

#             angle_to_obj = np.arctan2(world_grasp_pose.position.y, world_grasp_pose.position.x)
#             base_x = world_grasp_pose.position.x - DESIRED_DISTANCE*np.cos(angle_to_obj)
#             base_y = world_grasp_pose.position.y - DESIRED_DISTANCE*np.sin(angle_to_obj)
#             base_theta = angle_to_obj
#             target_pose = PoseStamped()
#             target_pose.header.frame_id = 'map' # Since obj_pose is now in world frame
#             target_pose.pose = world_grasp_pose

#             whole_body.set_pose_reference_frame("map")
#             whole_body.set_pose_target(target_pose, HAND_TF) # target for EE
#             whole_body.set_joint_value_target("world_joint", [base_x, base_y, base_theta])

#             plan = whole_body.plan()
#             if plan[0]:
#                 rospy.loginfo("Found a valid IK plan")
#                 arm_joint_values = [plan[1].joint_trajectory.points[-1].positions[i] for i in range(6)] # 6 joints in arm
#                 base_joint_values = whole_body.get_current_joint_values()[:3] # x,y,theta
#                 rospy.loginfo(f"Arm Joint Values: {arm_joint_values}")
#                 rospy.loginfo(f"Base Joint Values: {base_joint_values}")
#                 arm_conf = Conf(arm_group, arm_group.get_active_joints(), arm_joint_values, moveit_plan=plan[1])
#                 base_conf = Conf(whole_body, ["world_joint"], base_joint_values)
#                 yield (arm_conf, base_conf)
#             else:
#                 rospy.logwarn("Failed to find IK plan")
#                 return None



#             # base_conf -> base pose
#             # DESIRED_DISTANCE = 0.5
#             # angle_to_obj = np.arctan2(
#             #     world_grasp_pose.position.y,
#             #     world_grasp_pose.position.x,
#             # )
#             # base_x = world_grasp_pose.position.x - DESIRED_DISTANCE*np.cos(angle_to_obj)
#             # base_y = world_grasp_pose.position.y - DESIRED_DISTANCE*np.sin(angle_to_obj)
#             # base_theta = angle_to_obj
#             # move_group.clear_pose_targets()
#             # move_group.set_joint_value_target("world_joint", [base_x, base_y, base_theta])

#             # print(arm_group.get_active_joints())
#             # # move_group.set_pose_target(target_pose)
#             # plan = move_group.plan() # Find a plan, only for the base conf
#             # if not plan[0]:
#             #     rospy.logwarn("Failed to find base IK solution")
#             #     rospy.logwarn("Moveit error code: %s"%plan[-1])
#             #     return None
#             # base_conf = Conf(move_group, ['world_joint'], [base_x, base_y, base_theta], moveit_plan=plan[1])

#             # # target_pose -> End-effector pose; Use hand_tf = "hand_palm_link" here
#             # move_group.clear_pose_targets()
#             # move_group.set_end_effector_link("hand_palm_link")
#             # target_pose = PoseStamped()
#             # # target_pose.header.frame_id = HAND_TF # DOING THIS LEADS TO FAILURE IN FINDING PLAN
#             # target_pose.pose.position = world_grasp_pose.position
#             # target_pose.pose.orientation = world_grasp_pose.orientation
#             # arm_joints = [
#             #     'arm_lift_joint',
#             #     'arm_flex_joint',
#             #     'arm_roll_joint',
#             #     'wrist_flex_joint',
#             #     'wrist_roll_joint',
#             #     'wrist_ft_sensor_frame_joint'
#             # ]
#             # # move_group.set_joint_value_target(arm_joints, )
#             # # joint_goal = move_group.get_joint_value_target() # Moveit IK solver to get Arm Conf

#             # success = move_group.set_pose_target(target_pose, "hand_palm_link") # Do we need this?
#             # plan = move_group.plan()
#             # if plan[0]:
#             #     joint_values = move_group.get_joint_value_target()
#             #     breakpoint()
#             #     arm_conf = Conf(move_group, arm_joints, joint_values, moveit_plan=plan[1])
#             #     move_group.clear_pose_targets()
#             #     return (arm_conf, base_conf)

#             # rospy.logwarn("Failed to find an IK solution")
#             # return None

#             # joint_goal = move_group.get_joint_value_target()
#             # move_group.clear_pose_targets()
#             # arm_joints = [
#             #     'arm_lift_joint',
#             #     'arm_flex_joint',
#             #     'arm_roll_joint',
#             #     'wrist_flex_joint',
#             #     'wrist_roll_joint',
#             #     'wrist_ft_sensor_frame_joint'
#             # ]
#             # arm_conf = Conf(move_group, arm_joints, joint_goal)
#             # return (arm_conf, base_conf)
#         # try:
#         #     joint_goal = move_group.get_joint_value_target()
#         #     move_group.clear_pose_targets()
#         #     q = Conf(move_group, move_group.get_active_joints(), joint_goal)
#         #     return (q,)
#         except MoveItCommanderException as e:
#             rospy.logwarn(f"Failed to find an IK solution: {e}")
#             return None
#         finally:
#             move_group.clear_pose_targets()

#     return fn


def pose_to_matrix(pose):
    trans_matrix = T.translation_matrix((
        pose.position.x,
        pose.position.y,
        pose.position.z
    ))

    rot_matrix = T.quaternion_matrix((
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ))

    return np.dot(trans_matrix, rot_matrix)

def matrix_to_pose(matrix):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quat = T.quaternion_from_matrix(matrix)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose



def get_ik_fn_old(whole_body, arm_group):
    # This generates correct arm conf, but very wrong base conf
    move_group = whole_body
    HAND_TF = "hand_palm_link"

    def fn(arm, obj, obj_pose, grasp):
        # grasp_pose will be relative to the object
        try:
            rospy.loginfo(f"Object Pose: {obj_pose}")
            rospy.loginfo(f"Grasp Pose (in object frame): {grasp}")
            obj_matrix = pose_to_matrix(obj_pose)
            grasp_matrix = pose_to_matrix(grasp)

            world_grasp_matrix = np.dot(obj_matrix, grasp_matrix) # grasp_pose -> world
            world_grasp_pose = matrix_to_pose(world_grasp_matrix)
            rospy.loginfo(f"Grasp Pose (in world frame): {world_grasp_pose}")

            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map' # Since obj_pose is now in world frame
            target_pose.pose = world_grasp_pose
            whole_body.set_pose_target(target_pose, HAND_TF) # target for EE
            plan = whole_body.plan()
            if plan[0]:
                rospy.loginfo("Found a valid IK plan")
                arm_joint_values = [plan[1].joint_trajectory.points[-1].positions[i] for i in range(6)] # 6 joints in arm
                base_joint_values = whole_body.get_current_joint_values()[:3] # x,y,theta
                rospy.loginfo(f"Arm Joint Values: {arm_joint_values}")
                rospy.loginfo(f"Base Joint Values: {base_joint_values}")
                arm_conf = Conf(arm_group, arm_group.get_active_joints(), arm_joint_values, moveit_plan=plan[1])
                base_conf = Conf(whole_body, ["world_joint"], base_joint_values)
                yield (arm_conf, base_conf)
            else:
                rospy.logwarn("Failed to find IK plan")
                return None
        except MoveItCommanderException as e:
            rospy.logwarn(f"Failed to find an IK solution: {e}")
            return None
        finally:
            move_group.clear_pose_targets()

    return fn
