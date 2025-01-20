import random
import moveit_commander
import copy
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
    odom_to_map
)
from sensor_msgs.msg import JointState
GRASPS = list()

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


def get_grasp_gen(whole_body, scene, robot_state):
    def gen(obj):
        whole_body.set_start_state(robot_state)
        width, length, height = (0.06, 0.16, 0.21) if obj == 'ycb_003_cracker_box' else (0.1, 0.1, 0.1)

        grasps = []
        object_pose = scene.get_object_poses([obj])[obj] # scene returns in odom frame because planning_frame is odom. Idk how to change that.

        init_quat = (0.707, 0.0, 0.707, 0.0)  # copied from moveit_pick_and_place_demo.py 

        for angle in [0, np.pi/2, np.pi, -np.pi/2]:
            side_grasp = PoseStamped()
            offset = width/2 + 0.1
            side_grasp.pose.position = Point(
                offset * np.cos(angle),
                offset * np.sin(angle),
                height/2
            )
            quat = T.quaternion_from_euler(0, 0, angle)
            side_grasp.pose.orientation = Quaternion(*quat)

            moveit_grasp = Grasp()
            moveit_grasp.id = f"grasp_{len(grasps)}"
            moveit_grasp.pre_grasp_posture = make_gripper_posture(0.8) # Open gripper
            moveit_grasp.grasp_posture = make_gripper_posture(0.2, -0.02) # Close gripper
            moveit_grasp.pre_grasp_approach = make_gripper_translation(0.05, 0.1, [0,0,1])
            moveit_grasp.post_grasp_retreat = make_gripper_translation(0.05, 0.1, [0,0,1], "base_footprint")
            init = T.quaternion_multiply(init_quat, quat)
            # moveit_grasp.grasp_pose = make_pose(
            #     offset * np.cos(angle),
            #     offset * np.sin(angle),
            #     height / 2,
            #     0,
            #     0,
            #     angle,
            #     reference_frame="odom",
            #     init=init,
            # )
            grasp_pose_ee = convert_obj_frame_to_ee(object_pose, side_grasp.pose)
            # side_grasp.pose = grasp_pose_ee
            moveit_grasp.grasp_pose = copy.deepcopy(side_grasp)
            moveit_grasp.grasp_pose.pose = grasp_pose_ee
            GRASPS.append(moveit_grasp)
            # moveit_grasp.grasp_pose = side_grasp

            # moveit_grasp.grasp_pose = side_grasp
            moveit_grasp.allowed_touch_objects = [obj]
            moveit_grasp.max_contact_force = 0.5

            # side_grasp.moveit_grasp = moveit_grasp
            grasps.append((side_grasp, moveit_grasp))

        return [(g[0],) for g in grasps] # For PDDLStream, return only the Poses
    return gen

    # def get_grasp_gen(move_group, scene):
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


def convert_obj_frame_to_ee(obj_pose, grasp_pose):
    hand_correction = np.array([
        [1, 0,  0, 0],
        [0, 0, -1, 0],
        [0, 1,  0, 0],
        [0, 0,  0, 1]
    ])
    obj_pose_map = odom_to_map(obj_pose).pose
    obj_matrix = pose_to_matrix(obj_pose_map)
    grasp_matrix = pose_to_matrix(grasp_pose)
    world_grasp_matrix = np.dot(obj_matrix, grasp_matrix) # obj_frame -> map
    corrected_world_grasp_matrix = np.dot(world_grasp_matrix, hand_correction) # map -> EE?
    world_grasp_pose = matrix_to_pose(corrected_world_grasp_matrix)
    return world_grasp_pose


def get_ik_fn(whole_body, arm_group, robot_state):
    # This generates correct arm conf, but very wrong base conf
    move_group = whole_body
    HAND_TF = "hand_palm_link"
    DESIRED_DISTANCE = 0.5
    POSITION_TOLERANCE = 0.5
    hand_correction = np.array([
        [1, 0,  0, 0],
        [0, 0, -1, 0],
        [0, 1,  0, 0],
        [0, 0,  0, 1]
    ])
    def verify_end_effector_position(group, target_pose, base_values, arm_values):
        try:
            # First move the base to planned position
            group.set_joint_value_target("world_joint", base_values)
            
            # Then set arm joint values
            for joint, value in zip(arm_group.get_active_joints(), arm_values):
                group.set_joint_value_target(joint, value)
            
            # Get resulting end effector pose
            end_effector_pose = group.get_current_pose(HAND_TF)
            
            # Compare with target
            dx = abs(end_effector_pose.pose.position.x - target_pose.pose.position.x)
            dy = abs(end_effector_pose.pose.position.y - target_pose.pose.position.y)
            dz = abs(end_effector_pose.pose.position.z - target_pose.pose.position.z)
            
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            rospy.loginfo(f"Distance between planned and target: {distance}m")
            rospy.loginfo(f"Planned end effector: {end_effector_pose.pose}")
            rospy.loginfo(f"Target pose: {target_pose.pose}")
            
            return distance < POSITION_TOLERANCE
            
        finally:
            group.clear_pose_targets()

    def fn(arm, obj, obj_pose, grasp):
        # grasp_pose will be relative to the object
        grasp_pose = grasp.pose
        try:
            rospy.loginfo(f"Object Pose (in odom): {obj_pose}")
            object_pose_map = odom_to_map(obj_pose).pose
            rospy.loginfo(f"Object Pose (in map): {object_pose_map}")
            rospy.loginfo(f"Grasp Pose (in object frame): {grasp_pose}")
            move_group.set_start_state(robot_state)
            rospy.loginfo("Set start state for planning")
            
            
            obj_matrix = pose_to_matrix(object_pose_map)
            grasp_matrix = pose_to_matrix(grasp_pose)

            # correction_matrix = T.euler_matrix(0, -np.pi/2, 0) # ;; added now
            # corrected_grasp_matrix = np.dot(grasp_matrix, correction_matrix) # ;; added now
            # corrected_world_grasp_pose = matrix_to_pose(corrected_grasp_matrix) # ;; added now

            world_grasp_matrix = np.dot(obj_matrix, grasp_matrix) # obj_frame -> map
            corrected_world_grasp_matrix = np.dot(world_grasp_matrix, hand_correction) # map -> EE?
            world_grasp_pose = matrix_to_pose(corrected_world_grasp_matrix)
            world_grasp_pose.position.x += 0.1 # ;; added now
            # grasp.pose = world_grasp_pose
            rospy.loginfo(f"Grasp Pose (in world frame): {world_grasp_pose}")

            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map' # Since obj_pose is now in world frame
            # target_pose.pose = corrected_world_grasp_pose
            target_pose.pose = world_grasp_pose


            # angle_to_obj = np.arctan2(world_grasp_pose.position.y, world_grasp_pose.position.x)
            # base_x = world_grasp_pose.position.x - DESIRED_DISTANCE*np.cos(angle_to_obj)
            # base_y = world_grasp_pose.position.y - DESIRED_DISTANCE*np.sin(angle_to_obj)
            # base_theta = angle_to_obj
          
            # whole_body.set_joint_value_target("world_joint", [base_x, base_y, base_theta])
            whole_body.set_pose_target(target_pose) # target for EE
            whole_body.allow_replanning(True)
            base_plan = whole_body.plan()
            if base_plan[0]:
                # arm_plan = arm_group.plan()
                # if arm_plan[0]:
                rospy.loginfo("Found a valid IK plan, verifying end effector position...")
                arm_joint_values = [base_plan[1].joint_trajectory.points[-1].positions[i] for i in range(6)] # 6 joints in arm
                base_joint_values = whole_body.get_current_joint_values()[:3] # x,y,theta
                rospy.loginfo("End effector position verified!")
                arm_conf = Conf(arm_group, arm_group.get_active_joints(), arm_joint_values, moveit_plan=base_plan[1])
                base_conf = Conf(whole_body, ["world_joint"], base_joint_values)
                yield (arm_conf, base_conf)
                    
                    # if verify_end_effector_position(whole_body, target_pose, base_joint_values, arm_joint_values):
                    #     return None
                    # else:
                    #     rospy.logwarn("End effector would not reach target position, trying another solution...")

                    
                    # rospy.loginfo(f"Arm Joint Values: {arm_joint_values}")
                    # rospy.loginfo(f"Base Joint Values: {base_joint_values}")
                    # arm_conf = Conf(arm_group, arm_group.get_active_joints(), arm_joint_values, moveit_plan=arm_plan[1])
                    # base_conf = Conf(whole_body, ["world_joint"], base_joint_values, moveit_plan=base_plan[1])
                    # yield (arm_conf, base_conf)
                # else:
                #     rospy.logwarn("Failed to find IK plan for arm")
                #     return None
            else:
                rospy.logwarn("Failed to find IK plan for base")
                return None
        except MoveItCommanderException as e:
            rospy.logwarn(f"Failed to find an IK solution: {e}")
            return None
        finally:
            move_group.clear_pose_targets()
            arm_group.clear_pose_targets()

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
