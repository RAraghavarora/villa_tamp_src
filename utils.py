import rospy
import tf2_ros
import tf2_geometry_msgs # Crucial to import
import trajectory_msgs
import numpy as np
from moveit_msgs.msg import GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander.exception import MoveItCommanderException
from tf import transformations as T
from geometry_msgs.msg import PoseStamped
from primitives import Conf
from tf.transformations import quaternion_from_euler, quaternion_multiply

def make_gripper_posture(pos, effort=0.0):
    t = JointTrajectory()
    t.joint_names = ["hand_motor_joint"]
    tp = trajectory_msgs.msg.JointTrajectoryPoint()
    tp.positions = [pos]
    tp.effort = [effort]
    tp.time_from_start = rospy.Duration(2.0)
    t.points.append(tp)
    return t


def make_gripper_translation(min_dist, desired, vector, frame=None):
    g = GripperTranslation()
    g.direction.vector.x = vector[0]
    g.direction.vector.y = vector[1]
    g.direction.vector.z = vector[2]
    g.direction.header.frame_id = frame or "hand_palm_link"
    g.min_distance = min_dist
    g.desired_distance = desired
    return g

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

def make_pose(x, y, z, roll, pitch, yaw, reference_frame="map", init=(0.707, 0.0, 0.707, 0.0)):
    pose = PoseStamped()
    pose.header.frame_id = reference_frame
    q = quaternion_from_euler(roll, pitch, yaw)
    q = quaternion_multiply(init, q)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

def odom_to_map(object_pose):
    try:
        object_pose_stamped = PoseStamped()
        object_pose_stamped.header.frame_id = "odom"
        object_pose_stamped.header.stamp = rospy.Time(0)
        object_pose_stamped.pose = object_pose

        tf2_buffer = tf2_ros.Buffer()
        tf2_listener = tf2_ros.TransformListener(tf2_buffer)

        tf2_buffer.can_transform('map', 'odom', rospy.Time(0), rospy.Duration(5.0))
        object_pose_map = tf2_buffer.transform(object_pose_stamped, "map", rospy.Duration(5.0))
        return object_pose_map

    except (tf2_ros.LookupException, 
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Failed to transform pose from odom to map: {e}")