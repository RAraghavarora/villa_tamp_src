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
import math

GRIPPER_SIZE = 0.135 + 0.02  # Allow some extra room since the bounding box is not tight
GRIPPER_DEPTH = 0.02


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



def compute_grasp_poses(bbox, top=False, side=True, relative=False, side_up=False, rigid=True, off_center=False,
                        pre_grasp=0.0):
    '''
    Given a (visualization_msgs/Marker) bounding box
    Computes potential grasp poses for the motion planner to consume

    :param bbox: bounding box Marker in the 'ground' frame
    '''

    grasp_poses = []

    # Axes of the object to align to
    axes_to_align = []
    gripper_rotation_to_object = []
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

    # Assuming z points out in gripper axis
    gripper_axis = np.array([0., 0., 1.])

    # I want to face the opposite direction
    # ----- objaxis ---> (offset)  <----- gripper axis -----

    # Top and side grasps
    if top:
        axes_to_align.append(np.array([0., 0., 1.]))
        gripper_rotation_to_object.append(T.rotation_matrix(np.pi, xaxis))  #  -- obj_z -> (offset) <- gripper z --, x aligned

    if side:
        axes_to_align.extend([np.array([1., 0., 0.]),
                              np.array([-1., 0., 0.])])
        gripper_rotation_to_object.extend([T.rotation_matrix(-np.pi / 2, yaxis),  # gripper x points up, z points to negative x
                                           T.rotation_matrix(-np.pi / 2, yaxis) @ T.rotation_matrix(np.pi, xaxis)])  # gripper z points to x

        axes_to_align.extend([np.array([0., 1., 0.]),
                              np.array([0., -1., 0.])])
        gripper_rotation_to_object.extend([T.rotation_matrix(np.pi / 2, xaxis) @ T.rotation_matrix(np.pi / 2, zaxis),
                                           T.rotation_matrix(-np.pi / 2, xaxis) @ T.rotation_matrix(np.pi / 2, zaxis)])

    scale = [bbox.scale.x, bbox.scale.y, bbox.scale.z]
    for ax, gripper_rotation in zip(axes_to_align, gripper_rotation_to_object):
        ax_id = np.nonzero(ax)[0][0]

        # Make sure the grasp pose is not in the object
        gripper_offset = max(0.0, 0.5 * scale[ax_id] - 0.05) + GRIPPER_DEPTH
        gripper_offset += pre_grasp

        # We want to offset a lil' bit
        gripper_offset_to_obj = np.dot(gripper_rotation, T.translation_matrix(-gripper_offset * gripper_axis))

        R = T.quaternion_matrix(
            [bbox.pose.orientation.x, bbox.pose.orientation.y, bbox.pose.orientation.z, bbox.pose.orientation.w])
        t = T.translation_matrix([bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z])
        obj_to_ground = np.dot(t, R)

        if ax_id < 2:  # side grasp
            axis_between_gripper = 1 - ax_id
            if rigid and scale[axis_between_gripper] > GRIPPER_SIZE or (rigid and scale[2] < 0.02) or \
                    (not rigid and scale[axis_between_gripper] > scale[ax_id]):  # if not rigid, allow grasp along the shorter dimension
                gripper_angles = []
            elif side_up:
                gripper_angles = [0]
            else:
                gripper_angles = [0, math.pi]
        else:  # top grasp
            gripper_angles = []
            if scale[0] < GRIPPER_SIZE:  # x axis is between gripper
                gripper_angles.extend([-math.pi / 2, math.pi / 2])
            if scale[1] < GRIPPER_SIZE:  # y axis is between gripper
                gripper_angles.extend([-math.pi, 0])
            if not rigid and scale[0] >= GRIPPER_SIZE and scale[1] >= GRIPPER_SIZE:  # if not rigid, allow grasp along the shorter dimension
                if scale[0] < scale[1]:
                    gripper_angles.extend([-math.pi / 2, math.pi / 2])
                else:
                    gripper_angles.extend([-math.pi, 0])

        for angle in gripper_angles:
            r = T.rotation_matrix(angle, np.array([0., 0., 1.]))
            gripper_to_obj_rotated = np.dot(gripper_offset_to_obj, r)

            if ax_id < 2 and scale[2] < 0.08:
                x_offsets = [0.04]
            elif ax_id < 2 and scale[2] > 0.2 and off_center:
                # not top grasp, add offsets
                # if side_up:
                #     x_offsets = [scale[2] / 2 - 0.05, 0]
                # else:
                x_offsets = [scale[2] / 2 - 0.05, 0, -(scale[2] / 2 - 0.05)]
            else:
                x_offsets = [0]

            for x_offset in x_offsets:
                trans = T.translation_matrix([x_offset, 0, 0])
                gripper_to_obj = gripper_to_obj_rotated @ trans

                if relative:
                    position = gripper_to_obj[:3, 3]
                    rot = np.eye(4)
                    rot[:3, :3] = gripper_to_obj[:3, :3]
                else:
                    gripperToGround = np.dot(obj_to_ground, gripper_to_obj)
                    position = gripperToGround[:3, 3]
                    rot = np.eye(4)
                    rot[:3, :3] = gripperToGround[:3, :3]

                quat = T.quaternion_from_matrix(rot)
                pose = PoseStamped()
                pose.header.frame_id = bbox.header.frame_id
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.position.z = position[2]
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                print (ax_id, angle, gripper_to_obj[:3, 3])
                grasp_poses.append(pose)

    return grasp_poses

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
