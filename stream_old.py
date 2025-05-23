from primitives import Trajectory, Commands, State, Conf, PoseStamped as OurPoseStamped
from geometry_msgs.msg import PoseStamped
from tf import transformations as T
import numpy as np
from utils import compute_grasp_poses
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

def get_grasp_gen(whole_body, obj_pose, hsrb_gripper):
    def gen(obj):
        bbox = Marker()
        bbox.header.frame_id = "map"
        bbox.pose = obj_pose.pose
        if obj == "ycb_003_cracker_box":
            bbox.scale.x = 0.06  # cracker box dimensions
            bbox.scale.y = 0.16
            bbox.scale.z = 0.21
        else:
            bbox.scale.x = 0.01
            bbox.scale.y = 0.01
            bbox.scale.z = 0.01

        grasps = compute_grasp_poses(
            bbox,
            top=False, 
            side=True,
            relative=False,
            side_up=True,
            rigid=True,
            off_center=False,
            pre_grasp=0.06
        )
        possible_grasps = []
        possible_base_confs = []

        hsrb_gripper.command(1.0)
        for grasp in grasps:
            whole_body.set_pose_target(grasp)
            plan = whole_body.plan()
            if plan[0]:
                # Calculate base position for this grasp
                angle_to_obj = np.arctan2(grasp.pose.position.y, grasp.pose.position.x)
                base_x = grasp.pose.position.x - 0.5*np.cos(angle_to_obj)
                base_y = grasp.pose.position.y - 0.5*np.sin(angle_to_obj)
                base_theta = angle_to_obj
                
                base_conf = Conf(whole_body, ["world_joint"], [base_x, base_y, base_theta])
                converted_grasp = OurPoseStamped()
                converted_grasp.header.frame_id = grasp.header.frame_id
                converted_grasp.pose = grasp.pose
                converted_grasp.plan = plan[1]
                possible_grasps.append(converted_grasp)
                possible_base_confs.append(base_conf)
            
        return [(g,b,) for g,b in zip(possible_grasps, possible_base_confs)]
            
    return gen

def get_place_gen(whole_body, table_pose):
    def gen(obj):
        table_x = table_pose.position.x
        table_y = table_pose.position.y
        table_z = table_pose.position.z + 0.21/2

        x_range = np.linspace(table_x - 0.2, table_x + 0.2, 3)
        y_range = np.linspace(table_y - 0.2, table_y + 0.2, 3)

        for x in x_range:
            for y in y_range:
                place_pose = PoseStamped()
                place_pose.header.frame_id = "map"
                place_pose.pose.position = Point(x, y, table_z)
                place_pose.pose.orientation = Quaternion(0, 0, 0, 1)

                # Create bbox for grasp generation
                bbox = Marker()
                bbox.header.frame_id = "map"
                bbox.pose = place_pose.pose
                bbox.scale.x = 0.06
                bbox.scale.y = 0.16
                bbox.scale.z = 0.21

                grasps = compute_grasp_poses(
                    bbox, top=False, side=True, relative=False, side_up=True
                )

                for grasp in grasps:
                    whole_body.set_pose_target(grasp)
                    plan = whole_body.plan()
                    if plan[0]:
                        angle_to_pose = np.arctan2(
                            grasp.pose.position.y, grasp.pose.position.x
                        )
                        base_x = grasp.pose.position.x - 0.5 * np.cos(angle_to_pose)
                        base_y = grasp.pose.position.y - 0.5 * np.sin(angle_to_pose)
                        base_conf = Conf(
                            whole_body, ["world_joint"], [base_x, base_y, angle_to_pose]
                        )
                        converted_grasp = OurPoseStamped()
                        converted_grasp.header.frame_id = grasp.header.frame_id
                        converted_grasp.pose = grasp.pose
                        converted_grasp.plan = plan[1]
                        yield (place_pose, grasp, base_conf)

    return gen

# def get_pose_gen(whole_body, table_pose):
#     def gen(obj):

#                 yield (place_pose,)
#     return gen
