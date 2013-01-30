#!/usr/bin/env python

import roslib
roslib.load_manifest('trajopt_plugin')
import rospy

from moveit_msgs.msg import *
from geometry_msgs.msg import *
from shape_msgs.msg import *

import actionlib

import IPython
from tf.transformations import *
import time
import sqlite3
import numpy as np

def build_motion_plan_request(pos, quat, arm="right"):
    m = MoveGroupGoal()
    m.request.group_name = "%s_arm" % arm
    target_link = "%s_wrist_roll_link" % arm[0]
    m.request.start_state.joint_state.name = ['br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint', 'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint']
    m.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.657967, 0.888673, -1.4311, -1.073419, -0.705232, -1.107079, 2.806742, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.848628, 0.7797, 1.396294, -0.828274, 0.687905, -1.518703, 0.394348, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    m.request.start_state.multi_dof_joint_state.joint_names =  ['world_joint']
    m.request.start_state.multi_dof_joint_state.frame_ids = ['odom_combined']
    m.request.start_state.multi_dof_joint_state.child_frame_ids = ['base_footprint']
    base_pose = Pose()
    base_pose.orientation.w = 1
    m.request.start_state.multi_dof_joint_state.poses = [ base_pose ]

    pc = PositionConstraint()
    pc.link_name = target_link
    pc.header.frame_id = 'odom_combined'
    pose = Pose()
    pose.position = pos

    pc.constraint_region.primitive_poses = [pose]
    
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [.01]
    pc.constraint_region.primitives = [ sphere ]
    
    oc = OrientationConstraint()
    oc.link_name = target_link
    oc.header.frame_id = 'odom_combined'
    oc.orientation = quat
    c = Constraints()
    c.position_constraints = [ pc ]
    c.orientation_constraints = [ oc ]
    m.request.goal_constraints = [ c ]
    
    return m

def test_grid(center_point, x_range=0.1, y_range=0.2, z_range=0.2, dx=0.05, dy=0.05, dz=0.05):
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    
    print "Waiting for server"
    client.wait_for_server()
    print "Connected to actionserver"

    for xp in np.arange(center_point.x - x_range, center_point.x + x_range, dx):
        for yp in np.arange(center_point.y - y_range, center_point.y + y_range, dy):
            for zp in np.arange(center_point.z - z_range, center_point.z + z_range, dz):
                p = Point()
                p.x = xp
                p.y = yp
                p.z = zp
                print "Sending planning request to point", p
                q = Quaternion() # TODO: Configure orientation
                q.w = 1
                m = build_motion_plan_request(p, q)
                client.send_goal(m)
                t1 = time.time()
                client.wait_for_result()
                t2 = time.time()
                result = client.get_result()
                print "Motion planning request took", (t2-t1), "seconds"
    
    
def test_single():
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    
    print "Waiting for server"
    client.wait_for_server()
    print "Connected to actionserver"
    p = Point()
    p.x = .5
    p.y = -.01
    p.z = 0.94
    q = Quaternion()
    q.w = 1
    m = build_motion_plan_request(p, q)
    client.send_goal(m)
    t1 = time.time()
    client.wait_for_result()
    t2 = time.time()
    result = client.get_result()
    print "Motion planning request took", (t2-t1), "seconds"
    print result

if __name__ == "__main__":
    rospy.init_node("foobar")
    p = Point()
    p.x = .5
    p.y = -.01
    p.z = 0.94
    test_grid(p, x_range=0.1, y_range=0.2, z_range=0.2)
