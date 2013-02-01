#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile")
args = parser.parse_args()
import subprocess, os

envfile = "/tmp/%s.xml"%os.path.basename(args.scenefile)
subprocess.check_call("python scene2xml.py %s %s"%(args.scenefile, envfile), shell=True)


import roslib
import sys
sys.path.append("/home/joschu/ros/moveit/devel/lib/python2.7/dist-packages")
sys.path.append("/home/ibrahima/moveit/devel/lib/python2.7/dist-packages")
import rospy

from moveit_msgs.msg import *
from moveit_msgs.srv import *
from geometry_msgs.msg import *
from shape_msgs.msg import *
from trajopt_plugin.srv import *

import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku

import actionlib

import time
import numpy as np
import json
import openravepy as rave, numpy as np



ROS_JOINT_NAMES = ['br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint', 'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint']
ROS_DEFAULT_JOINT_VALS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.657967, 0.888673, -1.4311, -1.073419, -0.705232, -1.107079, 2.806742, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.848628, 0.7797, 1.396294, -0.828274, 0.687905, -1.518703, 0.394348, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def has_collision(traj, manip):
    traj_up = mu.interp2d(np.linspace(0,1,100), np.linspace(0,1,len(traj)), traj)
    robot = manip.GetRobot()
    ss = rave.RobotStateSaver(robot)
    arm_inds = manip.GetArmIndices()
    env = robot.GetEnv()
    collision = False
    col_times = []
    for (i,row) in enumerate(traj_up):
        robot.SetDOFValues(row, arm_inds)
        col_now = env.CheckCollision(robot)
        if col_now: 
            collision = True
            col_times.append(i)
    if col_times: print "collision at timesteps", col_times      
    else: print "no collisions"
    return collision

def build_joint_request(jvals, arm, robot):
    m = MotionPlanRequest()    
    m.group_name = "%s_arm" % arm
    m.start_state.joint_state.name = ROS_JOINT_NAMES
    m.start_state.joint_state.position = ROS_DEFAULT_JOINT_VALS
    m.start_state.multi_dof_joint_state.joint_names =  ['world_joint']
    m.start_state.multi_dof_joint_state.frame_ids = ['odom_combined']
    m.start_state.multi_dof_joint_state.child_frame_ids = ['base_footprint']
    base_pose = Pose()
    base_pose.orientation.w = 1
    m.start_state.multi_dof_joint_state.poses = [ base_pose ]
    
    
    c = Constraints()
    joints = robot.GetJoints()
    joint_inds = robot.GetManipulator("%sarm"%arm).GetArmIndices()
    c.joint_constraints = [JointConstraint(joint_name=joints[joint_inds[i]].GetName(), position = jvals[i])
                           for i in xrange(len(jvals))]
    jc = JointConstraint()
                                          
    m.goal_constraints = [c]
    return m
    

def build_cart_request(pos, quat, arm):
    m = MotionPlanRequest()
    m.group_name = "%s_arm" % arm
    target_link = "%s_wrist_roll_link" % arm[0]
    m.start_state.joint_state.name = ROS_JOINT_NAMES
    m.start_state.joint_state.position = ROS_DEFAULT_JOINT_VALS

    m.start_state.multi_dof_joint_state.joint_names =  ['world_joint']
    m.start_state.multi_dof_joint_state.frame_ids = ['odom_combined']
    m.start_state.multi_dof_joint_state.child_frame_ids = ['base_footprint']
    base_pose = Pose()
    base_pose.orientation.w = 1
    m.start_state.multi_dof_joint_state.poses = [ base_pose ]

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
    m.goal_constraints = [ c ]
    
    return m
<<<<<<< HEAD:src/move_group_battery.py
    
def test_plan_to_pose(xyz, xyzw, leftright, robot):
    manip = robot.GetManipulator(leftright + "arm")
=======

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
                
                if m.request.group_name == "right_arm": manipname = "rightarm"
                elif m.request.group_name == "left_arm": manipname = "leftarm"
                else: raise Exception("invalid group name")

                if rospy.is_shutdown(): return
    
    
def test_plan_to_pose(xyz, xyzw, leftright, robot):
    manip = robot.GetManipulator(leftright + "arm")
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)    
>>>>>>> 5d0ae256a280b6694a4f03ed492c8500b4cca424:src/test_move_group.py

    joint_solutions = ku.ik_for_link(rave.matrixFromPose(np.r_[xyzw[3], xyzw[:3], xyz]), manip, "%s_gripper_tool_frame"%leftright[0], 
                         1, True)


    if len(joint_solutions) == 0:
        print "pose is not reachable"
        return None

    m = build_joint_request(joint_solutions[0], leftright, robot)

<<<<<<< HEAD:src/move_group_battery.py
    t1 = time.time()
    t2 = time.time()

    response = get_motion_plan(m).motion_plan_response
    assert isinstance(response, MotionPlanResponse)
=======
    get_motion_plan = rospy.ServiceProxy('plan_kinematic_path', GetMotionPlan)
    t1 = time.time()
    response = get_motion_plan(m.request).motion_plan_response
    t2 = time.time()

    print response.planning_time
>>>>>>> 5d0ae256a280b6694a4f03ed492c8500b4cca424:src/test_move_group.py
    traj =  [list(jtp.positions) for jtp in response.trajectory.joint_trajectory.points]
    if response is not None:
        return dict(returned = True, safe = not has_collision(traj, manip), traj = traj, planning_time = response.planning_time)
    else:
        raise dict(returned = False)

    
def update_rave_from_ros(robot, ros_values, ros_joint_names):
    inds_ros2rave = np.array([robot.GetJointIndex(name) for name in ros_joint_names])
    good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
    rave_inds = inds_ros2rave[good_ros_inds] # openrave indices corresponding to those joints
    rave_values = [ros_values[i_ros] for i_ros in good_ros_inds]
    robot.SetJointValues(rave_values[:20],rave_inds[:20])
    robot.SetJointValues(rave_values[20:],rave_inds[20:])   
    
get_motion_plan = None
env = None
    
def main():
    global get_motion_plan, env, robot
    if rospy.get_name() == "/unnamed":
        rospy.init_node("move_group_battery")
    env = rave.Environment()
    env.Load("robots/pr2-beta-static.zae")
    loadsuccess = env.Load(envfile)    
    assert loadsuccess
    
<<<<<<< HEAD:src/move_group_battery.py
    get_motion_plan = rospy.ServiceProxy('trajopt_planner', GetMotionPlan)    
    print "waiting for trajopt_planner"
    get_motion_plan.wait_for_service()
    print "ok"
    
    robot = env.GetRobots()[0]
    update_rave_from_ros(robot, ROS_DEFAULT_JOINT_VALS, ROS_JOINT_NAMES)
    

    xs, ys, zs = np.mgrid[.35:.65:.05, 0:.5:.05, .8:.9:.1]
=======
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)    
    print "Waiting for server"
    client.wait_for_server()
    print "Connected to actionserver"
        
    robot = env.GetRobots()[0]
    update_rave_from_ros(robot, ROS_DEFAULT_JOINT_VALS, ROS_JOINT_NAMES)
    
    xs, ys, zs = np.mgrid[.35:.85:.05, 0:.5:.05, .8:.9:.1]
>>>>>>> 5d0ae256a280b6694a4f03ed492c8500b4cca424:src/test_move_group.py
    results = []
    for (x,y,z) in zip(xs.flat, ys.flat, zs.flat):
        result = test_plan_to_pose([x,y,z], [0,0,0,1], "left", robot)
        if result is not None: results.append(result)
        
    success_count, fail_count, no_answer_count = 0,0,0
    for result in results:
        if result["returned"]:
            if result["safe"]: success_count += 1
            else: fail_count += 1
        else:
            no_answer_count += 1
    print "success count:", success_count
    print "fail count:", fail_count
    print "no answer count:", no_answer_count

main()
