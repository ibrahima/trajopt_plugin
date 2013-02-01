#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile")
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import sys
sys.path.append("/home/joschu/ros/moveit/devel/lib/python2.7/dist-packages")
sys.path.append("/home/ibrahima/moveit/devel/lib/python2.7/dist-packages")


import subprocess, os
from time import time
import trajoptpy
import rospy
import json
import moveit_msgs.msg as mm
import openravepy as rave, numpy as np
import moveit_msgs.srv as ms
import trajoptpy.math_utils as mu
import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm

GROUPMAP = {
    "left_arm":"leftarm",
    "right_arm":"rightarm",    
}


def update_rave_from_ros(robot, ros_values, ros_joint_names):
    inds_ros2rave = np.array([robot.GetJointIndex(name) for name in ros_joint_names])
    good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
    rave_inds = inds_ros2rave[good_ros_inds] # openrave indices corresponding to those joints
    rave_values = [ros_values[i_ros] for i_ros in good_ros_inds]
    robot.SetJointValues(rave_values[:20],rave_inds[:20])
    robot.SetJointValues(rave_values[20:],rave_inds[20:])   

def callback(getreq):
    assert isinstance(getreq, ms.GetMotionPlanRequest)
    req = getreq.motion_plan_request   
    getresp = ms.GetMotionPlanResponse()
    resp = getresp.motion_plan_response
    
    manip = GROUPMAP[req.group_name]
    
    update_rave_from_ros(robot, req.start_state.joint_state.position, req.start_state.joint_state.name)
    
    start_joints = robot.GetDOFValues(robot.GetManipulator(manip).GetArmIndices())
    
    n_steps = 10
    coll_coeff = 10
    dist_pen = .04
    
    inds = robot.GetManipulator(manip).GetArmJoints()
    alljoints = robot.GetJoints()
    names = arm_joint_names = [alljoints[i].GetName() for i in inds]
    
    
    for initialization in [0]:
        d = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : manip,
                "start_fixed" : True
            },
            "costs" : [
            {
                "type" : "joint_vel",
                "params": {"coeffs" : [1]}
            },            
            {
                "type" : "continuous_collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen]}
            }
            ],
            "constraints" : [],
            "init_info" : {
                "type" : "given_traj"
            }
        }
        
        for goal_cnt in req.goal_constraints:
            if len(goal_cnt.joint_constraints) > 0:
                assert len(goal_cnt.joint_constraints)==7
                end_joints = np.zeros(7)
                for i in xrange(7):
                    jc = goal_cnt.joint_constraints[i]
                    dof_idx = arm_joint_names.index(jc.joint_name)
                    end_joints[dof_idx] = jc.position
                
                d["constraints"].append({
                    "type" : "joint",
                    "name" : "joint", 
                    "params" : {
                        "vals" : end_joints.tolist()
                    }                
                })
                
                d["init_info"]["data"] = [row.tolist() for row in mu.linspace2d(start_joints, end_joints, n_steps)]
    
                
            if len(goal_cnt.position_constraints) > 0:
                raise NotImplementedError
                
        
        trajoptpy.SetInteractive(args.interactive)
        s = json.dumps(d)
        t_start = time()
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        planning_duration_seconds = time() - t_start

    resp.trajectory.joint_trajectory.joint_names = names
    for row in result.GetTraj():
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = row.tolist()
        jtp.time_from_start = rospy.Duration(0)
        resp.trajectory.joint_trajectory.points.append(jtp)
        
    mdjt = resp.trajectory.multi_dof_joint_trajectory
    mdjt.joint_names =  ['world_joint']
    mdjt.frame_ids = ['odom_combined']
    mdjt.child_frame_ids = ['base_footprint']
    mdjt.header = req.start_state.joint_state.header
    pose = gm.Pose()
    pose.orientation.w = 1
    for _ in xrange(len(resp.trajectory.joint_trajectory.points)): 
        jtp = mm.MultiDOFJointTrajectoryPoint()
        jtp.poses.append(pose)
    #for i in xrange(len(mdjs.joint_names)):
        #if mdjs.joint_names[i] == "world_joint":
            #world_joint_pose = mdjs.poses[i]
            #world_joint_frame_id = mdjs.frame_ids[i]
            #world_joint_child_frame_id = mdjs.child_frame_ids[i]
    #print "world_joint_frame_id", world_joint_frame_id
    #print "world_joint_child_frame_id", world_joint_child_frame_id

    #mdjt = resp.trajectory.multi_dof_joint_trajectory
    #mdjt.header.frame_id = req.start_state.joint_state.header.frame_id
    #mdjt.joint_names.append("world_joint")

    resp.error_code.val = 1
    resp.planning_time = rospy.Duration(planning_duration_seconds)
    resp.trajectory.joint_trajectory.header = req.start_state.joint_state.header
    resp.group_name = req.group_name
    resp.trajectory_start.joint_state = req.start_state.joint_state
    
    #resp.trajectory.multi_dof_joint_trajectory
    #resp.trajectory.multi_dof_joint_trajectory.header = req.start_state.joint_state.header
    
    getresp.motion_plan_response = resp
    return getresp
if __name__ == "__main__":
    rospy.init_node("trajopt_planning_server")

    envfile = "/tmp/%s.xml"%os.path.basename(args.scenefile)
    subprocess.check_call("python scene2xml.py %s %s"%(args.scenefile, envfile), shell=True)
    env = rave.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    env.Load(envfile)
    robot = env.GetRobots()[0]
    
    svc = rospy.Service('plan_kinematic_path', ms.GetMotionPlan, callback)
    rospy.spin()
