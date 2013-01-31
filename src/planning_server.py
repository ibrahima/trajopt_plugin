#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile")
args = parser.parse_args()

import sys
sys.path.append("/home/joschu/ros/moveit/devel/lib/python2.7/dist-packages")
sys.path.append("/home/ibrahima/moveit/devel/lib/python2.7/dist-packages")


import subprocess, os
import trajoptpy
import rospy
import json
import moveit_msgs.msg as mm
import openravepy as rave, numpy as np
import moveit_msgs.srv as ms
import trajoptpy.math_utils as mu
import trajectory_msgs.msg as tm

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
    print "yoyoyo"
    assert isinstance(getreq, ms.GetMotionPlanRequest)
    req = getreq.motion_plan_request   
    getresp = ms.GetMotionPlanResponse()
    resp = getresp.motion_plan_response
    
    manip = GROUPMAP[req.group_name]
    
    update_rave_from_ros(robot, req.start_state.joint_state.position, req.start_state.joint_state.name)
    
    start_joints = robot.GetDOFValues(robot.GetManipulator(manip).GetArmIndices())
    
    n_steps = 10
    
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
            "params" : {"coeffs" : [10],"dist_pen" : [0.01]}
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
            joint_vals = [jc.position for jc in goal_cnt.joint_constraints]

            d["constraints"].append({
                "type" : "joint",
                "name" : "joint", 
                "params" : {
                    "vals" : joint_vals
                }                
            })
            
            d["init_info"]["data"] = [row.tolist() for row in mu.linspace2d(start_joints, joint_vals, n_steps)]

            
        if len(goal_cnt.position_constraints) > 0:
            raise NotImplementedError
            
    
    trajoptpy.SetInteractive(False)
    s = json.dumps(d)
    try:    
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
    except Exception:
        return None

    inds = robot.GetManipulator(manip).GetArmJoints()
    alljoints = robot.GetJoints()
    names = [alljoints[i].GetName() for i in inds]
    resp.trajectory.joint_trajectory.joint_names = names
    for row in result.GetTraj():
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = row.tolist()
        jtp.time_from_start = rospy.Duration(0)
        resp.trajectory.joint_trajectory.points.append(jtp)
        
    resp.error_code.val = 1
    resp.planning_time = rospy.Duration(0)
    resp.trajectory.joint_trajectory.header = req.start_state.joint_state.header
    resp.group_name = req.group_name
    resp.trajectory_start.joint_state = req.start_state.joint_state
    resp.trajectory.multi_dof_joint_trajectory.header = req.start_state.joint_state.header
    print "done"
    return getresp
if __name__ == "__main__":
    rospy.init_node("planning_server")

    envfile = "/tmp/%s.xml"%os.path.basename(args.scenefile)
    subprocess.check_call("python scene2xml.py %s %s"%(args.scenefile, envfile), shell=True)
    env = rave.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    env.Load(envfile)
    robot = env.GetRobots()[0]
    
    rospy.Service('trajopt_planner', ms.GetMotionPlan, callback)
    rospy.spin()
