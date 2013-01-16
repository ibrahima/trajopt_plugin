#include <trajopt/common.hpp>
#include <ipi/sco/optimizers.hpp>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <moveit/kinematic_state/conversions.h>
#include <Eigen/Dense>
#include <trajopt_interface_ros/ros_rave_conversions.h>
#include <trajopt/rave_utils.hpp>
#include <utils/eigen_conversions.hpp>

#include <iostream>

namespace trajopt_interface_ros
{

TrajoptInterfaceROS::TrajoptInterfaceROS(const kinematic_model::KinematicModelConstPtr& kmodel) :
  kmodel(kmodel), nh_("~") 
{
  penv = OpenRAVE::RaveCreateEnvironment();
  penv->StopSimulation();
  // Load the PR2 by default
  penv->Load("robots/pr2-beta-static.zae") ;
  robot = trajopt::GetRobot(*penv);

  viewer.reset(new OSGViewer(penv));
  viewer->UpdateSceneData();
  penv->AddViewer(viewer);

  loadParams();
}

TrajoptInterfaceROS::~TrajoptInterfaceROS()
{
  viewer.reset();
  penv.reset();
  OpenRAVE::RaveDestroy();
}
void TrajoptInterfaceROS::loadParams(void) {
  
}

bool TrajoptInterfaceROS::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::GetMotionPlan::Request &req, 
                         moveit_msgs::GetMotionPlan::Response &res) const
{


  ros::WallTime start_time = ros::WallTime::now();

  Eigen::MatrixXd initTraj;

  trajopt::ProblemConstructionInfo pci(penv);

  trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(pci);
  
  ipi::sco::BasicTrustRegionSQP opt(prob);

  const kinematic_model::JointModelGroup* model_group = 
    planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

  int numJoints = model_group->getJointModels().size();
  Eigen::VectorXd initialState(numJoints);
  Eigen::VectorXd goalState(numJoints);
  // // LOG_INFO_FMT("Planning for %d joints", numJoints);

  jointStateToArray(planning_scene->getKinematicModel(),
                    req.motion_plan_request.start_state.joint_state, 
                    req.motion_plan_request.group_name,
                    initialState);
  // LOG_INFO("Got initial joint states as array");
  std::cout << initialState << std::endl;

  pci.rad->SetDOFValues(util::toDblVec(initialState));

  sensor_msgs::JointState js;
  
  // Gathers the goal joint constraints into a JointState object
  // TODO: Handle all constraints, not just first joint constraints
  // TODO: Refactor, this is ugly
  for(unsigned int i = 0; i < req.motion_plan_request.goal_constraints[0].joint_constraints.size(); i++) {
    js.name.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name);
    js.position.push_back(req.motion_plan_request.goal_constraints[0].joint_constraints[i].position);
  }

  // LOG_INFO("Gathered goal joint state");


  // Note: May need to check req.mpr.start_state.multi_dof_joint_state for base transform and others
  // TODO: This function is broken
  setRaveRobotState(robot, req.motion_plan_request.start_state.joint_state);
  // LOG_INFO("Set RAVE Robot State");
  // Get the goal state
  jointStateToArray(planning_scene->getKinematicModel(),
                    js, 
                    req.motion_plan_request.group_name, 
                    goalState);
  // LOG_INFO ("Got Goal state");
  std::cout << goalState << std::endl;

  // Handle multi DOF joint start state (base)
  moveit_msgs::MultiDOFJointState multiDofJoints = req.motion_plan_request.start_state.multi_dof_joint_state;
  std::vector<geometry_msgs::Pose> poses = multiDofJoints.poses;
  std::vector<std::string>::iterator mdJoints = multiDofJoints.joint_names.begin();
  std::vector<geometry_msgs::Pose>::iterator posit = poses.begin();
  std::vector<std::string>::iterator frameIds = multiDofJoints.frame_ids.begin();
  while(mdJoints != multiDofJoints.joint_names.end()){
    // LOG_DEBUG("Multi DOF Joint: " << *mdJoints << " in frame " << *frameIds);
    if(*mdJoints == "world_joint"){ // world_joint represents the offset from odom_combined to base_footprint
      OpenRAVE::RaveVector<double> trans(posit->position.x, posit->position.y, posit->position.z);
      OpenRAVE::RaveVector<double> rot(posit->orientation.w, posit->orientation.x, posit->orientation.y, posit->orientation.z);
      OpenRAVE::Transform t(rot, trans);
      robot->SetTransform(t);
    }
    posit++;
    mdJoints++;
    frameIds++;
  }
  // optimize!
  kinematic_state::KinematicState start_state(planning_scene->getCurrentState());
  kinematic_state::robotStateToKinematicState(*planning_scene->getTransforms(), req.motion_plan_request.start_state, start_state);
  ros::WallTime create_time = ros::WallTime::now();
  // LOG_INFO("Gathered start and goal states");

  // Create OpenRAVE world
  // Got a rave and a bullet instance
  // Copy planning scene obstacles into OpenRAVE world
  importCollisionWorld(penv, planning_scene->getCollisionWorld());
  // LOG_INFO("Imported collision world");

  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());  

  OpenRAVE::RobotBase::ManipulatorPtr manip = getManipulatorFromGroup(robot, model_group);
  
  //setupArmToJointTarget(opt, goalState, rro->createManipulator(manip->GetName(), false));
  //setEndFixed(opt);
  //trajOuterOpt(opt, AllowedCollisions());
  ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());

  create_time = ros::WallTime::now();

  // assume that the trajectory is now optimized, fill in the output structure:
  // LOG_WARN("Final Trajectory");

  trajopt::TrajArray finalTraj = trajopt::getTraj(opt.x(), prob->GetVars());
  // fill in joint names:
  // TODO: May need to make sure that joint names are in the same order
  res.trajectory.joint_trajectory.joint_names.resize(numJoints);
  for (size_t i = 0; i < model_group->getJointModels().size(); i++){
    res.trajectory.joint_trajectory.joint_names[i] = model_group->getJointModels()[i]->getName();
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(prob->GetNumSteps());
  for (int i=0; i < prob->GetNumSteps(); i++){
    // TODO: Trajectory may include base or other DOFs
    res.trajectory.joint_trajectory.points[i].positions.resize(numJoints);
    for (size_t j=0; j < res.trajectory.joint_trajectory.points[i].positions.size(); j++){
      res.trajectory.joint_trajectory.points[i].positions[j] = finalTraj(i,j);
      if(i == finalTraj.rows()-1) {
        ROS_INFO_STREAM("Joint " << j << " " << res.trajectory.joint_trajectory.points[i].positions[j]);
      }
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  // Fill in multi DOF trajectory (base)
  //res.trajectory.multi_dof_joint_trajectory;
  
  ROS_INFO("Response took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[finalTraj.rows()].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time = ros::Duration((ros::WallTime::now() - start_time).toSec());
  return true;
}

}
