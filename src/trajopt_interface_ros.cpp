#include <openrave-core.h>
#include <Eigen/Core>
#include <trajopt/common.hpp>
#include <ipi/sco/optimizers.hpp>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <moveit/kinematic_state/conversions.h>


#include <Eigen/Dense>
#include <trajopt_interface_ros/ros_rave_conversions.h>
#include <trajopt/rave_utils.hpp>
#include <utils/eigen_conversions.hpp>
#include <trajopt/plot_callback.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

using namespace std;
namespace trajopt_interface_ros
{

TrajoptInterfaceROS::TrajoptInterfaceROS(const kinematic_model::KinematicModelConstPtr& kmodel) :
  kmodel(kmodel), nh_("~") 
{
  OpenRAVE::RaveInitialize();
  penv = OpenRAVE::RaveCreateEnvironment();
  penv->StopSimulation();
  // Load the PR2 by default
  penv->Load("robots/pr2-beta-static.zae") ;
  robot = trajopt::GetRobot(*penv);
  bool enableViewer = false;
  nh_.param("enable_viewer", enableViewer, false);
  if(enableViewer){
    viewer.reset(new OSGViewer(penv));
    viewer->UpdateSceneData();
    penv->AddViewer(viewer);
  }
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

  trajopt::ProblemConstructionInfo pci(penv);

  const kinematic_model::JointModelGroup* model_group = 
    planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

  OpenRAVE::RobotBase::ManipulatorPtr manip = getManipulatorFromGroup(robot, model_group);

  int numJoints = model_group->getJointModels().size();
  int numSteps = 10; // TODO: Configurable

  Eigen::VectorXd initialState(numJoints);
  Eigen::VectorXd goalState(numJoints);
  // // LOG_INFO_FMT("Planning for %d joints", numJoints);

  jointStateToArray(planning_scene->getKinematicModel(),
                    req.motion_plan_request.start_state.joint_state, 
                    req.motion_plan_request.group_name,
                    initialState);
  // LOG_INFO("Got initial joint states as array");
  cout << initialState << endl;

  trajopt::RobotAndDOFPtr rad(new trajopt::RobotAndDOF(robot, manip->GetArmIndices(), 0, OpenRAVE::Vector(0,0,1)));
  pci.rad = rad; // trajopt::RADFromName(manip->GetName(), robot);
  pci.rad->SetDOFValues(util::toDblVec(initialState));


  // Sample the goal constraints to get a joint state
  kinematic_state::KinematicStatePtr ksp(new kinematic_state::KinematicState(planning_scene->getCurrentState()));
  constraint_samplers::ConstraintSamplerPtr csp = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(planning_scene, req.motion_plan_request.group_name, req.motion_plan_request.goal_constraints[0]);

  kinematic_state::JointStateGroup *jsg = ksp->getJointStateGroup(req.motion_plan_request.group_name);
  csp->sample(jsg, *ksp);
  vector<double> sampled_vals;
  jsg->getVariableValues(sampled_vals);
  ROS_INFO("Sampled joint values");
  // BOOST_FOREACH(double d, sampled_vals){
  //   ROS_INFO("--- %f\n", d);
  // }
  goalState = util::toVectorXd(sampled_vals);



  setRaveRobotState(robot, req.motion_plan_request.start_state.joint_state);
  // LOG_INFO("Set RAVE Robot State");
  // Handle multi DOF joint start state (base)
  moveit_msgs::MultiDOFJointState multiDofJoints = req.motion_plan_request.start_state.multi_dof_joint_state;
  vector<geometry_msgs::Pose> poses = multiDofJoints.poses;
  vector<string>::iterator mdJoints = multiDofJoints.joint_names.begin();
  vector<geometry_msgs::Pose>::iterator posit = poses.begin();
  vector<string>::iterator frameIds = multiDofJoints.frame_ids.begin();
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

  
  ///////////////////////////////////////////////////////
  ///// Set up planning problem
  ///////////////////////////////////////////////////////
  trajopt::InitInfo initinfo;
  initinfo.type = trajopt::InitInfo::STATIONARY;
  initinfo.data = util::toVectorXd(rad->GetDOFValues()).transpose().replicate(numSteps, 1);

  // Linearly interpolate in joint space
  for(int k=0; k<numJoints; k++){
    initinfo.data.col(k) = Eigen::VectorXd::LinSpaced(numSteps, initialState(k), goalState(k));
  }
  ROS_INFO("Init Traj dimensions: %d x %d", initinfo.data.rows(), initinfo.data.cols());
  
  boost::shared_ptr<trajopt::JointVelCostInfo> jvci(new trajopt::JointVelCostInfo());
  jvci->coeffs = trajopt::DblVec(numJoints, 1); //TODO: Make configurable
  jvci->name = "jvel0";

  boost::shared_ptr<trajopt::CollisionCostInfo> cci(new trajopt::CollisionCostInfo());
  cci->coeffs = trajopt::DblVec(numSteps, 20);
  cci->dist_pen = trajopt::DblVec(numSteps, 0.025);
  cci->name = "collision0";

  boost::shared_ptr<trajopt::JointConstraintInfo> jci(new trajopt::JointConstraintInfo());
  jci->vals = util::toDblVec(goalState);
  jci->timestep = numSteps - 1;
  jci->name = "joint0";

  pci.basic_info.n_steps = numSteps;
  pci.basic_info.manip = manip->GetName();
  pci.basic_info.start_fixed = true;

  pci.init_info = initinfo;
  pci.cost_infos.push_back(jvci);
  pci.cost_infos.push_back(cci);
  pci.cnt_infos.push_back(jci);

  trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(pci);

  ROS_INFO("Problem has %d steps and %d dofs", prob->GetNumSteps(), prob->GetNumDOF());
  ipi::sco::BasicTrustRegionSQP opt(prob);

  // optimize!
  ros::WallTime create_time = ros::WallTime::now();
  // LOG_INFO("Gathered start and goal states");

  // Copy planning scene obstacles into OpenRAVE world
  vector<OpenRAVE::KinBodyPtr> importedBodies = importCollisionWorld(penv, planning_scene->getCollisionWorld());
  // LOG_INFO("Imported collision world");

  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());

  // Why does this not happen in the constructor?
  opt.initialize(trajopt::trajToDblVec(prob->GetInitTraj()));
  ROS_INFO("Gave optimization initial trajectory");
  if(viewer){
    ROS_INFO("Viewer enabled");
    opt.addCallback(trajopt::PlotCallback(*prob));
  }

  opt.optimize();
  ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());

  create_time = ros::WallTime::now();

  // assume that the trajectory is now optimized, fill in the output structure:

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

  // Remove imported bodies from OpenRAVE
  // TODO: Do this somewhere else or do something faster like clone and trash the environment when done?
  BOOST_FOREACH(OpenRAVE::KinBodyPtr body, importedBodies){
    penv->Remove(body);
    ROS_INFO("Removed %s from OpenRAVE", body->GetName().c_str());
    body->Destroy();
    body.reset();
  }
  return true;
}

}
