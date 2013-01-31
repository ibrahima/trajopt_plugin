#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <trajopt_interface_ros/trajopt_interface_ros.h>

#include <boost/shared_ptr.hpp>

#include <pluginlib/class_list_macros.h>

namespace trajopt_interface_ros
{

class TrajoptPlanner : public planning_interface::Planner
{
public:
  void init(const kinematic_model::KinematicModelConstPtr& model)
  {
    trajopt_interface_.reset(new TrajoptInterfaceROS(model));
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
  {
    // TODO: Actually respond with something reasonable
    //      capabilities.dummy = false;
    return true;
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::MotionPlanRequest &req, 
             moveit_msgs::MotionPlanResponse &res) const
  {
    return trajopt_interface_->solve(planning_scene, req, res);
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::MotionPlanRequest &req,
             moveit_msgs::MotionPlanDetailedResponse &res) const
  {
    moveit_msgs::MotionPlanResponse res2;

    if (trajopt_interface_->solve(planning_scene, req,res2))
    {
      res.trajectory_start = res2.trajectory_start;
      res.trajectory.push_back(res2.trajectory);
      res.description.push_back("plan");
      res.processing_time.push_back(res2.planning_time);
      return true;
    }
    else
      return false;
  }

  std::string getDescription(void) const { return "trajopt"; }
  
  void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.resize(1);
    algs[0] = "trajopt";
  }

  void terminate(void) const
  {
    //TODO - make interruptible
  }
     
private:
  boost::shared_ptr<TrajoptInterfaceROS> trajopt_interface_;
};

} // trajopt_interface_ros

PLUGINLIB_EXPORT_CLASS( trajopt_interface_ros::TrajoptPlanner, 
                        planning_interface::Planner);
