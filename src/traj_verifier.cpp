#include <openrave-core.h>
#include <trajopt/common.hpp>
#include <trajopt/collision_checker.hpp>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <trajopt_interface_ros/ros_rave_conversions.h>
#include <trajopt/rave_utils.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <json/json.h>
#include <trajopt_plugin/String2String.h>
#include <trajopt/json_marshal.hpp>
#include <utils/eigen_conversions.hpp>
#include <utils/interpolation.hpp>
#include <utils/stl_to_string.hpp>
#include <boost/foreach.hpp>
#include <trajopt/plot_callback.hpp>
using namespace planning_scene_monitor;
using namespace trajopt_plugin;
using namespace std;
using namespace trajopt;
using namespace OpenRAVE;
using namespace util;

#define ENSURE(expr) if (!expr) throw std::runtime_error("failure: "#expr);

void PlotTraj(OSGViewer& viewer, RobotAndDOF& rad, const TrajArray& x, vector<GraphHandlePtr>& handles) {
  RobotBase::RobotStateSaver saver = rad.Save();
  for (int i=0; i < x.rows(); ++i) {
    rad.SetDOFValues(toDblVec(x.row(i)));
    handles.push_back(viewer.PlotKinBody(rad.GetRobot()));
    SetTransparency(handles.back(), .35);
  }
}

class TrajVerifier {
  PlanningSceneMonitor m_psm;
  EnvironmentBasePtr m_robot_env, m_full_env;
  RobotBasePtr m_full_robot;
public:
  TrajVerifier() : m_psm("robot_description") {
    m_psm.startSceneMonitor();
    m_robot_env = RaveCreateEnvironment();
    m_robot_env->StopSimulation();
    m_robot_env->Load("robots/pr2-beta-static.zae");
  }
  bool Callback(String2String::Request& req, String2String::Response& resp);
  bool CheckSafetyWithRave(const TrajArray& traj, RobotAndDOFPtr rad);
  bool CheckSafetyWithTrajopt(const TrajArray& traj, RobotAndDOFPtr rad);
};

Json::Value readJsonFile(const std::string& doc) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(doc, root);
  if (!success) throw std::runtime_error("couldn't parse string as json");
  return root;
}

RobotAndDOFPtr RADFromName(const string& name, RobotBasePtr robot) {
  if (name == "active") {
    return RobotAndDOFPtr(new RobotAndDOF(robot, robot->GetActiveDOFIndices(), robot->GetAffineDOF(), robot->GetAffineRotationAxis()));
  }
  vector<int> dof_inds;
  int affinedofs = 0;
  Vector rotationaxis(0,0,1);
  vector<string> components;
  boost::split(components, name, boost::is_any_of("+"));
  for (int i=0; i < components.size(); ++i) {
    std::string& component = components[i];
    if (RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, component)) {
      vector<int> inds = manip->GetArmIndices();
      dof_inds.insert(dof_inds.end(), inds.begin(), inds.end());
    }
    else if (component == "base") {
      affinedofs |= DOF_X | DOF_Y | DOF_RotationAxis;
    }
    else if (KinBody::JointPtr joint = robot->GetJoint(component)) {
      dof_inds.push_back(joint->GetDOFIndex());
    }
    else throw MY_EXCEPTION( (boost::format("error in reading manip description: %s must be a manipulator, link, or 'base'")%component).str() );
  }
  return RobotAndDOFPtr(new RobotAndDOF(robot, dof_inds, affinedofs, rotationaxis));
}

bool TrajVerifier::Callback(String2String::Request& req, String2String::Response& resp) {

  TrajArray traj;
  Json::Value root = readJsonFile(req.s);


  if (root.isConvertibleTo(Json::stringValue) && root.asString() == "update") {
    planning_scene::PlanningSceneConstPtr ps = m_psm.getPlanningScene();
    if (m_full_env) m_full_env->Destroy();
    m_full_env = RaveCreateEnvironment();
    m_full_env->Load("robots/pr2-beta-static.zae");
    m_full_env->StopSimulation();
    m_full_env->GetCollisionChecker()->InitEnvironment();
    m_full_robot = m_full_env->GetRobot("pr2");
    importCollisionWorld(m_full_env, ps->getCollisionWorld());
    return true;
  }

  cout << "json request: " << root << endl;
  int n_steps, n_dof;
  childFromJson(root, n_steps, "n_steps");
  childFromJson(root, n_dof, "n_dof");
  string manip_str;
  childFromJson(root, manip_str, "manip");
  Json::Value& json_traj = root["traj"];
  if (json_traj.size() != n_steps) {
    ROS_ERROR("given initialization traj has wrong length");
    return false;
  }
  traj.resize(n_steps, n_dof);
  for (int i=0; i < n_steps; ++i) {
    DblVec row;
    fromJsonArray(json_traj[i], row, n_dof);
    traj.row(i) = toVectorXd(row);
  }

  vector<KinBodyPtr> bodies;  m_full_env->GetBodies(bodies);
  cout << "Bodies: " << endl;
  BOOST_FOREACH(KinBodyPtr body, bodies) {
    cout << body->GetName() << endl;
  }

  RobotAndDOFPtr rad = RADFromName(manip_str, m_full_robot);

  Json::Value json_resp;
  json_resp["rave_safe"] = CheckSafetyWithRave(traj, rad);
  json_resp["trajopt_safe"] = CheckSafetyWithTrajopt(traj, rad);

  resp.s = json_resp.toStyledString();
  return true;
}

bool TrajVerifier::CheckSafetyWithRave(const TrajArray& traj, RobotAndDOFPtr rad) {
  bool safe = true;
  int nUp = 20;
  EnvironmentBasePtr env = rad->GetRobot()->GetEnv();
  TrajArray trajup = interp2d(VectorXd::LinSpaced(nUp, 0, 1), VectorXd::LinSpaced(traj.rows(), 0, 1), traj);
  for (int i=0; i < trajup.rows(); ++i) {
    rad->SetDOFValues(toDblVec(trajup.row(i)));
    CollisionReportPtr report(new CollisionReport());
    bool collision_happened = env->CheckCollision(env->GetKinBody("pr2"), env->GetKinBody("tabletop"), report);
    RAVELOG_INFO("%i collisions at time %i\n", report->numCols, i);
    safe = safe && !collision_happened;
  }
  return safe;
}

bool TrajVerifier::CheckSafetyWithTrajopt(const TrajArray& traj, RobotAndDOFPtr rad) {
  bool safe = true;
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(rad->GetRobot()->GetEnv());
  EnvironmentBasePtr env = rad->GetRobot()->GetEnv();
  vector<GraphHandlePtr> handles;
  PlotTraj(*viewer, *rad, traj, handles);
  if (!env->GetViewer("osg")) env->AddViewer(viewer);
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*env);
  vector<trajopt::Collision> collisions;
  for (int i=0; i < traj.rows(); ++i) {
    rad->SetDOFValues(toDblVec(traj.row(i)));
    cc->AllVsAll(collisions);
  }
  PlotCollisions(collisions, *env, handles, 0);
//  cc->ContinuousCheckTrajectory(traj, *rad, collisions);
  vector<Collision> badcollisions;
  BOOST_FOREACH(const Collision& c, collisions) {
    if (c.distance < 0) badcollisions.push_back(c);
  }
  stringstream ss; ss << Str(badcollisions) << endl;
  RAVELOG_INFO(ss.str().c_str());

  viewer->Idle();
  return badcollisions.size() == 0;
}

int main(int argc, char* argv[]) {
  RaveInitialize(true);
  ros::init(argc, argv, "traj_verifier_server");

  ros::NodeHandle nh;
  TrajVerifier tv;
  ros::ServiceServer ss = nh.advertiseService("traj_verifier", &TrajVerifier::Callback, &tv);
  ros::spin();
  return 0;
}
