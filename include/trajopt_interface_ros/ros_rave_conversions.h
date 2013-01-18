#include <moveit/collision_detection/collision_world.h>
#include <openrave/openrave.h>

OpenRAVE::KinBodyPtr moveitObjectToKinBody(collision_detection::CollisionWorld::ObjectConstPtr object, OpenRAVE::EnvironmentBasePtr env);

std::vector<OpenRAVE::KinBodyPtr> importCollisionWorld(OpenRAVE::EnvironmentBasePtr env, const collision_detection::CollisionWorldConstPtr world);

bool setRaveRobotState(OpenRAVE::RobotBasePtr robot, sensor_msgs::JointState js);

OpenRAVE::RobotBase::ManipulatorPtr getManipulatorFromGroup(const OpenRAVE::RobotBasePtr robot, const kinematic_model::JointModelGroup* model_group);
