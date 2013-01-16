#include <trajopt_interface_ros/ros_rave_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <boost/shared_ptr.hpp>
#include <assert.h>
#include <string>

using namespace std;
OpenRAVE::KinBodyPtr moveitObjectToKinBody(collision_detection::CollisionWorld::ObjectConstPtr object, OpenRAVE::EnvironmentBasePtr env){
  vector<shapes::ShapeConstPtr> shapes = object->shapes_;
  EigenSTL::vector_Affine3d poses = object->shape_poses_;
  list<OpenRAVE::KinBody::Link::GeometryInfo> geometries;
  for(int i = 0; i < shapes.size(); i++){
	shapes::ShapeConstPtr shape = shapes[i];
    OpenRAVE::KinBody::Link::GeometryInfo info;
    // Convert the Eigen::Affine3D pose into a RaveTransform (translation + quaternion)
    Eigen::Vector3d translation = poses[i].translation();
    //LOG_WARN("Translation " << translation );
    Eigen::Matrix3d rot = poses[i].rotation();
    //LOG_WARN("Rotation " << rot );
    info._t.trans = OpenRAVE::RaveVector<double>(translation[0], translation[1], translation[2]);
    OpenRAVE::geometry::RaveTransformMatrix<double> rotm;
    rotm.rotfrommat(rot(0,0), rot(0,1), rot(0,2),
		    rot(1,0), rot(1,1), rot(1,2),
		    rot(2,0), rot(2,1), rot(2,2));
    info._t.rot = OpenRAVE::geometry::quatFromMatrix(rotm);
    // Fill in the geometry data (_vGeomData)
    switch(shape->type){
    case shapes::SPHERE: {//for sphere it is radius
      //LOG_DEBUG("Importing a sphere");
      info._type = OpenRAVE::KinBody::Link::GeomSphere;
	  boost::shared_ptr<const shapes::Sphere> sph = boost::dynamic_pointer_cast<const shapes::Sphere>(shape);
      info._vGeomData.x = sph->radius;
      break;
	}
    case shapes::CYLINDER: {//for cylinder, first 2 values are radius and height
      //LOG_DEBUG("Importing a cylinder");
      boost::shared_ptr<const shapes::Cylinder> cyl = boost::dynamic_pointer_cast<const shapes::Cylinder>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomCylinder;
      info._vGeomData.x = cyl->radius;
      info._vGeomData.y = cyl->length;
      break;
	}
    case shapes::BOX: { //for boxes, first 3 values are extents
      // LOG_DEBUG("Importing a box");
      boost::shared_ptr<const shapes::Box> box = boost::dynamic_pointer_cast<const shapes::Box>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomBox;
      // OpenRAVE uses half extents
      info._vGeomData.x = box->size[0]/2.;
      info._vGeomData.y = box->size[1]/2.;
      info._vGeomData.z = box->size[2]/2.;
      break;
	}
    case shapes::MESH: {
      // LOG_WARN("Importing a mesh");
      // Not actually sure if mesh and trimesh are the same thing.
      // Hopefully don't have to triangulate non-triangular meshes
      boost::shared_ptr<const shapes::Sphere> sph = boost::dynamic_pointer_cast<const shapes::Sphere>(shape);
      info._type = OpenRAVE::KinBody::Link::GeomTrimesh;
      continue; // TODO: Implement this and remove
      break;
	}
    default:
      // LOG_WARN("Unknown shape type");
      continue;
    }
    geometries.push_back(info);
  }
  //TODO: Set a name for the KinBody
  OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env);
  body->InitFromGeometries(geometries);
  // Perhaps it should do something with the return code, if it's false?
  return body;
}

void importCollisionWorld(OpenRAVE::EnvironmentBasePtr env, const collision_detection::CollisionWorldConstPtr world){
  std::vector<string> objectIds = world->getObjectIds();
  // LOG_DEBUG("Importing ROS collision world");
  // LOG_DEBUG_FMT("World contains %d objects", world->getObjectsCount());
  for(int i = 0; i < objectIds.size(); i++){
    // LOG_DEBUG_FMT("Importing world object %d of %d", i+1, objectIds.size());
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(objectIds[i]);
	OpenRAVE::KinBodyPtr body = moveitObjectToKinBody(obj, env);
    // I think this is necessary, not 100% sure
    env->Add(body);

  }
}

bool setRaveRobotState(OpenRAVE::RobotBasePtr robot, sensor_msgs::JointState js){
  vector<string>::iterator nameit = js.name.begin();
  vector<double>::iterator posit = js.position.begin();

  vector<int> dofs;
  vector<double> positions;
  bool foundAllJoints = true;
  while(nameit != js.name.end()){
	// Not sure if different types of joints need to be handled separately
	OpenRAVE::KinBody::JointPtr joint = robot->GetJoint(*nameit);
	if(joint){
      if(joint->GetDOFIndex()>0){
        dofs.push_back(joint->GetDOFIndex());
        positions.push_back(*posit);
      }
	}else{
      // LOG_INFO_FMT("Could not find DOF Index for joint %s", nameit->c_str());
	  foundAllJoints = false;
	}

    nameit++;
    posit++;
  }
  // LOG_INFO_FMT("Finished iterating through joints, setting %d DOFs corresponding to %d joints from ROS", dofs.size(), js.position.size());

  try{
    robot->SetDOFValues(positions, OpenRAVE::KinBody::CLA_CheckLimits, dofs);
  }
  catch(const OpenRAVE::openrave_exception& ex) {
    RAVELOG_WARN("exception caught: %s\n",ex.what());
  }// catch(const out_of_range& oor){
  //   cerr << "Out of Range error: " << oor.what() << endl;
  // }
  // catch(const exception& ex){
  //   LOG_ERROR_FMT("Unknown exception: %s\n", ex.what());
  // }
  // catch(...){
  //   LOG_WARN("Really unknown exception");
  // }
  // LOG_INFO("Set OpenRAVE joint states from ROS");
  return foundAllJoints;
}

OpenRAVE::RobotBase::ManipulatorPtr getManipulatorFromGroup(const OpenRAVE::RobotBasePtr robot, const kinematic_model::JointModelGroup* model_group){
  std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manipulators = robot->GetManipulators();
  // LOG_INFO_FMT("Found %d manipulators", manipulators.size());
  // LOG_INFO_FMT("Looking for %s", model_group->getName().c_str());
  std::vector<std::string>::const_iterator nit = model_group->getJointModelNames().begin();
  while(nit++ != model_group->getJointModelNames().end()){
    // LOG_INFO_FMT("  Joint: %s", nit->c_str());
  }

  std::string name = model_group->getName();
  // This is an ugly hack, just remove all underscores and hope it matches an OpenRAVE manipulator
  size_t underscore_pos = string::npos;
  while((underscore_pos = name.find('_')) != string::npos){
    name.erase(underscore_pos, 1);
  }
  // LOG_INFO_FMT("Cleaned name: %s", name.c_str());
  // Don't the next few lines just make you love C++?
  std::vector<OpenRAVE::RobotBase::ManipulatorPtr>::iterator iter = manipulators.begin();
  while(iter != manipulators.end()){
    // LOG_INFO_FMT("OpenRAVE has %s", (*iter)->GetName().c_str());
    if((*iter)->GetName() == name){
      // LOG_INFO("Found the corresponding OpenRAVE manipulator");
      return (*iter);
    }
    iter++;
  }
  return manipulators[0];
}
