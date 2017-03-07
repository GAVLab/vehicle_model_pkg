/*
 * ROS node wrapping the kinematic vehicle model class
 *
 * Dan Pierce
 * 2017-03-07
 */
#include "vehicle_model_pkg/kinematic_model_node.h"

KinematicModelNode::KinematicModelNode()
{
	ros::NodeHandle nh("~");
	
	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_steer_angle", 0, &KinematicModelNode::steerAngleCallback, this);
}

KinematicModelNode::~KinematicModelNode()
{
  std::cout << "KinematicModelNode::~KinematicModelNode" << std::endl;
}

void KinematicModelNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& steer_msg){
  ros::Time stamp_ = steer_msg->header.stamp;

  return;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc,argv,"kinematic_model_node");
  // KinematicModelNode node;
  ros::spin();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////