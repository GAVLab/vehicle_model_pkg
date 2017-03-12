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

	std::string drive_axle;
	nh.param("sprung_mass", mass, 1528.2);
	nh.param("wheel_radius", wheel_radius, 0.3255);
	nh.param("drive_axle", drive_axle, std::string("front"));

	nh.param("time_step", dt, 0.05);

	if (~drive_axle.compare("front")){drive_type=0;}else{drive_type=1;}
	
	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_node/steer_angle", 0, &KinematicModelNode::steerAngleCallback, this);
	ws_sub = nh.subscribe<g35can::g35can_wheel_speed>("/g35can_node/wheel_speeds", 0, &KinematicModelNode::wheelSpeedCallback, this);
}

KinematicModelNode::~KinematicModelNode()
{
  std::cout << "KinematicModelNode::~KinematicModelNode" << std::endl;
}

void KinematicModelNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  ros::Time stamp_ = msg->header.stamp;
  // KinematicModelNode::propagate( del );
  return;
}

void KinematicModelNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
	ros::Time stamp_ = msg->header.stamp;
	std::cout << drive_type << std::endl;

	KinematicModelNode::calculateVehicleSpeed(msg->wheel_speed_left_front,msg->wheel_speed_right_front,msg->wheel_speed_left_rear,msg->wheel_speed_right_rear);

	return;
}


void KinematicModelNode::calculateVehicleSpeed(double ws_lf,double ws_rf,double ws_lr,double ws_rr){
  
  switch(drive_type){
    case 0: // front wheel drive, calculate speed with rear wheels
      speed = (ws_lr+ws_rr)*wheel_radius/2.0;
    case 1: // rear wheel drive, calculate speed with front wheels
      speed = (ws_lf+ws_rf)*wheel_radius/2.0;
    case 2:	// all wheel drive, calculate speed with all wheels
      speed = (ws_lf+ws_rf+ws_lr+ws_rr)*wheel_radius/4.0;
  }

  std::cout << speed << std::endl;

  return;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc,argv,"kinematic_model_node");
  KinematicModelNode node;
  ros::spin();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////