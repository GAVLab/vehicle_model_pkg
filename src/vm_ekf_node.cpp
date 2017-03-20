/*
 * 
 *
 * Dan Pierce
 * 2017-03-15
 */
#include "vehicle_model_pkg/vm_ekf_node.h"

VehicleModelEkfNode::VehicleModelEkfNode()
{
	ros::NodeHandle nh("~");

	std::string drive_axle;
	nh.param("sprung_mass", mass, 1528.2);
	nh.param("wheel_radius", wheel_radius, 0.3255);
	nh.param("drive_axle", drive_axle, std::string("front"));
	nh.param("steering_wheel_gear_ratio", Nsw, 15.9);
	nh.param("front_axle_to_cg", a, 1.368);
	nh.param("rear_axle_to_cg", b, 1.482);
  nh.param("time_step", dt, 0.05);
  
  if (~drive_axle.compare("front")){drive_type=0;}else{drive_type=1;}
  

  //////////////
  /* EKF STUFF*/
  //////////////
  EkfParameters prm;
  prm.pos_i[0] = 0.0; prm.pos_i[1] = 0.0; prm.psi_i = 0.0;

  // ----- Process Uncertainty
  nh.param("std_steer_angle", ekf.prm.std_del, 0.05);
  nh.param("std_velocity", prm.std_vel, 0.05);
  // ----- Measurement Uncertainty
  nh.param("std_pos", prm.std_pos, 0.1);
  // ----- Initial Uncertainty
  nh.param("std_initial_position", prm.std_pos_i, 0.05);
  nh.param("std_initial_heading", prm.std_psi_i, 0.05);

  ekf.init(prm);


	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_node/steer_angle", 0, &VehicleModelEkfNode::steerAngleCallback, this);
	ws_sub = nh.subscribe<g35can::g35can_wheel_speed>("/g35can_node/wheel_speeds", 0, &VehicleModelEkfNode::wheelSpeedCallback, this);
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);


}

VehicleModelEkfNode::~VehicleModelEkfNode()
{
  std::cout << "VehicleModelEkfNode::~VehicleModelEkfNode" << std::endl;
}

void VehicleModelEkfNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  ros::Time stamp_ = msg->header.stamp;

  double del = ( msg->steer_angle/Nsw )*M_PI/180;
  // propagate( del );

  // publishLatestState(stamp_);
  
  return;
}

void VehicleModelEkfNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
	ros::Time stamp_ = msg->header.stamp;
	
	calculateVehicleSpeed(msg->wheel_speed_left_front,msg->wheel_speed_right_front,msg->wheel_speed_left_rear,msg->wheel_speed_right_rear);

	return;
}

void VehicleModelEkfNode::publishLatestState(ros::Time stamp_){

  // -------- ROS Publish Odometry
  // nav_msgs::Odometry odom_msg_;
  // odom_msg_.header.stamp = stamp_;
  // odom_msg_.header.frame_id = odom_frame_id;
  // odom_msg_.child_frame_id = base_link_frame_id;
  
  // odom_msg_.pose.pose.position.x = pos[0];
  // odom_msg_.pose.pose.position.y = pos[1];

  // geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw );

  // odom_msg_.pose.pose.orientation = q;

  // odom_msg_.twist.twist.linear.x = speed;
  // odom_msg_.twist.twist.angular.z = omega;

  // odom_pub.publish(odom_msg_);

  // // -------- ROS TF Odometry
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = stamp_;
  // odom_trans.header.frame_id = odom_frame_id;
  // odom_trans.child_frame_id = base_link_frame_id;

  // // to 
  // odom_trans.transform.translation.x = pos[0];
  // odom_trans.transform.translation.y = pos[1];
  // odom_trans.transform.translation.z = 0.0;

  // odom_trans.transform.rotation = q;

  // tf_broadcaster.sendTransform(odom_trans);

  return;
}

void VehicleModelEkfNode::calculateVehicleSpeed(double ws_lf,double ws_rf,double ws_lr,double ws_rr){
  
  switch(drive_type){
    case 0: // front wheel drive, calculate speed with rear wheels
      speed = (ws_lr+ws_rr)*wheel_radius/2.0;
    case 1: // rear wheel drive, calculate speed with front wheels
      speed = (ws_lf+ws_rf)*wheel_radius/2.0;
    case 2:	// all wheel drive, calculate speed with all wheels
      speed = (ws_lf+ws_rf+ws_lr+ws_rr)*wheel_radius/4.0;
  }

  return;
}

void VehicleModelEkfNode::wrapToPi(double &ang){
    ang = fmod(ang+M_PI,2*M_PI);
    if (ang < 0)
        ang += 2*M_PI;
    ang -= M_PI;
    return;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc,argv,"vehicle_model_ekf");
  VehicleModelEkfNode node;
  ros::spin();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////