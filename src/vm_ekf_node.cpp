/*
 * 
 *
 * Dan Pierce
 * 2017-03-15
 */
#include "vehicle_model_pkg/vm_ekf_node.h"

VehicleModelEkfNode::VehicleModelEkfNode()
{

  ros::NodeHandle nh;

  init = false;

  ////////////////////////
  /* Vehicle Parameters */
  ////////////////////////
	std::string drive_axle;
  double a,b;
	nh.param("wheel_radius", wheel_radius, 0.3255);
	nh.param("drive_axle", drive_axle, std::string("front"));
	nh.param("steering_wheel_gear_ratio", Nsw, 15.9);
	nh.param("front_axle_to_cg", a, 1.368);
	nh.param("rear_axle_to_cg", b, 1.482);
  
  if (~drive_axle.compare("front")){drive_type=0;}else{drive_type=1;}
  
  ////////////////////
  /* EKF Parameters */
  ////////////////////
  ekf.vehicle.a = a; ekf.vehicle.b = b;
  
  // (!!!!!!!)
  ekf.prm.pos_i[0] = 2.789630919503271; 
  ekf.prm.pos_i[1] = 113.8500361211426; 
  ekf.prm.psi_i = 1.566862634015955;
  // (!!!!!!!)

  // ----- Time Step
  nh.param("time_step", ekf.prm.dt, 0.01);
  // ----- Process Uncertainty
  nh.param("std_steer_angle", ekf.prm.std_del, 0.002);
  nh.param("std_velocity", ekf.prm.std_vel, 0.03);
  // ----- Measurement Uncertainty
  nh.param("std_pos", ekf.prm.std_pos, 0.1);
  // ----- Initial Uncertainty
  nh.param("std_initial_position", ekf.prm.std_pos_i, 0.1);
  nh.param("std_initial_heading", ekf.prm.std_psi_i, 0.1);

  ekf.init();

  /////////////////////////
  /* ROS Service Clients */
  /////////////////////////
  enu_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu");
  lla_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2lla");

  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_node/steer_angle", 0, &VehicleModelEkfNode::steerAngleCallback, this);
	ws_sub = nh.subscribe<g35can::g35can_wheel_speed>("/g35can_node/wheel_speeds", 0, &VehicleModelEkfNode::wheelSpeedCallback, this);
	gps_sub = nh.subscribe<nav_msgs::Odometry>("/septentrio/odom", 0, &VehicleModelEkfNode::gpsCallback, this);

	odom_pub = nh.advertise<nav_msgs::Odometry>("vm_ekf_node/odom", 10);

}

VehicleModelEkfNode::~VehicleModelEkfNode()
{
  std::cout << "VehicleModelEkfNode::~VehicleModelEkfNode" << std::endl;
}

void VehicleModelEkfNode::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
  ros::Time stamp_ = msg->header.stamp;
  
  double gps_pos[2];

  wgs_conversions::WgsConversion srv;

  srv.request.xyz[0] = msg->pose.pose.position.x;
  srv.request.xyz[1] = msg->pose.pose.position.y;
  srv.request.xyz[2] = msg->pose.pose.position.z;

  if (!init){
    // Convert from XYZ to LLA
    if(lla_client.call(srv)){
      ref_lla[0] = srv.response.lla[0];
      ref_lla[1] = srv.response.lla[1];
      ref_lla[2] = srv.response.lla[2];
      std::cout << "\nReference LLA set:" << std::endl;
      std::cout << "\tLat: " << ref_lla[0] << "\tLong: " << ref_lla[1] << "\tAlt: " << ref_lla[2] << std::endl;
      init=true;
    }else{
      ROS_ERROR("Failed to call service xyz2lla");
    }
    return;
  }

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  if(enu_client.call(srv)){
    gps_pos[1] = srv.response.enu[0]; // gps_pos is in NED convention
    gps_pos[0] = srv.response.enu[1];
  }else{
    ROS_ERROR("Failed to call service xyz2enu");
    return;
  }

  // ekf.measurementUpdate( gps_pos );
  
  

  return;
}

void VehicleModelEkfNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  ros::Time stamp_ = msg->header.stamp;

  double del = ( msg->steer_angle/Nsw )*M_PI/180;
  ekf.timeUpdate( del , speed );

  publishLatestState(stamp_);
  
  return;
}

void VehicleModelEkfNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
	ros::Time stamp_ = msg->header.stamp;
	
	calculateVehicleSpeed(msg->wheel_speed_left_front,msg->wheel_speed_right_front,msg->wheel_speed_left_rear,msg->wheel_speed_right_rear);

	return;
}

void VehicleModelEkfNode::publishLatestState(ros::Time stamp_){

  // -------- ROS Publish Odometry
  nav_msgs::Odometry odom_msg_;
  odom_msg_.header.stamp = stamp_;
  // odom_msg_.header.frame_id = odom_frame_id;
  // odom_msg_.child_frame_id = base_link_frame_id;
  
  odom_msg_.pose.pose.position.x = ekf.est.pos[0];
  odom_msg_.pose.pose.position.y = ekf.est.pos[1];

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, ekf.est.psi );

  odom_msg_.pose.pose.orientation = q;

  odom_msg_.twist.twist.linear.x = speed;
  // odom_msg_.twist.twist.angular.z = omega;

  odom_pub.publish(odom_msg_);

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