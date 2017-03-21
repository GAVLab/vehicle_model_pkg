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
	nh.param("steering_wheel_gear_ratio", Nsw, 15.9);
	nh.param("front_axle_to_cg", a, 1.368);
	nh.param("rear_axle_to_cg", b, 1.482);

	nh.param("time_step", dt, 0.01);

	if (~drive_axle.compare("front")){drive_type=0;}else{drive_type=1;}
	
	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_node/steer_angle", 0, &KinematicModelNode::steerAngleCallback, this);
	ws_sub = nh.subscribe<g35can::g35can_wheel_speed>("/g35can_node/wheel_speeds", 0, &KinematicModelNode::wheelSpeedCallback, this);
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

}

KinematicModelNode::~KinematicModelNode()
{
  std::cout << "KinematicModelNode::~KinematicModelNode" << std::endl;
}

void KinematicModelNode::propagate(){
  
  omega = (speed/(a+b))*tan(del);
  
  yaw += omega*dt;
  wrapToPi(yaw);

  pos[0] += cos(yaw)*speed*dt;
  pos[1] += sin(yaw)*speed*dt;

  publishLatestState();

  return;
}

void KinematicModelNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  
  stamp = msg->header.stamp;

  del = ( msg->steer_angle/Nsw )*M_PI/180;
  
  return;
}

void KinematicModelNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
	
	calculateVehicleSpeed(msg->wheel_speed_left_front,msg->wheel_speed_right_front,msg->wheel_speed_left_rear,msg->wheel_speed_right_rear);

	return;
}

void KinematicModelNode::publishLatestState(){

  // -------- ROS Publish Odometry
  nav_msgs::Odometry odom_msg_;
  odom_msg_.header.stamp = stamp;
  odom_msg_.header.frame_id = odom_frame_id;
  odom_msg_.child_frame_id = base_link_frame_id;
  
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw );

  odom_msg_.pose.pose.orientation = q;

  odom_msg_.twist.twist.linear.x = speed;
  odom_msg_.twist.twist.angular.z = omega;

  odom_pub.publish(odom_msg_);

  // -------- ROS TF Odometry
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

void KinematicModelNode::calculateVehicleSpeed(double ws_lf,double ws_rf,double ws_lr,double ws_rr){
  
  switch(drive_type){
    case 0: // front wheel drive, calculate speed with rear wheels
      speed = (ws_lr+ws_rr)*(2*M_PI/60)*wheel_radius/2.0;
    case 1: // rear wheel drive, calculate speed with front wheels
      speed = (ws_lf+ws_rf)*(2*M_PI/60)*wheel_radius/2.0;
    case 2: // all wheel drive, calculate speed with all wheels
      speed = (ws_lf+ws_rf+ws_lr+ws_rr)*(2*M_PI/60)*wheel_radius/4.0;
  }

  return;
}

void KinematicModelNode::wrapToPi(double &ang){
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
  ros::init(argc,argv,"kinematic_model_node");
  KinematicModelNode node;

  ros::Rate rate_(100);
  while (ros::ok())
  {
    node.propagate();
    ros::spinOnce();
    rate_.sleep();
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////