/*
 * ROS node wrapping the dynamic vehicle model class
 *
 * Dan Pierce
 * 2017-03-07
 */
#include "vehicle_model_pkg/dynamic_model_node.h"

DynamicModelNode::DynamicModelNode()
{
	ros::NodeHandle nh("~");

	std::string drive_axle;
	nh.param("mass", m, 1528.2);
  nh.param("yaw_inertia", Izz, 2400.0);
  nh.param("front_lateral_tire_stiffness", CaF, 45836.6);
  nh.param("rear_lateral_tire_stiffness", CaR, 74484.5);
	nh.param("wheel_radius", wr, 0.3255);
	nh.param("drive_axle", drive_axle, std::string("front"));
	nh.param("steering_wheel_gear_ratio", Nsw, 15.9);
	nh.param("front_axle_to_cg", a, 1.368);
	nh.param("rear_axle_to_cg", b, 1.482);
  nh.param("front_track_width", tf, 1.5011);
  nh.param("rear_track_width", tr, 1.5062);

	nh.param("time_step", dt, 0.01);

	if (~drive_axle.compare("front")){drive_type=0;}else{drive_type=1;}
	
	steer_sub = nh.subscribe<g35can::g35can_steer_angle>("/g35can_node/steer_angle", 0, &DynamicModelNode::steerAngleCallback, this);
	ws_sub = nh.subscribe<g35can::g35can_wheel_speed>("/g35can_node/wheel_speeds", 0, &DynamicModelNode::wheelSpeedCallback, this);
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

  // Initialize states
  pos[0],pos[1],vx,vy,yaw,omega=0.0;

}

DynamicModelNode::~DynamicModelNode()
{
  std::cout << "DynamicModelNode::~DynamicModelNode" << std::endl;
}

void DynamicModelNode::propagate(){
  
  double Fyf, Fyr, omega_dot, vy_dot;

  calculateTireForce(Fyf,Fyr);

  // --- Propagate yaw
  omega_dot = (a*Fyf*cos(del) - b*Fyr)/Izz;
  omega += dt*omega_dot;
  yaw += dt*omega;
  wrapToPi(yaw);

  // --- Propagate position
  vy_dot = (Fyf*cos(del) + Fyr)/m - vx*omega;
  vy += dt * vy_dot;
  pos[0]+=dt*(vx*cos(yaw)-vy*sin(yaw));
  pos[1]+=dt*(vx*sin(yaw)+vy*cos(yaw));

  publishLatestState();

}

void DynamicModelNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  
  stamp = msg->header.stamp;

  del = ( msg->steer_angle/Nsw )*M_PI/180;
  
}

void DynamicModelNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
  
  double ws_lf = msg->wheel_speed_left_front;
  double ws_rf = msg->wheel_speed_right_front;
  double ws_lr = msg->wheel_speed_left_rear;
  double ws_rr = msg->wheel_speed_right_rear;

  double vel;
  switch(drive_type){
    case 0: // front wheel drive, calculate speed with rear wheels
      vel = (ws_lr+ws_rr)*(2*M_PI/60)*wr/2.0;
    case 1: // rear wheel drive, calculate speed with front wheels
      vel = (ws_lf+ws_rf)*(2*M_PI/60)*wr/2.0;
    case 2: // all wheel drive, calculate speed with all wheels
      vel = (ws_lf+ws_rf+ws_lr+ws_rr)*(2*M_PI/60)*wr/4.0;
  }

  vx = pow(pow(vel,2) - pow(vy,2),0.5);

}

void DynamicModelNode::publishLatestState(){

  // -------- ROS Publish Odometry
  nav_msgs::Odometry msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = "ENU";
  
  msg.pose.pose.position.x = pos[1];
  msg.pose.pose.position.y = pos[0];

  msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI/2-yaw );

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.x = -vy;
  msg.twist.twist.angular.z = -omega;

  odom_pub.publish(msg);

}

void DynamicModelNode::calculateTireForce(double &Fyf,double &Fyr){

  double alpha_FL = atan((vy+omega*a)/(vx+(tf/2)*omega))-del;
  double alpha_FR = atan((vy+omega*a)/(vx-(tf/2)*omega))-del;
  double alpha_RL = atan((vy-omega*b)/(vx+(tr/2)*omega));
  double alpha_RR = atan((vy-omega*b)/(vx-(tr/2)*omega));

  Fyf = -CaF*alpha_FL - CaF*alpha_FR;
  Fyr = -CaR*alpha_RL - CaR*alpha_RR;

}

void DynamicModelNode::wrapToPi(double &ang){
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
  ros::init(argc,argv,"dynamic_model_node");
  DynamicModelNode node;

  sleep(100); // allow node to initialize

  ros::Rate rate_(1/node.dt);
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