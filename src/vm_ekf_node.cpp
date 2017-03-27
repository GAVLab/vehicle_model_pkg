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
  
  // ----- Time Step
  nh.param("time_step", dt, 0.01);
  // ----- Process Uncertainty
  nh.param("std_steer_angle", ekf.prm.std_del, 0.002);
  nh.param("std_velocity", ekf.prm.std_vel, 0.03);

  // ----- Initial Uncertainty
  nh.param("std_initial_position", ekf.prm.std_pos_i, 0.1);
  nh.param("std_initial_heading", ekf.prm.std_psi_i, 0.1);

  ekf.prm.dt = dt;

  /////////////////////////
  /* ROS Service Clients */
  /////////////////////////
  enu_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu");
  lla_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2lla");
  cov_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu_cov");
  vel_client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu_vel");

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

void VehicleModelEkfNode::propagate(){

	ekf.timeUpdate( del , speed );

	publishLatestState();

	return;
}

void VehicleModelEkfNode::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
  
  wgs_conversions::WgsConversion srv;

  srv.request.xyz[0] = msg->pose.pose.position.x;
  srv.request.xyz[1] = msg->pose.pose.position.y;
  srv.request.xyz[2] = msg->pose.pose.position.z;

  //////////////////////////////////////
  /* Get reference lat, long, and alt */
  //////////////////////////////////////
  if (!ekf.init){
    // Convert position measurement from XYZ to LLA
    if(lla_client.call(srv)){
      for(int i=0;i<3;i++)
        ref_lla[i] = srv.response.lla[i];
      ROS_INFO("\nReference LLA set:\n\tLatitude: %0.12f\tLongitude: %0.12f\tAltitude: %0.12f",ref_lla[0],ref_lla[1],ref_lla[2]);
    }else{
      ROS_ERROR("Failed to call service xyz2lla");
    }
  }
  
  //////////////////////////////////////
  /* Convert position from XYZ to ENU */
  //////////////////////////////////////
  for(int i=0;i<3;i++)
    srv.request.ref_lla[i] = ref_lla[i];

  double gps_pos[2];
  if(enu_client.call(srv)){

    gps_pos[0] = srv.response.enu[0]; // gps_pos is in NED convention
    gps_pos[1] = srv.response.enu[1];
  
  }else{
    ROS_ERROR("Failed to call service xyz2enu");
    return;
  }

  ////////////////////////////////////////
  /* Convert covariance from XYZ to ENU */
  ////////////////////////////////////////
  for(int i=0;i<9;i++)
    srv.request.xyz_cov[i] = msg->pose.covariance[i+3*(i>2)+3*(i>5)]; // Weird indexing is there to skip over attitude covariance

  double pos_cov[2][2];
  if (cov_client.call(srv)){
    
    for(int i=0;i<2;i++)
      for(int j=0;j<2;j++)
        pos_cov[i][j]=srv.response.enu_cov[3*i+j];

  }else{
    ROS_ERROR("Failed to call service xyz2enu_cov");
    return;
  }

  if (ekf.init){
  
    ekf.measurementUpdate( gps_pos , pos_cov );
  
  }else{

    //////////////////////////////////////
    /* Convert velocity from XYZ to ENU */
    //////////////////////////////////////
    srv.request.xyz[0] = msg->twist.twist.linear.x;
    srv.request.xyz[1] = msg->twist.twist.linear.y;
    srv.request.xyz[2] = msg->twist.twist.linear.z;

    double gps_course;
    if (vel_client.call(srv)){

        gps_course = atan2(srv.response.enu[1],srv.response.enu[0]);

    }else{
      ROS_ERROR("Failed to call service xyz2enu_vel");
      return;
    }

    ekf.prm.pos_i[0] = gps_pos[0];
    ekf.prm.pos_i[1] = gps_pos[1]; 
    ekf.prm.psi_i = gps_course;
    ekf.prm.std_pos_i = pos_cov[0][0];

    ekf.initialize();

  }
  
  return;
}

void VehicleModelEkfNode::steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg){
  
  stamp = msg->header.stamp;

  del = -( msg->steer_angle/Nsw )*M_PI/180;
  
  return;
}

void VehicleModelEkfNode::wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg){
	
	double ws_lf = msg->wheel_speed_left_front;
	double ws_rf = msg->wheel_speed_right_front;
	double ws_lr = msg->wheel_speed_left_rear;
	double ws_rr = msg->wheel_speed_right_rear;

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

void VehicleModelEkfNode::publishLatestState(){

  // -------- ROS Publish Odometry
  nav_msgs::Odometry odom_msg_;
  odom_msg_.header.stamp = stamp;
  odom_msg_.header.frame_id = "ENU";
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc,argv,"vehicle_model_ekf");
  VehicleModelEkfNode node;

  sleep(1); // allow node to initialize

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