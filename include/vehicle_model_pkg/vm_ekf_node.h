/*
 * Header file for vehicle model ekf
 *
 * Dan Pierce
 * 2017-03-07
 */
#ifndef _VEHICLE_MODEL_EKF_NODE_H_
#define _VEHICLE_MODEL_EKF_NODE_H_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "g35can/g35can_steer_angle.h"
#include "g35can/g35can_wheel_speed.h"

#include "vehicle_model_pkg/vm_ekf.h"

/*! /brief Primary class for the kinematic vehicle model class
*
*/
class VehicleModelEkfNode
{
  public:

    VehicleModelEkfNode();
    ~VehicleModelEkfNode();

  private:

    void steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg);
    void wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg);
    
    void publishLatestState(ros::Time stamp_);

    void calculateVehicleSpeed(double ws_lf,double ws_rf,double ws_lr,double ws_rr);
    
    void wrapToPi(double &ang);

    ros::NodeHandle nh;
    
    // Subscribers
    ros::Subscriber steer_sub;
    ros::Subscriber ws_sub;
    
    // Publishers
    ros::Publisher odom_pub; /*!< odometry publish (with respect to inititial pose) */
    
    tf::Transform pose; /*!< TF of odometry position */

    std::string odom_frame_id;
    std::string base_link_frame_id;

    tf::TransformBroadcaster tf_broadcaster;

    // 
    double dt;

    // Parameters
    double mass,wheel_radius,Nsw,a,b;
    int drive_type; // 0 for front wheel drive, 1 for rear wheel drive, 2 for all wheel drive

    // States
    double speed; // Linear speed calculated from wheel speeds
    double pos[2]; // 2D position
    double yaw,omega; // 

    // 
    VehicleModelEkf ekf;

};

#endif