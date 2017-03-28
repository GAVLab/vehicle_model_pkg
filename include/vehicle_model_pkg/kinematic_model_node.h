/*
 * Header file for kinematic vehicle model node
 *
 * Dan Pierce
 * 2017-03-07
 */
#ifndef _KINEMATIC_MODEL_NODE_H_
#define _KINEMATIC_MODEL_NODE_H_

#include <ros/ros.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "g35can/g35can_steer_angle.h"
#include "g35can/g35can_wheel_speed.h"
          
/* Primary class for the kinematic vehicle model class */
class KinematicModelNode
{
  public:

    KinematicModelNode();
    ~KinematicModelNode();

    void propagate();

    double dt;

  private:

    void steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg);
    void wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg);
    
    void publishLatestState();

    void wrapToPi(double &ang);

    ros::NodeHandle nh;
    
    // Subscribers
    ros::Subscriber steer_sub,ws_sub;
    
    // Publishers
    ros::Publisher odom_pub; /*!< odometry publish (with respect to inititial pose) */

    ros::Time stamp;

    // Parameters
    double mass,wheel_radius,Nsw,a,b;
    int drive_type; // 0 for front wheel drive, 1 for rear wheel drive, 2 for all wheel drive

    // States/inputs
    double speed; // Linear speed calculated from wheel speeds
    double del; // Linear speed calculated from wheel speeds
    double pos[2]; // 2D position
    double yaw,omega; // 
};

#endif