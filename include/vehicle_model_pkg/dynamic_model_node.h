/*
 * Header file for dynamic vehicle model node
 *
 * Dan Pierce
 * 2017-03-07
 */
#ifndef _DYNAMIC_MODEL_NODE_H_
#define _DYNAMIC_MODEL_NODE_H_

#include <ros/ros.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "g35can/g35can_steer_angle.h"
#include "g35can/g35can_wheel_speed.h"
          
/*! /brief Primary class for the dynamic vehicle model class
*
*/
class DynamicModelNode
{
  public:

    DynamicModelNode();
    ~DynamicModelNode();

    void propagate();

    double dt;

  private:

    void steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& msg);
    void wheelSpeedCallback(const g35can::g35can_wheel_speed::ConstPtr& msg);
    
    void publishLatestState();

    void calculateTireForce(double &Fyf,double &Fyr);

    void wrapToPi(double &ang);

    ros::NodeHandle nh;
    
    // Subscribers
    ros::Subscriber steer_sub,ws_sub;
    
    // Publishers
    ros::Publisher odom_pub; /*!< odometry publish (with respect to inititial pose) */
    
    ros::Time stamp;

    // Parameters
    double m,wr,Nsw,a,b,Izz,CaF,CaR,tr,tf;
    int drive_type; // 0 for front wheel drive, 1 for rear wheel drive, 2 for all wheel drive

    // States/inputs
    double vx; // x component of vehicle velocity
    double vy; // y component of vehicle velocity
    
    double del; // Linear speed calculated from wheel speeds
    double pos[2]; // 2D position
    double yaw,omega; // 
};

#endif