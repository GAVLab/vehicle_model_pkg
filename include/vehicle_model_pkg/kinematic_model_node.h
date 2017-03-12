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
#include <tf/transform_broadcaster.h>

#include "g35can/g35can_steer_angle.h"
          
/*! /brief Primary class for the kinematic vehicle model class
*
*/
class KinematicModelNode
{
  public:

    KinematicModelNode();
    ~KinematicModelNode();

  private:

    void steerAngleCallback(const g35can::g35can_steer_angle::ConstPtr& steer_msg);
    
    ros::NodeHandle nh;
    
    // Subscribers
    ros::Subscriber steer_sub;
    
    // Publishers
    ros::Publisher odom_pub; /*!< odometry publish (with respect to inititial pose) */
    
    tf::Transform pose; /*!< TF of odometry position */

    std::string odom_frame_id;
    std::string base_link_frame_id;

    tf::TransformBroadcaster tf_broadcaster;

    double mass;

};

#endif