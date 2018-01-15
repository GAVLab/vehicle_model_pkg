#!/usr/bin/env python
import roslib
import sys
import rospy

from time import sleep 

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

import tf
from tf.transformations import quaternion_from_euler as e2q
from tf.transformations import euler_from_quaternion as q2e

import numpy as np

class G35TfBroadcaster:

  def __init__(self):
    # ---- Class variables
    self.reference_initialized = False
    self.ref_lla = (0.,0.,0.)

    self.rot_angle_fl = self.ang_vel_fl = 0.;
    self.rot_angle_rl = self.ang_vel_rl = 0.;
    self.rot_angle_fr = self.ang_vel_fr = 0.;
    self.rot_angle_rr = self.ang_vel_rr = 0.;

    self.pos = (0.,0.,0.);
    self.quat = (0.,0.,0.,1.);

    # ---- ROS parameters
    self.marker_frame_id = rospy.get_param('~marker_frame_id','/base')
    nav_topic = rospy.get_param('~nav_topic','/gps_ins_node/odom')
    self.a = rospy.get_param('~front_axle_to_cg',0.)
    self.b = rospy.get_param('~rear_axle_to_cg',0.)
    self.Nsw = rospy.get_param('~steering_wheel_gear_ratio',0.)
    self.wr = rospy.get_param('~wheel_radius',0.)
    self.t_f = rospy.get_param('~front_track_width',0.)
    self.t_r = rospy.get_param('~rear_track_width',0.)
    self.update_rate = rospy.get_param('~update_rate',20.0)

    self.dt = 1/self.update_rate

    # ---- ROS TF
    self.br = tf.TransformBroadcaster()

    # ---- ROS subscribers
    # ws_sub = rospy.Subscriber("/g35can_node/wheel_speeds",g35can_wheel_speed,self.ws_callback,queue_size=1)
    nav_sub = rospy.Subscriber(nav_topic, Odometry, self.navCallback,queue_size=1)

    self.stamp = None

    # ---- Orientation
    # self.quat = (0.,0.,0.,1.)
    # self.pos = (0.,0.,-1.0)

    self.ang_vel_fl = 0.
    self.ang_vel_rl = 0.
    self.ang_vel_fr = 0.
    self.ang_vel_rr = 0.

  def navCallback(self,msg):
    self.stamp = msg.header.stamp

    pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)

    quat = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    self.br.sendTransform( (0,0,0),quat,msg.header.stamp,"base_footprint",self.marker_frame_id)

    self.tf_update()

  def tf_update(self):

    if self.stamp is None:
        return

    stamp_ = self.stamp

    self.rot_angle_fl += self.ang_vel_fl*self.dt
    self.rot_angle_rl += self.ang_vel_rl*self.dt
    self.rot_angle_fr += self.ang_vel_fr*self.dt
    self.rot_angle_rr += self.ang_vel_rr*self.dt

    """""""""""""""
    "" Handle TF ""
    """""""""""""""
    pos_z = self.wr

    # Front left wheel (at tire contact)
    pos_x = self.a
    pos_y = self.t_f/2.
    q_tire = e2q(0., self.rot_angle_fl, 0.)

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,stamp_,
                          "wheel_left_front","base_footprint")

    # Front right wheel (at tire contact)
    pos_x = self.a
    pos_y = -self.t_f/2.
    q_tire = e2q(0., self.rot_angle_fr, 0.)

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,stamp_,
                          "wheel_right_front","base_footprint")

    # Rear left wheel (at tire contact)
    pos_x = -self.b
    pos_y = self.t_r/2.
    q_tire = e2q(0., self.rot_angle_rl, 0.)

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,stamp_,
                          "wheel_left_rear","base_footprint")

    # Rear right wheel (at tire contact)
    pos_x = -self.b
    pos_y = -self.t_r/2.
    q_tire = e2q(0., self.rot_angle_rr, 0.)

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,stamp_,
                          "wheel_right_rear","base_footprint")
    
def main(args):
  rospy.init_node('g35_tf_broadcaster')
  node = G35TfBroadcaster()

  rospy.spin()

  # while not rospy.is_shutdown():
  #   node.tf_update()
  #   sleep(node.dt)

if __name__ == '__main__':
    main(sys.argv)