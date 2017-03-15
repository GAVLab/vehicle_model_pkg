#!/usr/bin/env python
import roslib
import sys
import rospy
from g35can.msg import g35can_steer_angle
from g35can.msg import g35can_wheel_speed

from wgs_conversions.srv import WgsConversion

from nav_msgs.msg import Odometry

import tf
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import euler_from_quaternion as efq

import numpy as np

class MarkerHandlerNode:

  def __init__(self):
    # rospy.on_shutdown(self.shutdown)

    # ---- Class variables
    self.del_tire = 0.
    
    # ---- ROS parameters
    self.a = rospy.get_param('~front_axle_to_cg',0.)
    self.b = rospy.get_param('~rear_axle_to_cg',0.)
    self.Nsw = rospy.get_param('~steering_wheel_gear_ratio',0.)
    self.hcg = rospy.get_param('~sprung_cg_height',0.)
    self.wr = rospy.get_param('~wheel_radius',0.)
    self.t_f = rospy.get_param('~front_track_width',0.)
    self.t_r = rospy.get_param('~rear_track_width',0.)

    # ---- ROS TF
    self.br = tf.TransformBroadcaster()

    # ---- ROS subscribers
    steer_sub = rospy.Subscriber("/g35can_node/steer_angle",g35can_steer_angle,self.steer_callback,queue_size=1)
    # ws_sub = rospy.Subscriber("/g35can_node/wheel_speeds",g35can_wheel_speed,self.ws_callback,queue_size=1)
    # odom_sub = rospy.Subscriber("/kinematic_model_node/odom",Odometry,self.odom_callback,queue_size=1)
    
  def steer_callback(self,msg):
    self.del_tire = ( msg.steer_angle/self.Nsw )*np.pi/180.;
    
    print self.del_tire*180/np.pi

    q_tire = qfe(0., 0., self.del_tire)

    """""""""""""""""""""""""""
    "" Handle TFs
    """""""""""""""""""""""""""
    # Front left wheel (at center of wheel)
    pos_x = self.a
    pos_y = -self.t_f/2.
    pos_z = self.hcg-self.wr

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,msg.header.stamp,
                          "wheel_left_front","vm_link")

    # Front right wheel (at center of wheel)
    pos_x = self.a
    pos_y = self.t_f/2.
    pos_z = self.hcg-self.wr

    self.br.sendTransform((pos_x, pos_y, pos_z),q_tire,msg.header.stamp,
                          "wheel_right_front","vm_link")

    # Rear left wheel (at center of wheel)
    pos_x = -self.b
    pos_y = -self.t_r/2.
    pos_z = self.hcg-self.wr

    self.br.sendTransform((pos_x, pos_y, pos_z),(0.,0.,0.,1.),msg.header.stamp,
                          "wheel_left_rear","vm_link")

    # Rear right wheel (at center of wheel)
    pos_x = -self.b
    pos_y = self.t_r/2.
    pos_z = self.hcg-self.wr

    self.br.sendTransform((pos_x, pos_y, pos_z),(0.,0.,0.,1.),msg.header.stamp,
                          "wheel_right_rear","vm_link")


    """""""""""""""""""""""""""
    "" Markers
    """""""""""""""""""""""""""


def main(args):
  rospy.init_node('marker_handler_node')
  node = MarkerHandlerNode()
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)