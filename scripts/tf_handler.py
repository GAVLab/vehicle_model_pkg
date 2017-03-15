#!/usr/bin/env python
import roslib
import sys
import rospy
from g35can.msg import g35can_steer_angle
from g35can.msg import g35can_wheel_speed

from wgs_conversions.srv import WgsConversion

from nav_msgs.msg import Odometry

import tf
from tf.transformations import quaternion_from_euler as e2q
from tf.transformations import euler_from_quaternion as q2e

import numpy as np

class TfHandlerNode:

  def __init__(self):
    # rospy.on_shutdown(self.shutdown)

    # ---- Class variables
    self.last_odom_update_time = 0.
    self.del_tire = 0.
    self.reference_initialized = False
    self.odom_initialized = False
    self.ref_lla = (0.,0.,0.)
    
    self.vm_pos = (0.,0.,0.)
    self.vm_quat = (0.,0.,0.,1.)
    self.sep_pos = (0.,0.,0.)
    self.sep_quat = (0.,0.,0.,1.)
    self.rel_pos = (0.,0.,0.)
    self.rel_quat = (0.,0.,0.,1.)
    self.rel_pos_i = (0.,0.,0.)
    self.rel_quat_i = (0.,0.,0.,1.)

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
    odom_sub = rospy.Subscriber("/kinematic_model_node/odom",Odometry,self.odom_callback,queue_size=1)
    
    rospy.wait_for_service('xyz2lla')
    rospy.wait_for_service('xyz2enu')
    self.xyz2lla = rospy.ServiceProxy('xyz2lla', WgsConversion)
    self.xyz2enu = rospy.ServiceProxy('xyz2enu', WgsConversion)

    sep_sub = rospy.Subscriber("/septentrio/odom",Odometry,self.sep_callback,queue_size=1)


  def sep_callback(self,msg):
    # ---- Orientation
    quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    roll, pitch, yaw = q2e(quat)
    quat = e2q(roll, pitch, np.pi/2-yaw)
    self.sep_quat = quat

    # ---- Position
    xyz = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)

    if not self.reference_initialized:
      rsp = self.xyz2lla(xyz=xyz)
      self.ref_lla = rsp.lla
      self.reference_initialized = True
      self.sep_pos = (0.,0.,0.)
    else:
      rsp = self.xyz2enu(xyz=xyz,ref_lla=self.ref_lla)
      self.sep_pos = (rsp.enu[1],rsp.enu[0],-rsp.enu[2])

  def odom_callback(self,msg):
    self.vm_quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    self.vm_pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
    self.odom_initialized = True

  def steer_callback(self,msg):
    self.del_tire = ( msg.steer_angle/self.Nsw )*np.pi/180.;

    q_tire = e2q(0., 0., self.del_tire)

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

  def tf_update(self):
    stamp_ = rospy.get_rostime()
    self.br.sendTransform(self.sep_pos,self.sep_quat,stamp_,"sep_link","ned")

    self.br.sendTransform(self.vm_pos,self.vm_quat,stamp_,"vm_link","odom")
    if not self.reference_initialized or not self.odom_initialized:
      self.rel_pos_i = (self.vm_pos[0] - self.sep_pos[0],# + ,
                        self.vm_pos[1] - self.sep_pos[1],# + self.vm_pos[1],
                        self.vm_pos[2] - self.sep_pos[2])# + self.vm_pos[2])
      r1,p1,y1 = q2e(self.sep_quat)
      r2,p2,y2 = q2e(self.vm_quat)
      self.rel_quat_i = e2q(r2-r1,p2-p1,y2-y1)
      return

    if (stamp_.to_sec() - self.last_odom_update_time)>10:
      self.rel_pos = (self.rel_pos_i[0] - self.vm_pos[0] + self.sep_pos[0],# + ,
                      self.rel_pos_i[1] - self.vm_pos[1] + self.sep_pos[1],# + self.vm_pos[1],
                      self.rel_pos_i[2] - self.vm_pos[2] + self.sep_pos[2])# + self.vm_pos[2])
      r1,p1,y1 = q2e(self.sep_quat)
      r2,p2,y2 = q2e(self.vm_quat)
      r3,p3,y3 = q2e(self.rel_quat_i)
      self.rel_quat = e2q(r3-r2+r1,p3-p2+p1,y3-y2+y1,)
      self.last_odom_update_time = stamp_.to_sec()
    
    # Broadcast static TF between odom frame and enu frame (assumes that the both reference 
    # position and dead reckoning are initialized while car is static and that the dead 
    # reckoning is initialized to zero
    self.br.sendTransform(self.rel_pos,self.rel_quat,stamp_,"odom","ned")


def main(args):
  rospy.init_node('tf_handler_node')
  node = TfHandlerNode()

  r = rospy.Rate(20) # 10hz
  while not rospy.is_shutdown():
    node.tf_update()
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)