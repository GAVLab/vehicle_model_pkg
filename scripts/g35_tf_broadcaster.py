#!/usr/bin/env python
import roslib
import sys
import rospy
from g35can.msg import g35can_steer_angle
from g35can.msg import g35can_wheel_speed

from wgs_conversions.srv import WgsConversion

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

    self.sep_pos = (0.,0.,0.);
    self.sep_quat = (0.,0.,0.,1.);

    # ---- ROS parameters
    self.marker_frame_id = rospy.get_param('~marker_frame_id','/base_footprint')
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

    # ---- ROS services
    rospy.wait_for_service('xyz2lla')
    rospy.wait_for_service('xyz2enu')
    self.xyz2lla = rospy.ServiceProxy('xyz2lla', WgsConversion)
    self.xyz2enu = rospy.ServiceProxy('xyz2enu', WgsConversion)

    # ---- ROS subscribers
    sep_sub = rospy.Subscriber("/septentrio/odom",Odometry,self.sep_callback,queue_size=1)
    ws_sub = rospy.Subscriber("/g35can_node/wheel_speeds",g35can_wheel_speed,self.ws_callback,queue_size=1)

  def sep_callback(self,msg):
    # ---- Orientation
    self.sep_quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    # ---- Position
    xyz = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)

    if not self.reference_initialized:
      rsp = self.xyz2lla(xyz=xyz)
      self.ref_lla = rsp.lla
      self.reference_initialized = True
      self.sep_pos = (0.,0.,0.)
      return
    
    rsp = self.xyz2enu(xyz=xyz,ref_lla=self.ref_lla)
    self.sep_pos = (rsp.enu[0],rsp.enu[1],rsp.enu[2])

  def ws_callback(self,msg):
    self.ang_vel_fl = msg.wheel_speed_left_front*(2*np.pi/60)*self.wr # [rad/s]
    self.ang_vel_rl = msg.wheel_speed_left_rear*(2*np.pi/60)*self.wr # [rad/s]
    self.ang_vel_fr = msg.wheel_speed_right_front*(2*np.pi/60)*self.wr # [rad/s]
    self.ang_vel_rr = msg.wheel_speed_right_rear*(2*np.pi/60)*self.wr # [rad/s]

  def tf_update(self):
    stamp_ = rospy.get_rostime()

    self.rot_angle_fl += self.ang_vel_fl*self.dt
    self.rot_angle_rl += self.ang_vel_rl*self.dt
    self.rot_angle_fr += self.ang_vel_fr*self.dt
    self.rot_angle_rr += self.ang_vel_rr*self.dt

    """""""""""""""
    "" Handle TF ""
    """""""""""""""
    self.br.sendTransform(self.sep_pos,self.sep_quat,stamp_,"base_footprint","ned")

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

  r = rospy.Rate(node.update_rate)
  while not rospy.is_shutdown():
    node.tf_update()
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)