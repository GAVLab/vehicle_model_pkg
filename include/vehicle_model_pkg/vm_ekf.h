/*
 * Header file for vehicle model ekf
 *
 * Dan Pierce
 * 2017-03-07
 */
#ifndef _VEHICLE_MODEL_EKF_H_
#define _VEHICLE_MODEL_EKF_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "g35can/g35can_steer_angle.h"
#include "g35can/g35can_wheel_speed.h"

namespace ublas = boost::numeric::ublas;

/* Type Definitions */
typedef struct
{
    // Initial State
    double pos_i[2];
    double psi_i;
    // Initial Uncertainty
    double std_pos_i;
    double std_psi_i;
    // Process Uncertainty
    double std_del;
    double std_vel;
    // Measurement Uncertainty
    double std_pos;

} EkfParameters;

typedef struct
{
    double a;
    double b;
} VehicleParameters;

/*! /brief Primary class for the kinematic vehicle model class
*
*/
class VehicleModelEkf
{
  public:

    VehicleModelEkf();
    ~VehicleModelEkf();

    void init(EkfParameters prm);

    void timeUpdate(double del,double vel);

    void propagate(double del,double vel);

    EkfParameters prm;

  private:
    
    void getStateJacobian(ublas::matrix<double> &F,double vel);
    void getInputJacobian(ublas::matrix<double> &B,double del,double vel);

    //////////////
    /* EKF STUFF*/
    //////////////
    // ----- State
    ublas::matrix<double> x;
    // ----- Process Uncertainty
    double std_del;
    double std_vel;
    ublas::matrix<double> Q;
    // ----- Measurement Uncertainty
    double std_pos;
    ublas::matrix<double> R;
    // ----- Initial Uncertainty
    double std_pos_i;
    double std_psi_i;
    ublas::matrix<double> P;
    // ----- Measurement Model
    ublas::matrix<double> H;

};

#endif