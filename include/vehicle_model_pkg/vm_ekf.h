/*
 * Header file for vehicle model ekf
 *
 * Dan Pierce
 * 2017-03-07
 */
#ifndef _VEHICLE_MODEL_EKF_H_
#define _VEHICLE_MODEL_EKF_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace ublas = boost::numeric::ublas;

/* Type Definitions */
typedef struct
{
    // Time Step
    double dt;
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
    // Estimated State
    double pos[2];
    double psi;
    // Estimate Uncertainty
    double std_pos[2];
    double std_psi;
} EkfEstimate;

typedef struct
{
    double a;
    double b;
    double ox;
    double oy;   
} VehicleParameters;

/*! /brief Primary class for the kinematic vehicle model class
*
*/
class VehicleModelEkf
{
  public:

    VehicleModelEkf();
    ~VehicleModelEkf();

    void initialize();

    void timeUpdate(double del,double vel);

    bool measurementUpdate(double pos[2],double cov[2][2]);

    void propagate(double del,double vel);


    bool init;
    EkfParameters prm;
    EkfEstimate est;
    VehicleParameters vehicle;

  private:
    
    void getStateJacobian(ublas::matrix<double> &F,double vel);
    void getMeasurementJacobian(ublas::matrix<double> &H);
    void getDiscreteProcessCovariance(ublas::matrix<double> &Qd,double del,double vel);
    double wrapToPi(double ang);

    bool invertMatrix(const ublas::matrix<double>& input, ublas::matrix<double>& inverse);
    
    //////////////
    /* EKF STUFF*/
    //////////////
    // ----- Process Uncertainty
    ublas::matrix<double> Q;
    // ----- Initial Uncertainty
    ublas::matrix<double> P;
    
};

#endif