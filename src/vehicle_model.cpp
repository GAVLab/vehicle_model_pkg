/*
 * 
 *
 * Dan Pierce
 * 2017-03-15
 */
#include "vehicle_model_pkg/vehicle_model.h"

VehicleModel::VehicleModel()
{
  isInit = false;

  state_.yawRate = 0.0;
  state_.lateralVelocity = 0.0;
  state_.sideslipAngle = 0.0;

  dynamicModelMinSpeed_ = 5.0;

  std::cout << "VehicleModel::VehicleModel" << std::endl;
}

VehicleModel::~VehicleModel()
{
  std::cout << "VehicleModel::~VehicleModel" << std::endl;
}

void VehicleModel::update(double steerAngle, double vehicleSpeed, double timeStep) {

  if (vehicleSpeed < dynamicModelMinSpeed_) {
    kinematicModelUpdate(steerAngle,vehicleSpeed);
  } else {
    dynamicModelUpdate(steerAngle,vehicleSpeed,timeStep);
  }

  // publishLatestState();

}

void VehicleModel::kinematicModelUpdate(double del, double speed) {

  // beta = atan2(b*tan(del),a+b);
  // omega = (vel/(a+b))*cos(beta)*tan(del);

  double L = prm_.frontAxleToCg + prm_.rearAxleToCg;
  state_.yawRate = ( speed / L ) * tan(del);

  state_.yawAccel = 0.0;
  state_.lateralVelocity = 0.0;
  state_.lateralAccel = 0.0;
  state_.sideslipAngle = 0.0;
  state_.lateralTireForceFront = 0.0;
  state_.lateralTireForceRear = 0.0;

}

void VehicleModel::dynamicModelUpdate(double del, double speed, double timeStep) {
    
    /* Unpack states */
    double vy = state_.lateralVelocity;
    double r = state_.yawRate;

    double vx = pow(pow(speed,2)-pow(vy,2),0.5);

    /* Unpack parameters */
    double Izz = prm_.yawInertia;
    double m = prm_.mass;

    double a = prm_.frontAxleToCg;
    double b = prm_.rearAxleToCg;

    double track_f = prm_.frontTrackWidth;
    double track_r = prm_.rearTrackWidth;

    double cAlpha_f = prm_.frontLateralTireStiffness;
    double cAlpha_r = prm_.rearLateralTireStiffness;

    /* Calculate slip angles at tires */
    double alpha_fl=atan((vy+r*a)/(vx+(track_f/2)*r))-del;
    double alpha_fr=atan((vy+r*a)/(vx-(track_f/2)*r))-del;
    double alpha_rl=atan((vy-r*b)/(vx+(track_r/2)*r));
    double alpha_rr=atan((vy-r*b)/(vx-(track_r/2)*r));
    
    /* Update slip angle and forces */
    double alpha_f = (alpha_fl + alpha_fr)/2;
    double alpha_r  = (alpha_rl + alpha_rr)/2;
    
    double Fy_f = -cAlpha_f*alpha_f*2;
    double Fy_r = -cAlpha_r*alpha_r*2;

    /* Yaw rate dynamics */
    double rDot = (1/Izz) * (a*Fy_f*cos(del) - b*Fy_r);
    r += timeStep*rDot;

    /* Lateral dynamics */
    double vyDot = (Fy_f*cos(del) + Fy_r)/m - vx*r;
    vy += timeStep*vyDot;

    double beta = atan2(vy,vx);

    /* Package states */
    state_.yawRate = r;
    state_.yawAccel = rDot;
    state_.lateralVelocity = vy;
    state_.lateralAccel = vyDot;
    state_.sideslipAngle = beta;
    state_.lateralTireForceFront = Fy_f;
    state_.lateralTireForceRear = Fy_r;

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

