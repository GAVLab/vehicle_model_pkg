/*
 * Header file for dynamic vehicle model node
 *
 * Dan Pierce
 * 2018-01-14
 */
#ifndef _VEHICLE_MODEL_H_
#define _VEHICLE_MODEL_H_

#include <math.h>
#include <iostream>

/* Type Definitions */
typedef struct
{
    double mass;
    double yawInertia;
    double frontAxleToCg;
    double rearAxleToCg;
    double wheelRadius;
    double frontTrackWidth;
    double rearTrackWidth; 
    double frontLateralTireStiffness;
    double rearLateralTireStiffness;
    double steeringWheelGearRatio;
} VehicleModelParameters;

typedef struct
{
    double yawRate;
    double yawAccel;
    double lateralVelocity;
    double lateralAccel;
    double sideslipAngle;
    double lateralTireForceFront;
    double lateralTireForceRear;
} VehicleModelState;

class VehicleModel
{
  public:

    VehicleModel();
    ~VehicleModel();
    
    // bool setVehicleModelParameters();

    // Variables
    VehicleModelParameters prm_;
    
    void update(double steerAngle, double vehicleSpeed, double timeStep);

    VehicleModelState state_;

    double dynamicModelMinSpeed_;

  private:

    void dynamicModelUpdate(double steerAngle, double vehicleSpeed, double timeStep);
    void kinematicModelUpdate(double steerAngle, double vehicleSpeed);

    bool isInit;

};

#endif
