/*
 * 
 *
 * Dan Pierce
 * 2017-03-15
 */
#include "vehicle_model_pkg/vm_ekf.h"

VehicleModelEkf::VehicleModelEkf()
{
  std::cout << "VehicleModelEkf::VehicleModelEkf" << std::endl;
}

VehicleModelEkf::~VehicleModelEkf()
{
  std::cout << "VehicleModelEkf::~VehicleModelEkf" << std::endl;
}

void VehicleModelEkf::init(EkfParameters prm)
{
  // ----- Initialize State
  x.resize(3,1,false);
  x(0,0) = prm.pos_i[0];
  x(1,0) = prm.pos_i[1];
  x(2,0) = prm.psi_i;
  // ----- Process Uncertainty
  Q.resize(2,2,false);
  Q(0,0) = pow(prm.std_del,2);
  Q(1,1) = pow(prm.std_vel,2);
  // ----- Measurement Uncertainty
  R.resize(2,2,false);
  R(0,0) = pow(prm.std_pos,2);
  R(1,1) = pow(prm.std_pos,2);
  // ----- Initial Uncertainty
  P.resize(3,3,false);
  P(0,0) = pow(prm.std_pos_i,2);
  P(1,1) = pow(prm.std_pos_i,2);
  P(2,2) = pow(prm.std_psi_i,2);
  // ----- Measurement Model
  H.resize(2,3,false);
  H(0,0) = 1.0; H(1,1) = 1.0;

  // Should make this message a debug callback instead
  std::cout << "Vehicle Model EKF Initialized" << std::endl;

}

void VehicleModelEkf::timeUpdate(double del,double vel){
  ublas::matrix<double> u(2,1);
  u(0,0) = del; u(1,0) = vel;
  

}

void VehicleModelEkf::propagate(double del,double vel){
  
  // double omega = (vel/(a+b))*tan(del);
  
  // yaw += omega*dt;
  // wrapToPi(yaw);

  // pos[0] += cos(yaw)*speed*dt;
  // pos[1] += sin(yaw)*speed*dt;

  return;
}


void VehicleModelEkf::getStateJacobian(ublas::matrix<double> &F,double vel){
  F.resize(3,3,false);

  F(0,2) = -vel*sin(x(3,0));
  F(1,2) = vel*cos(x(3,0));
}

void VehicleModelEkf::getInputJacobian(ublas::matrix<double> &B,double del,double vel){
  B.resize(3,2,false);
  
  // [0,cos(x(3));0,sin(x(3));
  //  (u(2)*(tan(u(1))^2 + 1))/(prm.a + prm.b),tan(u(1))/(prm.a + prm.b)]

}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  // VehicleModelEkf ekf;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////