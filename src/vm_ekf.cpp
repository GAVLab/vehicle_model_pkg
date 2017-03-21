/*
 * 
 *
 * Dan Pierce
 * 2017-03-15
 */
#include "vehicle_model_pkg/vm_ekf.h"

VehicleModelEkf::VehicleModelEkf()
{
  P.resize(3,3);
  Q.resize(2,2);
  R.resize(2,2);

  std::cout << "VehicleModelEkf::VehicleModelEkf" << std::endl;
}

VehicleModelEkf::~VehicleModelEkf()
{
  std::cout << "VehicleModelEkf::~VehicleModelEkf" << std::endl;
}

void VehicleModelEkf::init()
{

  // ----- Initialize State
  est.pos[0] = prm.pos_i[0];
  est.pos[1] = prm.pos_i[1];
  est.psi = prm.psi_i;
  est.std_pos[0] = prm.std_pos_i;
  est.std_pos[1] = prm.std_pos_i;
  est.std_psi = prm.std_psi_i;
  // ----- Initial Uncertainty
  P.clear();
  P(0,0) = pow(prm.std_pos_i,2);
  P(1,1) = pow(prm.std_pos_i,2);
  P(2,2) = pow(prm.std_psi_i,2);
  // ----- Process Uncertainty
  Q.clear();
  Q(0,0) = pow(prm.std_del,2);
  Q(1,1) = pow(prm.std_vel,2);
  // ----- Measurement Uncertainty
  R.clear();
  R(0,0) = pow(prm.std_pos,2);
  R(1,1) = pow(prm.std_pos,2);
  
  // Should make this message a debug callback instead
  std::cout << "Vehicle Model EKF Initialized" << std::endl;

}

void VehicleModelEkf::timeUpdate(double del,double vel){

  // Print State
  // std::cout << "x = [" << est.pos[0] << std::endl;
  // std::cout << "     " << est.pos[1] << std::endl;
  // std::cout << "     " << est.psi << "]" << std::endl;
  // std::cout << "P = " << P << std::endl;
  // std::cout << "Q = " << Q << std::endl;
  // std::cout << "R = " << R << std::endl;
  
  propagate(del,vel);

  ublas::matrix<double> F, Qd;
  getStateJacobian(F,vel);
  getDiscreteProcessCovariance(Qd,del,vel);


  ublas::matrix<double> FP = prod(F,P); 

  P = prod(FP,trans(F)) + Qd; // P = F*P*F' + Qd;

  est.std_pos[0] = pow(P(0,0),0.5);
  est.std_pos[1] = pow(P(1,1),0.5);
  est.std_psi = pow(P(2,2),0.5);

  // std::cout << "x = [" << est.pos[0] << std::endl;
  // std::cout << "     " << est.pos[1] << std::endl;
  // std::cout << "     " << est.psi << "]" << std::endl;
  // std::cout << "F = " << F << std::endl;
  // std::cout << "Qd = " << Qd << std::endl;
  // std::cout << "P = " << P << std::endl;

}

bool VehicleModelEkf::measurementUpdate(double pos[2]){
  
  // Populate measurement vector
  ublas::matrix<double> y(2,1);
  y(0,0) = pos[0]; y(1,0) = pos[1];

  // Populate state vector
  ublas::matrix<double> x(3,1);
  x(0,0) = est.pos[0]; x(1,0) = est.pos[1]; x(2,0) = est.psi;

  // Measurement matrix
  ublas::matrix<double> H = ublas::zero_matrix<double> (2,3);
  H(0,0) = 1.0; H(1,1) = 1.0;

  // Calculate Kalman gain
  ublas::matrix<double> PHt = prod(P,trans(H)); // P*H'
  ublas::matrix<double> S = prod(H,PHt) + R; // innovation uncertainty (S = H*P*H' + R)
  ublas::matrix<double> Sinv(2,2); // inverse of S (initialize to be same size as S)

  if (!invertMatrix(S, Sinv)){
    std::cout << "Failed to perform matrix inversion for calculating Kalman gain!" << std::endl;
    return false;
  }

  ublas::matrix<double> K(3,2);
  K = prod(PHt,Sinv); // K = P*H'/(H*P*H' + R);

  // Update state estimate
  x += prod(K,y-prod(H,x)); // x = x + K*(y-H*x)

  // Update state estimate covariance matrix
  ublas::identity_matrix<double> I(3);

  P = prod(I-prod(K,H),P); // simple update equation
  P = 0.5*(P+trans(P)); // keep the covariance matrix symmetric

  est.pos[0] = x(0,0);
  est.pos[1] = x(1,0);
  est.psi = x(2,0);

  est.std_pos[0] = pow(P(0,0),0.5);
  est.std_pos[1] = pow(P(1,1),0.5);
  est.std_psi = pow(P(2,2),0.5);

  // // ublas::matrix<double> IminusKH = I-prod(K,H);
  // // ublas::matrix<double> RKt = prod(R,trans(K));
  // // ublas::matrix<double> KRKt = prod(K,RKt);
  // // P = prod(IminusKH,P); // update state estimate covariance matrix
  // // (I - K*H)*P*(eye(size(P)) - K*H)'+K*R*K';
  
  return true;
}


void VehicleModelEkf::propagate(double del,double vel){
  
  double omega = (vel/(vehicle.a+vehicle.b))*tan(del);
  
  est.psi = wrapToPi(est.psi + omega*prm.dt);

  est.pos[0] += cos(est.psi)*vel*prm.dt;
  est.pos[1] += sin(est.psi)*vel*prm.dt;

}


void VehicleModelEkf::getStateJacobian(ublas::matrix<double> &F,double vel){
  ublas::identity_matrix<double> I (3);
  ublas::matrix<double> A = ublas::zero_matrix<double> (3,3);

  
  A(0,2) = -vel*sin(est.psi);
  A(1,2) = vel*cos(est.psi);

  F = I + A*prm.dt;
}

void VehicleModelEkf::getDiscreteProcessCovariance(ublas::matrix<double> &Qd,double del,double vel){
  ublas::matrix<double> Bw = ublas::zero_matrix<double> (3,2); // Input Noise Jacobian

  Bw(0,1) = cos(est.psi); Bw(1,1) = sin(est.psi);
  Bw(2,0) = vel*(pow(tan(del),2) + 1)/(vehicle.a + vehicle.b);
  Bw(2,1) = tan(del)/(vehicle.a + vehicle.b);

  ublas::matrix<double> BwQ = prod(Bw,Q); 
  Qd = prod(BwQ,trans(Bw))*prm.dt;  // Qd = Bw*Q*Bw'*dt;

}

bool VehicleModelEkf::invertMatrix(const ublas::matrix<double>& input, ublas::matrix<double>& inverse)
{
  typedef ublas::permutation_matrix<std::size_t> pmatrix;

  // create a working copy of the input
  ublas::matrix<double> A(input);

  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.size1());

  // perform LU-factorization
  int res = ublas::lu_factorize(A, pm);
  if (res != 0)
    return false;

  // create identity matrix of "inverse"
  inverse.assign(ublas::identity_matrix<double> (A.size1()));

  // backsubstitute to get the inverse
  ublas::lu_substitute(A, pm, inverse);

  return true;
}

double VehicleModelEkf::wrapToPi(double ang){
    ang = fmod(ang+M_PI,2*M_PI);
    if (ang < 0)
        ang += 2*M_PI;
    return (ang-M_PI);
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

