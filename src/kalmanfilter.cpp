#include "ekf/kalmanfilter.h"

void KalmanFilter::start(
  const int nin, const VectorXd& xin, const MatrixXd& Pin, const MatrixXd& Fin, const MatrixXd& Qin){

  this->n = nin;
  this->I = MatrixXd::Identity(this->n, this->n);
  this->x = xin;
  this->P = Pin;
  this->F = Fin;
  this->Q = Qin;
}

void KalmanFilter::setQ(const MatrixXd& Qin){
  this->Q = Qin;
}

void KalmanFilter::updateF(const double dt){
  this->F(0, 2) = dt;
  this->F(1, 3) = dt;
}

VectorXd KalmanFilter::get() const{
  return this->x;
}

void KalmanFilter::predict(){
  this->x = this->F * this->x;
  this->P = this->F * this->P * this->F.transpose() + this->Q;
}

void KalmanFilter::update(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R){

  const MatrixXd PHt = this->P * H.transpose();
  const MatrixXd S = H * PHt + R;
  const MatrixXd K = PHt * S.inverse();
  VectorXd y = z - Hx;

  // Assume this is radar measurement
  // y(1) is an angle (phi), it shoulde be normalized
  // refer to the comment at the bottom of this file
  if (y.size() == 3) y(1) = atan2(sin(y(1)), cos(y(1)));

  this->x = this->x + K * y;
  this->P = (this->I - K * H) * this->P;
}

/*
Normalizing Angles
In C++, atan2() returns values between -pi and pi.
When calculating phi in y = z - h(x) for radar measurements,
the resulting angle phi in the y vector should be adjusted
so that it is between -pi and pi. The Kalman filter is expecting
small angle values between the range -pi and pi.
*/
