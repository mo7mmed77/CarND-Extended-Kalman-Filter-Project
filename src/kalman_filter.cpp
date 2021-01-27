#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

void KalmanFilter::Predict() {

  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;


}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */ //laser
  int x_size=x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);

  VectorXd y=z-H_*x_;
  MatrixXd S_=H_*P_*H_.transpose() +R_;
  MatrixXd K_=P_*H_.transpose()*S_.inverse();
  x_=x_+K_*y;

  P_=(I_ -K_*H_)*P_ +Q_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  long x_size=x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);

  VectorXd hx=VectorXd(3);
  double px =x_(0);
  double py=x_(1);
  double vx=x_(2);
  double vy=x_(3);

  double rho= sqrt(px*px + py*py);
  double rho_dot;

/*if(fabs(rho)<0,0001){
  rho_dot=0;
} else{
  */
 rho_dot=(vx*px +vy*py)/(rho);
//}

  double phi = atan2(py,px);
  hx<<rho,phi,rho_dot;

  VectorXd y=z-hx;

  // in order to normalize the phi angle between -pi and pi
  y(1)=atan2(sin(y(1)),cos(y(1)));

  MatrixXd S_=H_*P_*H_.transpose() +R_;
  MatrixXd K_=P_*H_.transpose()*S_.inverse();
  x_=x_+K_*y;

  P_=(I_ -K_*H_)*P_ +Q_;
}
