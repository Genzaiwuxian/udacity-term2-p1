#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	P_ = F_ * P_*(F_.transpose());
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	MatrixXd S = H_ * P_*H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose()*S.inverse();
	x_ = x_ + K * y;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	if (fabs(x_(0))<1.0e-8 && fabs(x_(1)<1.0e-8))
	{
		x_(0) = 1.0e-8;
		x_(0) = 1.0e-8;
	}
	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float theta = atan(x_(1) / x_(0));
	if (theta > M_PI)
	{
		theta -= 2 * M_PI;
	}
	else if (theta + M_PI < 0)
	{
		theta += 2 * M_PI;
	}
	float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
	h = VectorXd(3);
	h << rho, theta, rho_dot;
	VectorXd y = z - h;
	MatrixXd S = H_ * P_*H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose()*S.inverse();
	x_ = x_ + K * y;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H_)*P_;
}
