#include "kalman_filter.h"

namespace xmotors
{
    
namespace kalman_filter
{

void KalmanFilter::Init(const VectorXd &x, const MatrixXd &P, const double dt) {
    x_ = x; // x, y, yaw, v
    P_ = P;
    Q_ = MatrixXd(4,4);
    Q_(0,0) = 0.1; // variance of location on x-axis
    Q_(1,1) = 0.1; // variance of location on y-axis
    Q_(2,2) = 1.0/180.0*M_PI; // variance of location on x-axis
    Q_(3,3) = 1.0; // variance of velocity
    Q_ = Q_*Q_;
    R_ = MatrixXd(2,2); // observe only x, y
    R_(0,0) = 1.0;
    R_(0,0) = 1.0;
    R_ = R_*R_;
}

void KalmanFilter::Predict(const VectorXd &u) {
    // input is [v, yaw_rate]
    // predict state
    MotionModel(u);
    // predict covariance
    // P_ = (F_*P_)*F_.transpose() + Q_;
    // calculate Jacobian
    MatrixXd Jf = CalJacobianF(u);
    P_ = (Jf*P_)*Jf.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    // predict observation state
    ObervationModel();

    // calculate residual
    VectorXd y = z - z_;

    // update kalman gain
    // K_ = P_*H_.transpose() * (H_*P_*H_.transpose() + R_).inverse();
    MatrixXd Jh = CalJacobianH();
    K_ = P_*H_.transpose() * (Jh*P_*Jh.transpose() + R_).inverse();

    // update state
    x_ = x_ + K_*y;

    // update state covariance
    P_ = (MatrixXd::Identity(4,4) - K_*H_)*P_;
}

void KalmanFilter::MotionModel(const VectorXd &u) {
    // 2D discrete bicycle model

    F_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,        
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
    
    B_ << dt_*cos(x_(2)), 0.0,
          dt_*sin(x_(2)), 0.0,
          0.0,            dt_,
          1.0,            0.0;

    x_ = F_*x_ + B_*u;

}

void KalmanFilter::ObervationModel() {

    H_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

    z_ = H_*x_;
}

MatrixXd KalmanFilter::CalJacobianF(const VectorXd &u) {
    // Jacobian of Motion Model

    // motion model
    // x_{t+1} = x_t+v*dt*cos(yaw)
    // y_{t+1} = y_t+v*dt*sin(yaw)
    // yaw_{t+1} = yaw_t+omega*dt
    // v_{t+1} = v{t}
    // so
    // dx/dyaw = -v*dt*sin(yaw)
    // dx/dv = dt*cos(yaw)
    // dy/dyaw = v*dt*cos(yaw)
    // dy/dv = dt*sin(yaw)

    MatrixXd J;
    J << 1.0, 0.0, -u(0)*dt_*sin(x_(2)), dt_*cos(x_(2)),
         0.0, 1.0, u(0)*dt_*cos(x_(2)),  dt_*sin(x_(2)),        
         0.0, 0.0, 1.0,                  0.0,
         0.0, 0.0, 0.0,                  1.0;

    return J;
}

MatrixXd KalmanFilter::CalJacobianH() {
    MatrixXd J;
    J << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0; 
    
    return J;
}

} // namespace kalman_filter


} // namespace xmotors
