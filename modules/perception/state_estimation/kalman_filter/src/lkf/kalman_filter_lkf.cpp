/* 
 * kalman_filter_lkf.cpp
 * 
 * Created on: Feb 18, 2022 21:47
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#include "kalman_filter/kalman_filter.hpp"

#include "beacon/beacon.hpp"
#include "sensor/gps_sensor.hpp"
#include "sensor/gyro_sensor.hpp"
#include "sensor/lidar_sensor.hpp"

#include "helper/helper.hpp"

constexpr bool INIT_ON_FIRST_PREDICTION = false;
constexpr double INIT_POS_STD = 0;
constexpr double INIT_VEL_STD = 15;
constexpr double ACCEL_STD = 0.1;
constexpr double GPS_POS_STD = 3.0;

namespace pllee4 {
void KalmanFilter::PredictionStep(double dt) {
  if (!IsInitialised() && INIT_ON_FIRST_PREDICTION) {
    // Implement the State Vector and Covariance Matrix Initialisation in the
    // section below if you want to initialise the filter WITHOUT waiting for
    // the first measurement to occur. Make sure you call the SetState() /
    // SetCovariance() functions once you have generated the initial conditions.
    // Assume the state vector has the form [X,Y,VX,VY].
    VectorXd state = Vector4d::Zero();
    MatrixXd cov = Matrix4d::Zero();

    // Assume the initial position is (X,Y) = (0,0) m
    // Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) =
    // (5*cos(45deg),5*sin(45deg)) m/s
    state << 0, 0, 5.0 * cos(M_PI / 4), 5.0 * sin(M_PI / 4);

    const double init_pos_std = INIT_POS_STD;
    const double init_vel_std = INIT_VEL_STD;
    cov(0, 0) = INIT_POS_STD * INIT_POS_STD;
    cov(1, 1) = INIT_POS_STD * INIT_POS_STD;
    cov(2, 2) = INIT_VEL_STD * INIT_VEL_STD;
    cov(3, 3) = INIT_VEL_STD * INIT_VEL_STD;

    SetState(state);
    SetCovariance(cov);
  }

  if (IsInitialised()) {
    VectorXd state = GetState();
    MatrixXd cov = GetCovariance();

    // Kalman Filter Prediction Step
    MatrixXd F = Matrix4d();
    F << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    MatrixXd Q = Matrix2d::Zero();
    Q(0, 0) = (ACCEL_STD * ACCEL_STD);
    Q(1, 1) = (ACCEL_STD * ACCEL_STD);

    MatrixXd L = MatrixXd(4, 2);
    L << (0.5 * dt * dt), 0, 0, (0.5 * dt * dt), dt, 0, 0, dt;

    state = F * state;
    cov = F * cov * F.transpose() + L * Q * L.transpose();

    SetState(state);
    SetCovariance(cov);
  }
}

void KalmanFilter::HandleGPSMeasurement(const GPSMeasurement& meas) {
  if (IsInitialised()) {
    VectorXd state = GetState();
    MatrixXd cov = GetCovariance();

    // Kalman Filter Update Step for the GPS Measurements
    // section below.
    // Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.

    VectorXd z = Vector2d();
    MatrixXd H = MatrixXd(2, 4);
    MatrixXd R = Matrix2d::Zero();

    z << meas.x, meas.y;
    H << 1, 0, 0, 0, 0, 1, 0, 0;
    R(0, 0) = GPS_POS_STD * GPS_POS_STD;
    R(1, 1) = GPS_POS_STD * GPS_POS_STD;

    VectorXd z_hat = H * state;
    VectorXd y = z - z_hat;
    MatrixXd S = H * cov * H.transpose() + R;
    MatrixXd K = cov * H.transpose() * S.inverse();

    state = state + K * y;
    cov = (MatrixXd::Identity(4, 4) - K * H) * cov;

    SetState(state);
    SetCovariance(cov);
  } else {
    // State Vector and Covariance Matrix Initialisation
    // Assume the state vector has the form [X,Y,VX,VY].

    VectorXd state = Vector4d::Zero();
    MatrixXd cov = Matrix4d::Zero();

    state(0) = meas.x;
    state(1) = meas.y;
    cov(0, 0) = GPS_POS_STD * GPS_POS_STD;
    cov(1, 1) = GPS_POS_STD * GPS_POS_STD;
    cov(2, 2) = INIT_VEL_STD * INIT_VEL_STD;
    cov(3, 3) = INIT_VEL_STD * INIT_VEL_STD;

    SetState(state);
    SetCovariance(cov);
  }
}

Matrix2d KalmanFilter::GetRobotStatePositionCovariance() {
  Matrix2d pos_cov = Matrix2d::Zero();
  MatrixXd cov = GetCovariance();
  if (IsInitialised() && cov.size() != 0) {
    pos_cov << cov(0, 0), cov(0, 1), cov(1, 0), cov(1, 1);
  }
  return pos_cov;
}

RobotState KalmanFilter::GetRobotState() {
  if (IsInitialised()) {
    VectorXd state = GetState();  // STATE VECTOR [X,Y,VX,VY]
    double psi = std::atan2(state[3], state[2]);
    double v = std::sqrt(state[2] * state[2] + state[3] * state[3]);
    return RobotState(state[0], state[1], psi, v);
  }
  return RobotState();
}

void KalmanFilter::PredictionStep(const GyroMeasurement& gyro, double dt) {
  PredictionStep(dt);
}
void KalmanFilter::HandleLidarMeasurements(
    const std::vector<LidarMeasurement>& dataset, const BeaconMap& map) {}
void KalmanFilter::HandleLidarMeasurement(const LidarMeasurement& meas,
                                          const BeaconMap& map) {}
}  // namespace pllee4