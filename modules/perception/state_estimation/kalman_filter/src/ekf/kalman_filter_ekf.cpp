/* 
 * kalman_filter_ekf.cpp
 * 
 * Created on: Feb 18, 2022 21:48
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

constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01 / 180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0 / 180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;

namespace pllee4 {
void KalmanFilter::HandleLidarMeasurements(
    const std::vector<LidarMeasurement>& dataset, const BeaconMap& map) {
  // Assume No Correlation between the Measurements and Update Sequentially
  for (const auto& meas : dataset) {
    HandleLidarMeasurement(meas, map);
  }
}

void KalmanFilter::HandleLidarMeasurement(const LidarMeasurement& meas,
                                          const BeaconMap& map) {
  if (IsInitialised()) {
    VectorXd state = GetState();
    MatrixXd cov = GetCovariance();

    // Kalman Filter Update Step for the Lidar Measurements
    // use the WrapAngle() function on angular values to always keep angle
    // values within correct range, otherwise strange angle effects might be
    // seen.
    // The mapped-matched beacon position can be accessed by the variables
    // map_beacon.x and map_beacon.y

    BeaconData map_beacon = map.GetBeaconWithId(
        meas.id);  // Match Beacon with built in Data Association Id
    if (meas.id != -1 && map_beacon.id != -1) {
      // Measurement Vector
      VectorXd z = Vector2d::Zero();
      z << meas.range, meas.theta;

      // Predicted Measurement Vector (Measurement Model)
      VectorXd z_hat = Vector2d::Zero();
      double delta_x = map_beacon.x - state[0];
      double delta_y = map_beacon.y - state[1];
      double zhat_range = sqrt(delta_x * delta_x + delta_y * delta_y);
      double zhat_theta = WrapAngle(atan2(delta_y, delta_x) - state[2]);
      z_hat << zhat_range, zhat_theta;

      // Measurement Model Sensitivity Matrix
      MatrixXd H = MatrixXd(2, 4);
      H << -delta_x / zhat_range, -delta_y / zhat_range, 0, 0,
          delta_y / zhat_range / zhat_range, -delta_x / zhat_range / zhat_range,
          -1, 0;

      // Generate Measurement Model Noise Covariance Matrix
      MatrixXd R = Matrix2d::Zero();
      R(0, 0) = LIDAR_RANGE_STD * LIDAR_RANGE_STD;
      R(1, 1) = LIDAR_THETA_STD * LIDAR_THETA_STD;

      VectorXd y = z - z_hat;
      MatrixXd S = H * cov * H.transpose() + R;
      MatrixXd K = cov * H.transpose() * S.inverse();

      y(1) = WrapAngle(y(1));  // Wrap the Heading Innovation

      state = state + K * y;
      cov = (Matrix4d::Identity() - K * H) * cov;
    }

    SetState(state);
    SetCovariance(cov);
  }
}

void KalmanFilter::PredictionStep(const GyroMeasurement& gyro, double dt) {
  if (IsInitialised()) {
    VectorXd state = GetState();
    MatrixXd cov = GetCovariance();

    // Kalman Filter Prediction Step
    // Assume the state vector has the form [PX, PY, PSI, v].
    // Use the Gyroscope measurement as an input into the prediction step.
    // use the WrapAngle() function on angular values to always keep angle
    // values within correct range, otherwise strange angle effects might be
    // seen.

    double x = state(0);
    double y = state(1);
    double psi = state(2);
    double v = state(3);

    // Update State
    double x_new = x + dt * v * cos(psi);
    double y_new = y + dt * v * sin(psi);
    double psi_new = WrapAngle(psi + dt * gyro.psi_dot);
    double V_new = v;
    state << x_new, y_new, psi_new, V_new;

    // Generate F Matrix
    MatrixXd F = Matrix4d::Zero();
    F << 1, 0, -dt * v * sin(psi), dt * cos(psi), 0, 1, dt * v * cos(psi),
        dt * sin(psi), 0, 0, 1, 0, 0, 0, 0, 1;

    // Generate Q Matrix
    MatrixXd Q = Matrix4d::Zero();
    Q(2, 2) = dt * dt * GYRO_STD * GYRO_STD;
    Q(3, 3) = dt * dt * ACCEL_STD * ACCEL_STD;

    cov = F * cov * F.transpose() + Q;

    SetState(state);
    SetCovariance(cov);
  }
}

void KalmanFilter::HandleGPSMeasurement(const GPSMeasurement& meas) {
  // All this code is the same as the LKF as the measurement model is linear
  // so the UKF update state would just produce the same result.
  if (IsInitialised()) {
    VectorXd state = GetState();
    MatrixXd cov = GetCovariance();

    VectorXd z = Vector2d::Zero();
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
    cov = (Matrix4d::Identity() - K * H) * cov;

    SetState(state);
    SetCovariance(cov);
  } else {
    VectorXd state = Vector4d::Zero();
    MatrixXd cov = Matrix4d::Zero();

    state(0) = meas.x;
    state(1) = meas.y;
    cov(0, 0) = GPS_POS_STD * GPS_POS_STD;
    cov(1, 1) = GPS_POS_STD * GPS_POS_STD;
    cov(2, 2) = INIT_PSI_STD * INIT_PSI_STD;
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
    VectorXd state = GetState();  // STATE VECTOR [X,Y,PSI,v,...]
    return RobotState(state[0], state[1], state[2], state[3]);
  }
  return RobotState();
}

void KalmanFilter::PredictionStep(double dt) {}
}  // namespace pllee4
