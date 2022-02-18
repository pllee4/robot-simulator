/*
 * kalman_filter.hpp
 *
 * Created on: Feb 18, 2022 21:40
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <vector>

#include "robot/robot.hpp"

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

namespace pllee4 {

class KalmanFilterBase {
 public:
  KalmanFilterBase() = default;
  virtual ~KalmanFilterBase() {}
  void Reset() { initialised_ = false; }
  bool IsInitialised() const { return initialised_; }

 protected:
  VectorXd GetState() const { return state_; }
  MatrixXd GetCovariance() const { return covariance_; }
  void SetState(const VectorXd& state) {
    state_ = state;
    initialised_ = true;
  }
  void SetCovariance(const MatrixXd& cov) { covariance_ = cov; }

 private:
  bool initialised_{false};
  VectorXd state_;
  MatrixXd covariance_;
};

class GyroMeasurement;
class LidarMeasurement;
class BeaconMap;
class GPSMeasurement;

class KalmanFilter : public KalmanFilterBase {
 public:
  RobotState GetRobotState();
  Matrix2d GetRobotStatePositionCovariance();

  void PredictionStep(double dt);
  void PredictionStep(const GyroMeasurement& gyro, double dt);
  void HandleLidarMeasurements(const std::vector<LidarMeasurement>& meas,
                               const BeaconMap& map);
  void HandleLidarMeasurement(const LidarMeasurement& meas,
                              const BeaconMap& map);
  void HandleGPSMeasurement(const GPSMeasurement& meas);
};
}  // namespace pllee4
#endif /* KALMAN_FILTER_HPP */
