/*
 * simulation.hpp
 *
 * Created on: Feb 18, 2022 22:53
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <memory>
#include <vector>

#include "beacon/beacon.hpp"
#include "kalman_filter/kalman_filter.hpp"
#include "robot/robot.hpp"
#include "sensor/gps_sensor.hpp"
#include "sensor/gyro_sensor.hpp"
#include "sensor/lidar_sensor.hpp"

namespace pllee4 {
struct SimulationParams {
  std::string profile_name;
  double time_step{0.1};
  double end_time{120};

  bool gps_enabled{true};
  double gps_update_rate{1.0};
  double gps_position_noise_std{3};
  double gps_error_probability{0.0};
  double gps_denied_x{0.0};
  double gps_denied_y{0.0};
  double gps_denied_range{-1.0};

  bool lidar_enabled{false};
  bool lidar_id_enabled{true};
  double lidar_update_rate{10.0};
  double lidar_range_noise_std{0.001};
  double lidar_theta_noise_std{0.0};

  bool gyro_enabled{true};
  double gyro_update_rate{10.0};
  double gyro_noise_std{0.001};
  double gyro_bias{0.0};

  double robot_initial_x{0.0};
  double robot_initial_y{0.0};
  double robot_initial_psi{0.0};
  double robot_initial_velocity{5.0};

  std::vector<std::shared_ptr<MotionCommandBase>> robot_commands;

  SimulationParams() = default;
};

class Display;

class Simulation {
 public:
  Simulation() = default;
  void Reset();
  void Reset(SimulationParams sim_params);
  void Update();
  void Render(Display &disp);
  void IncreaseTimeMultiplier();
  void DecreaseTimeMultiplier();
  void SetTimeMultiplier(unsigned int multiplier);
  void IncreaseZoom();
  void DecreaseZoom();
  void TogglePauseSimulation();
  bool IsPaused();
  bool IsRunning();

 private:
  SimulationParams sim_parameters_;
  KalmanFilter kalman_filter_;
  Robot robot_;
  BeaconMap beacons_;
  GyroSensor gyro_sensor_;
  GPSSensor gps_sensor_;
  LidarSensor lidar_sensor_;

  bool is_paused_{false};
  bool is_running_{false};
  int time_multiplier_{1};
  double view_size_{100};

  double time_{0.0};
  double time_till_gyro_measurement_{0.0};
  double time_till_gps_measurement_{0.0};
  double time_till_lidar_measurement_{0.0};

  std::vector<GPSMeasurement> gps_measurement_history_;
  std::vector<LidarMeasurement> lidar_measurement_history_;

  std::vector<Vector2> vehicle_position_history_;
  std::vector<Vector2> filter_position_history_;

  std::vector<double> filter_error_x_position_history_;
  std::vector<double> filter_error_y_position_history_;
  std::vector<double> filter_error_heading_history_;
  std::vector<double> filter_error_velocity_history_;
};
}  // namespace pllee4
#endif /* SIMULATION_HPP */
