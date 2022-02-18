/*
 * simulation.cpp
 *
 * Created on: Feb 18, 2022 22:53
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "simulation/simulation.hpp"

#include "display/display.hpp"
#include "helper/details/transformation.hpp"
#include "helper/helper.hpp"

namespace pllee4 {
void Simulation::Reset() {
  // Reset Simulation
  time_ = 0.0;
  time_till_gyro_measurement_ = 0.0;
  time_till_gps_measurement_ = 0.0;
  time_till_lidar_measurement_ = 0.0;

  is_running_ = true;
  is_paused_ = false;

  kalman_filter_.Reset();

  gps_sensor_.Reset();
  gps_sensor_.SetGPSNoiseStd(sim_parameters_.gps_position_noise_std);
  gps_sensor_.SetGPSErrorProb(sim_parameters_.gps_error_probability);
  gps_sensor_.SetGPSDeniedZone(sim_parameters_.gps_denied_x,
                               sim_parameters_.gps_denied_y,
                               sim_parameters_.gps_denied_range);

  gyro_sensor_.Reset();
  gyro_sensor_.SetGyroNoiseStd(sim_parameters_.gyro_noise_std);
  gyro_sensor_.SetGyroBias(sim_parameters_.gyro_bias);

  lidar_sensor_.Reset();
  lidar_sensor_.SetLidarNoiseStd(sim_parameters_.lidar_range_noise_std,
                                 sim_parameters_.lidar_theta_noise_std);
  lidar_sensor_.SetLidarDAEnabled(sim_parameters_.lidar_id_enabled);

  robot_.Reset(sim_parameters_.robot_initial_x, sim_parameters_.robot_initial_y,
               sim_parameters_.robot_initial_psi,
               sim_parameters_.robot_initial_velocity);

  for (auto& cmd : sim_parameters_.robot_commands) {
    robot_.AddVehicleCommand(cmd.get());
  }

  // Plotting Variables
  gps_measurement_history_.clear();
  lidar_measurement_history_.clear();
  vehicle_position_history_.clear();
  filter_position_history_.clear();

  // Stats Variables
  filter_error_x_position_history_.clear();
  filter_error_y_position_history_.clear();
  filter_error_heading_history_.clear();
  filter_error_velocity_history_.clear();

  std::cout << "Simulation: Reset" << std::endl;
}

void Simulation::Update() {
  if (is_running_ && !is_paused_) {
    // Time Multiplier
    for (unsigned i = 0; i < time_multiplier_; ++i) {
      // Check for End Time
      if (time_ >= sim_parameters_.end_time) {
        is_running_ = false;
        std::cout << "Simulation: Reached End of Simulation Time (" << time_
                  << ")" << std::endl;
        return;
      }

      // Update Motion
      robot_.Update(time_, sim_parameters_.time_step);
      vehicle_position_history_.push_back(
          Vector2(robot_.GetRobotState().x, robot_.GetRobotState().y));

      // Gyro Measurement / Prediction Step
      if (sim_parameters_.gyro_enabled) {
        if (time_till_gyro_measurement_ <= 0) {
          GyroMeasurement meas = gyro_sensor_.GenerateGyroMeasurement(
              robot_.GetRobotState().yaw_rate);
          kalman_filter_.PredictionStep(meas, sim_parameters_.time_step);
          time_till_gyro_measurement_ += 1.0 / sim_parameters_.gyro_update_rate;
        }
        time_till_gyro_measurement_ -= sim_parameters_.time_step;
      }

      // GPS Measurement
      if (sim_parameters_.gps_enabled) {
        if (time_till_gps_measurement_ <= 0) {
          GPSMeasurement gps_meas = gps_sensor_.GenerateGPSMeasurement(
              robot_.GetRobotState().x, robot_.GetRobotState().y);
          kalman_filter_.HandleGPSMeasurement(gps_meas);
          gps_measurement_history_.push_back(gps_meas);
          time_till_gps_measurement_ += 1.0 / sim_parameters_.gps_update_rate;
        }
        time_till_gps_measurement_ -= sim_parameters_.time_step;
      }

      // Lidar Measurement
      if (sim_parameters_.lidar_enabled) {
        if (time_till_lidar_measurement_ <= 0) {
          std::vector<LidarMeasurement> lidar_measurements =
              lidar_sensor_.GenerateLidarMeasurements(
                  robot_.GetRobotState().x, robot_.GetRobotState().y,
                  robot_.GetRobotState().psi, beacons_);
          kalman_filter_.HandleLidarMeasurements(lidar_measurements, beacons_);
          lidar_measurement_history_ = lidar_measurements;
          time_till_lidar_measurement_ +=
              1.0 / sim_parameters_.lidar_update_rate;
        }
        time_till_lidar_measurement_ -= sim_parameters_.time_step;
      }

      // Save Filter History and Calculate Stats
      if (kalman_filter_.IsInitialised()) {
        RobotState vehicle_state = robot_.GetRobotState();
        RobotState filter_state = kalman_filter_.GetRobotState();
        filter_position_history_.push_back(
            Vector2(filter_state.x, filter_state.y));
        filter_error_x_position_history_.push_back(filter_state.x -
                                                   vehicle_state.x);
        filter_error_y_position_history_.push_back(filter_state.y -
                                                   vehicle_state.y);
        filter_error_heading_history_.push_back(
            WrapAngle(filter_state.psi - vehicle_state.psi));
        filter_error_velocity_history_.push_back(filter_state.v -
                                                 vehicle_state.v);
      }

      // Update Time
      time_ += sim_parameters_.time_step;
    }
  }
}

void Simulation::Render(Display& disp) {
  std::vector<Vector2> marker_lines1 = {{0.5, 0.5}, {-0.5, -0.5}};
  std::vector<Vector2> marker_lines2 = {{0.5, -0.5}, {-0.5, 0.5}};

  disp.SetView(view_size_ * disp.GetScreenAspectRatio(), view_size_,
               robot_.GetRobotState().x, robot_.GetRobotState().y);

  robot_.Render(disp);
  beacons_.Render(disp);

  disp.SetDrawColour(0, 100, 0);
  disp.DrawLines(vehicle_position_history_);

  disp.SetDrawColour(100, 0, 0);
  disp.DrawLines(filter_position_history_);

  if (kalman_filter_.IsInitialised()) {
    RobotState filter_state = kalman_filter_.GetRobotState();
    Eigen::Matrix2d cov = kalman_filter_.GetRobotStatePositionCovariance();

    double x = filter_state.x;
    double y = filter_state.y;
    double sigma_xx = cov(0, 0);
    double sigma_yy = cov(1, 1);
    double sigma_xy = cov(0, 1);

    std::vector<Vector2> marker_lines1_world =
        OffsetPoints(marker_lines1, Vector2(x, y));
    std::vector<Vector2> marker_lines2_world =
        OffsetPoints(marker_lines2, Vector2(x, y));
    disp.SetDrawColour(255, 0, 0);
    disp.DrawLines(marker_lines1_world);
    disp.DrawLines(marker_lines2_world);

    std::vector<Vector2> cov_world =
        GenerateEllipse(x, y, sigma_xx, sigma_yy, sigma_xy);
    disp.SetDrawColour(255, 0, 0);
    disp.DrawLines(cov_world);
  }

  // Render GPS Measurements
  std::vector<std::vector<Vector2>> m_gps_marker = {{{0.5, 0.5}, {-0.5, -0.5}},
                                                    {{0.5, -0.5}, {-0.5, 0.5}}};
  disp.SetDrawColour(255, 255, 255);
  for (const auto& meas : gps_measurement_history_) {
    disp.DrawLines(OffsetPoints(m_gps_marker, Vector2(meas.x, meas.y)));
  }

  // Render GPS Denied Zone
  if (sim_parameters_.gps_denied_range > 0) {
    std::vector<Vector2> zone_lines = GenerateCircle(
        sim_parameters_.gps_denied_x, sim_parameters_.gps_denied_y,
        sim_parameters_.gps_denied_range);
    disp.SetDrawColour(255, 150, 0);
    disp.DrawLines(zone_lines);
  }

  // Render Lidar Measurements
  for (const auto& meas : lidar_measurement_history_) {
    double x0 = robot_.GetRobotState().x;
    double y0 = robot_.GetRobotState().y;
    double delta_x = meas.range * cos(meas.theta + robot_.GetRobotState().psi);
    double delta_y = meas.range * sin(meas.theta + robot_.GetRobotState().psi);
    disp.SetDrawColour(201, 201, 0);
    disp.DrawLine(Vector2(x0, y0), Vector2(x0 + delta_x, y0 + delta_y));
  }

  int x_offset, y_offset;
  int stride = 20;
  // Simulation Status / Parameters
  x_offset = 10;
  y_offset = 30;
  std::string time_string =
      string_format("Time: %0.2f (x%d)", time_, time_multiplier_);
  std::string profile_string =
      string_format("Profile: %s", sim_parameters_.profile_name.c_str());
  std::string gps_string = string_format(
      "GPS: %s (%0.1f Hz)", (sim_parameters_.gps_enabled ? "ON" : "OFF"),
      sim_parameters_.gps_update_rate);
  std::string lidar_string = string_format(
      "LIDAR: %s (%0.1f Hz)", (sim_parameters_.lidar_enabled ? "ON" : "OFF"),
      sim_parameters_.lidar_update_rate);
  std::string gyro_string = string_format(
      "GYRO: %s (%0.1f Hz)", (sim_parameters_.gyro_enabled ? "ON" : "OFF"),
      sim_parameters_.gyro_update_rate);
  disp.DrawText(profile_string, Vector2(x_offset, y_offset + stride * -1), 1.0,
                {255, 255, 255});
  disp.DrawText(time_string, Vector2(x_offset, y_offset + stride * 0), 1.0,
                {255, 255, 255});
  disp.DrawText(gps_string, Vector2(x_offset, y_offset + stride * 1), 1.0,
                {255, 255, 255});
  disp.DrawText(lidar_string, Vector2(x_offset, y_offset + stride * 2), 1.0,
                {255, 255, 255});
  disp.DrawText(gyro_string, Vector2(x_offset, y_offset + stride * 3), 1.0,
                {255, 255, 255});
  if (is_paused_) {
    disp.DrawText("PAUSED", Vector2(x_offset, y_offset + stride * 4), 1.0,
                  {255, 0, 0});
  }
  if (!is_running_) {
    disp.DrawText("FINISHED", Vector2(x_offset, y_offset + stride * 5), 1.0,
                  {255, 0, 0});
  }

  // Vehicle State
  x_offset = 800;
  y_offset = 10;
  std::string velocity_string =
      string_format("    Velocity: %0.2f m/s", robot_.GetRobotState().v);
  std::string yaw_string = string_format(
      "   Heading: %0.2f deg", robot_.GetRobotState().psi * 180.0 / M_PI);
  std::string x_pos =
      string_format("X Position: %0.2f m", robot_.GetRobotState().x);
  std::string y_pos =
      string_format("Y Position: %0.2f m", robot_.GetRobotState().y);
  disp.DrawText("Vehicle State", Vector2(x_offset - 5, y_offset + stride * 0),
                1.0, {255, 255, 255});
  disp.DrawText(velocity_string, Vector2(x_offset, y_offset + stride * 1), 1.0,
                {255, 255, 255});
  disp.DrawText(yaw_string, Vector2(x_offset, y_offset + stride * 2), 1.0,
                {255, 255, 255});
  disp.DrawText(x_pos, Vector2(x_offset, y_offset + stride * 3), 1.0,
                {255, 255, 255});
  disp.DrawText(y_pos, Vector2(x_offset, y_offset + stride * 4), 1.0,
                {255, 255, 255});

  std::string kf_velocity_string = string_format(
      "    Velocity: %0.2f m/s", kalman_filter_.GetRobotState().v);
  std::string kf_yaw_string =
      string_format("   Heading: %0.2f deg",
                    kalman_filter_.GetRobotState().psi * 180.0 / M_PI);
  std::string kf_x_pos =
      string_format("X Position: %0.2f m", kalman_filter_.GetRobotState().x);
  std::string kf_y_pos =
      string_format("Y Position: %0.2f m", kalman_filter_.GetRobotState().y);
  disp.DrawText("Filter State", Vector2(x_offset, y_offset + stride * 6), 1.0,
                {255, 255, 255});
  disp.DrawText(kf_velocity_string, Vector2(x_offset, y_offset + stride * 7),
                1.0, {255, 255, 255});
  disp.DrawText(kf_yaw_string, Vector2(x_offset, y_offset + stride * 8), 1.0,
                {255, 255, 255});
  disp.DrawText(kf_x_pos, Vector2(x_offset, y_offset + stride * 9), 1.0,
                {255, 255, 255});
  disp.DrawText(kf_y_pos, Vector2(x_offset, y_offset + stride * 10), 1.0,
                {255, 255, 255});

  // Keyboard Input
  x_offset = 10;
  y_offset = 650;
  disp.DrawText("Reset Key: r", Vector2(x_offset, y_offset + stride * 0), 1.0,
                {255, 255, 255});
  disp.DrawText("Pause Key: [space bar]",
                Vector2(x_offset, y_offset + stride * 1), 1.0, {255, 255, 255});
  disp.DrawText("Speed Multiplier (+/-) Key: [ / ] ",
                Vector2(x_offset, y_offset + stride * 2), 1.0, {255, 255, 255});
  disp.DrawText("Zoom (+/-) Key: + / - (keypad)",
                Vector2(x_offset, y_offset + stride * 3), 1.0, {255, 255, 255});
  disp.DrawText("Motion Profile Key: 1 - 9,0",
                Vector2(x_offset, y_offset + stride * 4), 1.0, {255, 255, 255});

  // Filter Error State
  x_offset = 750;
  y_offset = 650;
  std::string x_pos_error_string =
      string_format("X Position RMSE: %0.2f m",
                    CalculateRMSE(filter_error_x_position_history_));
  std::string y_pos_error_string =
      string_format("Y Position RMSE: %0.2f m",
                    CalculateRMSE(filter_error_y_position_history_));
  std::string heading_error_string = string_format(
      "   Heading RMSE: %0.2f deg",
      180.0 / M_PI * CalculateRMSE(filter_error_heading_history_));
  std::string velocity_error_string =
      string_format("    Velocity RMSE: %0.2f m/s",
                    CalculateRMSE(filter_error_velocity_history_));
  disp.DrawText(x_pos_error_string, Vector2(x_offset, y_offset + stride * 0),
                1.0, {255, 255, 255});
  disp.DrawText(y_pos_error_string, Vector2(x_offset, y_offset + stride * 1),
                1.0, {255, 255, 255});
  disp.DrawText(heading_error_string, Vector2(x_offset, y_offset + stride * 2),
                1.0, {255, 255, 255});
  disp.DrawText(velocity_error_string, Vector2(x_offset, y_offset + stride * 3),
                1.0, {255, 255, 255});
}

void Simulation::Reset(SimulationParams sim_params) {
  sim_parameters_ = sim_params;
  Reset();
}
void Simulation::IncreaseTimeMultiplier() {
  time_multiplier_++;
  std::cout << "Simulation: Time Multiplier Increased (x" << time_multiplier_
            << ")" << std::endl;
}
void Simulation::DecreaseTimeMultiplier() {
  if (time_multiplier_ > 1) {
    time_multiplier_--;
    std::cout << "Simulation: Time Multiplier Decreased (x" << time_multiplier_
              << ")" << std::endl;
  }
}
void Simulation::SetTimeMultiplier(unsigned int multiplier) {
  time_multiplier_ = static_cast<int>(multiplier);
}
void Simulation::IncreaseZoom() {
  if (view_size_ > 25) {
    view_size_ -= 25;
  }
  std::cout << "Simulation: Zoom Increased (" << view_size_ << "m)"
            << std::endl;
}
void Simulation::DecreaseZoom() {
  if (view_size_ < 400) {
    view_size_ += 25;
  }
  std::cout << "Simulation: Zoom Decreased (" << view_size_ << "m)"
            << std::endl;
}
void Simulation::TogglePauseSimulation() {
  is_paused_ = !is_paused_;
  std::cout << "Simulation: Paused (" << (is_paused_ ? "True" : "False") << ")"
            << std::endl;
}
bool Simulation::IsPaused() { return is_paused_; }
bool Simulation::IsRunning() { return is_running_; }
}  // namespace pllee4