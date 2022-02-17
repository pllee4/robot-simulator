/* 
 * gps_sensor.cpp
 * Created on: Feb 16, 2022 22:25
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#include "sensor/gps_sensor.hpp"

namespace pllee4 {
GPSSensor::GPSSensor()
    : rand_gen_(std::mt19937()),
      noise_std_(0.0),
      error_prob_(0.0),
      gps_denied_x_(0.0),
      gps_denied_y_(0.0),
      gps_denied_range_(-1.0) {}

void GPSSensor::Reset() { rand_gen_ = std::mt19937(); }

void GPSSensor::SetGPSNoiseStd(double std) { noise_std_ = std; }

void GPSSensor::SetGPSErrorProb(double prob) { error_prob_ = prob; }

void GPSSensor::SetGPSDeniedZone(double x, double y, double r) {
  gps_denied_x_ = x;
  gps_denied_y_ = y;
  gps_denied_range_ = r;
}

GPSMeasurement GPSSensor::GenerateGPSMeasurement(double sensor_x,
                                                 double sensor_y) {
  GPSMeasurement meas;
  std::normal_distribution<double> gps_pos_dis(0.0, noise_std_);
  std::uniform_real_distribution<double> gps_error_dis(0.0, 1.0);
  meas.x = sensor_x + gps_pos_dis(rand_gen_);
  meas.y = sensor_y + gps_pos_dis(rand_gen_);
  if (gps_error_dis(rand_gen_) < error_prob_) {
    meas.x = 0;
    meas.y = 0;
  }
  double delta_x = sensor_x - gps_denied_x_;
  double delta_y = sensor_y - gps_denied_y_;
  double range = sqrt(delta_x * delta_x + delta_y * delta_y);
  if (range < gps_denied_range_) {
    meas.x = 0;
    meas.y = 0;
  }
  return meas;
}
}