/* 
 * gps_sensor.hpp
 * Created on: Feb 16, 2022 22:25
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef GPS_SENSOR_HPP
#define GPS_SENSOR_HPP

#include <random>

namespace pllee4 {
struct GPSMeasurement {
  double x, y;
};

class GPSSensor {
 public:
  GPSSensor();
  void Reset();
  void SetGPSNoiseStd(double std);
  void SetGPSErrorProb(double prob);
  void SetGPSDeniedZone(double x, double y, double r);
  GPSMeasurement GenerateGPSMeasurement(double sensor_x, double sensor_y);

 private:
  std::mt19937 rand_gen_;
  double noise_std_;
  double error_prob_;
  double gps_denied_x_, gps_denied_y_, gps_denied_range_;
};
}  // namespace pllee4
#endif /* GPS_SENSOR_HPP */
