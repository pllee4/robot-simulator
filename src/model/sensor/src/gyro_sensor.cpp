#include "sensor/gyro_sensor.hpp"

namespace pllee4 {
GyroSensor::GyroSensor()
    : rand_gen_(std::mt19937()), noise_std_(0.0), m_bias(0.0) {}

void GyroSensor::Reset() { rand_gen_ = std::mt19937(); }

void GyroSensor::SetGyroNoiseStd(double std) { noise_std_ = std; }

void GyroSensor::SetGyroBias(double bias) { m_bias = bias; }

GyroMeasurement GyroSensor::GenerateGyroMeasurement(double sensor_yaw_rate) {
  GyroMeasurement meas;
  std::normal_distribution<double> gyro_dis(0.0, noise_std_);
  meas.psi_dot = sensor_yaw_rate + m_bias + gyro_dis(rand_gen_);
  return meas;
}
}  // namespace pllee4