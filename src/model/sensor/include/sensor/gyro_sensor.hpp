#ifndef GYRO_SENSOR_HPP
#define GYRO_SENSOR_HPP

#include <random>

namespace pllee4 {
struct GyroMeasurement {
  double psi_dot;
};

class GyroSensor {
 public:
  GyroSensor();
  void Reset();
  void SetGyroNoiseStd(double std);
  void SetGyroBias(double bias);
  GyroMeasurement GenerateGyroMeasurement(double sensor_yaw_rate);

 private:
  std::mt19937 rand_gen_;
  double noise_std_;
  double m_bias;
};
}  // namespace pllee4
#endif /* GYRO_SENSOR_HPP */
