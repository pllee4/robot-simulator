#ifndef LIDAR_SENSOR_HPP
#define LIDAR_SENSOR_HPP

#include <random>
#include <vector>

namespace pllee4 {
struct LidarMeasurement {
  double range, theta;
  int id;
};

class BeaconMap;

class LidarSensor {
 public:
  LidarSensor();
  void Reset();
  void SetLidarNoiseStd(double range_std, double theta_std);
  void SetLidarMaxRange(double range);
  void SetLidarDAEnabled(bool id_enabled);
  std::vector<LidarMeasurement> GenerateLidarMeasurements(double sensor_x,
                                                          double sensor_y,
                                                          double sensor_yaw,
                                                          const BeaconMap& map);

 private:
  std::mt19937 rand_gen_;
  double range_noise_std_;
  double theta_noise_std_;
  double max_range_;
  bool id_enabled_;
};
}  // namespace pllee4
#endif /* LIDAR_SENSOR_HPP */
