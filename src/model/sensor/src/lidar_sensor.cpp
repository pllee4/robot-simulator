#include "sensor/lidar_sensor.hpp"

#include "beacon/beacon.hpp"
#include "helper/helper.hpp"

namespace pllee4 {
LidarSensor::LidarSensor()
    : rand_gen_(std::mt19937()),
      range_noise_std_(0.0),
      theta_noise_std_(0.0),
      max_range_(90.0),
      id_enabled_(true) {}

void LidarSensor::Reset() { rand_gen_ = std::mt19937(); }

void LidarSensor::SetLidarNoiseStd(double range_std, double theta_std) {
  range_noise_std_ = range_std;
  theta_noise_std_ = theta_std;
}

void LidarSensor::SetLidarMaxRange(double range) { max_range_ = range; }

void LidarSensor::SetLidarDAEnabled(bool id_enabled) {
  id_enabled_ = id_enabled;
}

std::vector<LidarMeasurement> LidarSensor::GenerateLidarMeasurements(
    double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap& map) {
  std::vector<LidarMeasurement> meas;
  std::normal_distribution<double> lidar_theta_dis(0.0, theta_noise_std_);
  std::normal_distribution<double> lidar_range_dis(0.0, range_noise_std_);

  for (const auto& beacon : map.GetBeacons()) {
    double delta_x = beacon.x - sensor_x;
    double delta_y = beacon.y - sensor_y;
    double theta = WrapAngle(atan2(delta_y, delta_x) - sensor_yaw);
    double beacon_range = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (beacon_range < max_range_) {
      LidarMeasurement beacon_meas;
      beacon_meas.range = std::abs(beacon_range + lidar_range_dis(rand_gen_));
      beacon_meas.theta = WrapAngle(theta + lidar_theta_dis(rand_gen_));
      beacon_meas.id = (id_enabled_ ? beacon.id : -1);
      meas.push_back(beacon_meas);
    }
  }
  return meas;
}
}  // namespace pllee4