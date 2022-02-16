/*
 * beacon.hpp
 * Created on: Feb 14, 2022 22:52
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef BEACON_HPP
#define BEACON_HPP

#include <vector>

namespace pllee4 {
class Display;
struct BeaconData {
  double x{}, y{};
  int id{-1};
  BeaconData() = default;
  BeaconData(double x_pos, double y_pos) : x(x_pos), y(y_pos) {}
  BeaconData(double x_pos, double y_pos, int beacon_id)
      : x(x_pos), y(y_pos), id(beacon_id) {}
};

class BeaconMap {
 public:
  BeaconMap();

  void AddBeacon(double x, double y);

  BeaconData GetBeaconWithId(int id) const;
  std::vector<BeaconData> GetBeaconsWithinRange(double x, double y,
                                                double range) const;
  std::vector<BeaconData> GetBeacons() const;

  void Render(Display& disp) const;

 private:
  std::vector<BeaconData> beacon_map_;
};
}  // namespace pllee4
#endif /* BEACON_HPP */
