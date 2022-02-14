/*
 * beacon.cpp
 * Created on: Feb 14, 2022 22:53
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "beacon/beacon.hpp"

#include <cmath>
#include <random>

#include "display/display.hpp"
#include "helper/details/transformation.hpp"

namespace pllee4 {
namespace model {
BeaconMap::BeaconMap() {
  std::mt19937 rand_gen;
  std::uniform_real_distribution<double> pos_dis(-500.0, 500.0);
  for (int i = 0; i < 200; i++) {
    AddBeacon(pos_dis(rand_gen), pos_dis(rand_gen));
  }
}

void BeaconMap::AddBeacon(double x, double y) {
  beacon_map_.push_back(BeaconData(x, y, beacon_map_.size()));
}

BeaconData BeaconMap::GetBeaconWithId(int id) const {
  for (const BeaconData& beacon : beacon_map_) {
    if (beacon.id == id) {
      return beacon;
    }
  }
  return BeaconData();
}

std::vector<BeaconData> BeaconMap::GetBeaconsWithinRange(double x, double y,
                                                         double range) const {
  std::vector<BeaconData> beacons;
  for (const BeaconData& beacon : beacon_map_) {
    auto delta_x = beacon.x - x;
    auto delta_y = beacon.y - y;
    auto beacon_range = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (beacon_range < range) {
      beacons.push_back(beacon);
    }
  }
  return beacons;
}

std::vector<BeaconData> BeaconMap::GetBeacons() const { return beacon_map_; }

void BeaconMap::Render(Display& disp) const {
  const std::vector<Vector2> beacon_lines = {{1, 0}, {0, 1}, {0, -1}, {1, 0}};
  disp.SetDrawColour(255, 255, 0);
  for (const auto& beacon : beacon_map_) {
    disp.DrawLines(OffsetPoints(beacon_lines, Vector2(beacon.x, beacon.y)));
  }
}
}  // namespace model
}  // namespace pllee4