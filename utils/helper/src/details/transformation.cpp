/*
 * transformation.cpp
 * Created on: Feb 13, 2022 17:57
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "helper/details/transformation.hpp"

#include <cmath>

std::vector<Vector2> TransformPoints(const std::vector<Vector2>& points,
                                     const Vector2& position,
                                     const double rotation) {
  double ctheta = cos(rotation);
  double stheta = sin(rotation);
  std::vector<Vector2> transformedPoints;
  for (const Vector2& point : points) {
    double x = point.x * ctheta - stheta * point.y + position.x;
    double y = point.x * stheta + ctheta * point.y + position.y;
    transformedPoints.push_back(Vector2(x, y));
  }
  return transformedPoints;
}

std::vector<std::vector<Vector2>> TransformPoints(
    const std::vector<std::vector<Vector2>>& dataset, const Vector2& position,
    const double rotation) {
  std::vector<std::vector<Vector2>> transformedDataset;
  for (const std::vector<Vector2> points : dataset) {
    transformedDataset.push_back(TransformPoints(points, position, rotation));
  }
  return transformedDataset;
}

std::vector<Vector2> OffsetPoints(const std::vector<Vector2>& points,
                                  const Vector2& offset) {
  std::vector<Vector2> transformedPoints;
  for (const Vector2& point : points) {
    transformedPoints.push_back(
        Vector2(point.x + offset.x, point.y + offset.y));
  }
  return transformedPoints;
}

std::vector<std::vector<Vector2>> OffsetPoints(
    const std::vector<std::vector<Vector2>>& dataset, const Vector2& offset) {
  std::vector<std::vector<Vector2>> transformedDataset;
  for (const std::vector<Vector2> points : dataset) {
    transformedDataset.push_back(OffsetPoints(points, offset));
  }
  return transformedDataset;
}