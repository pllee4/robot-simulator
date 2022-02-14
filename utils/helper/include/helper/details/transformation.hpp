/* 
 * transformation.hpp
 * Created on: Feb 13, 2022 17:56
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <vector>

#include "types/types.hpp"

std::vector<Vector2> TransformPoints(const std::vector<Vector2>& points,
                                     const Vector2& position,
                                     const double rotation);
std::vector<std::vector<Vector2>> TransformPoints(
    const std::vector<std::vector<Vector2>>& dataset, const Vector2& position,
    const double rotation);
std::vector<Vector2> OffsetPoints(const std::vector<Vector2>& points,
                                  const Vector2& offset);
std::vector<std::vector<Vector2>> OffsetPoints(
    const std::vector<std::vector<Vector2>>& dataset, const Vector2& offset);

#endif /* TRANSFORMATION_HPP */
