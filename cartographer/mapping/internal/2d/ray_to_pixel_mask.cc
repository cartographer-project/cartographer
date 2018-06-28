/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"

#include "Eigen/Dense"

namespace cartographer {
namespace mapping {
namespace {

bool isEqual(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
  return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}
}  // namespace

// Compute all pixels that contain some part of the line segment connecting
// 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
// by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
// greater than zero. Return values are in pixels and not scaled.
std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i& scaled_begin,
                                           const Eigen::Array2i& scaled_end,
                                           int subpixel_scale) {
  // For simplicity, we order 'scaled_begin' and 'scaled_end' by their x
  // coordinate.
  if (scaled_begin.x() > scaled_end.x()) {
    return RayToPixelMask(scaled_end, scaled_begin, subpixel_scale);
  }

  CHECK_GE(scaled_begin.x(), 0);
  CHECK_GE(scaled_begin.y(), 0);
  CHECK_GE(scaled_end.y(), 0);
  std::vector<Eigen::Array2i> pixel_mask;
  // Special case: We have to draw a vertical line in full pixels, as
  // 'scaled_begin' and 'scaled_end' have the same full pixel x coordinate.
  if (scaled_begin.x() / subpixel_scale == scaled_end.x() / subpixel_scale) {
    Eigen::Array2i current(
        scaled_begin.x() / subpixel_scale,
        std::min(scaled_begin.y(), scaled_end.y()) / subpixel_scale);
    pixel_mask.push_back(current);
    const int end_y =
        std::max(scaled_begin.y(), scaled_end.y()) / subpixel_scale;
    for (; current.y() <= end_y; ++current.y()) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    return pixel_mask;
  }

  const int64 dx = scaled_end.x() - scaled_begin.x();
  const int64 dy = scaled_end.y() - scaled_begin.y();
  const int64 denominator = 2 * subpixel_scale * dx;

  // The current full pixel coordinates. We scaled_begin at 'scaled_begin'.
  Eigen::Array2i current = scaled_begin / subpixel_scale;
  pixel_mask.push_back(current);

  // To represent subpixel centers, we use a factor of 2 * 'subpixel_scale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * subpixel_scale) / (2 * subpixel_scale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * subpixel_scale)
  // | | | | -- center of first subpixel = 1 / (2 * subpixel_scale)
  // +-+-+-+ -- 0 = 0 / (2 * subpixel_scale)

  // The center of the subpixel part of 'scaled_begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (scaled_begin.y() % subpixel_scale) + 1) * dx;

  // The distance from the from 'scaled_begin' to the right pixel border, to be
  // divided by 2 * 'subpixel_scale'.
  const int first_pixel =
      2 * subpixel_scale - 2 * (scaled_begin.x() % subpixel_scale) - 1;
  // The same from the left pixel border to 'scaled_end'.
  const int last_pixel = 2 * (scaled_end.x() % subpixel_scale) + 1;

  // The full pixel x coordinate of 'scaled_end'.
  const int end_x = std::max(scaled_begin.x(), scaled_end.x()) / subpixel_scale;

  // Move from 'scaled_begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * subpixel_scale;
    }
    // Move from the pixel border on the right to 'scaled_end'.
    sub_y += dy * last_pixel;
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
    return pixel_mask;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * subpixel_scale;
  }
  sub_y += dy * last_pixel;
  if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
  return pixel_mask;
}

}  // namespace mapping
}  // namespace cartographer
