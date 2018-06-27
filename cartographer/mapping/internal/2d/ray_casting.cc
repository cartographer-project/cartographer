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

#include "cartographer/mapping/internal/2d/ray_casting.h"

namespace cartographer {
namespace mapping {

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
std::vector<Eigen::Array2i> CastRay(const Eigen::Array2i& begin,
                                    const Eigen::Array2i& end,
                                    int subpixel_scale) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  if (begin.x() > end.x()) {
    return CastRay(end, begin, subpixel_scale);
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);
  std::vector<Eigen::Array2i> ray;
  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  if (begin.x() / subpixel_scale == end.x() / subpixel_scale) {
    Eigen::Array2i current(begin.x() / subpixel_scale,
                           std::min(begin.y(), end.y()) / subpixel_scale);
    ray.push_back(current);
    const int end_y = std::max(begin.y(), end.y()) / subpixel_scale;
    for (; current.y() <= end_y; ++current.y()) {
      if (!ray.back().isApprox(current)) ray.push_back(current);
    }
    return ray;
  }

  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  const int64 denominator = 2 * subpixel_scale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / subpixel_scale;
  ray.push_back(current);

  // To represent subpixel centers, we use a factor of 2 * 'subpixel_scale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * subpixel_scale) / (2 * subpixel_scale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * subpixel_scale)
  // | | | | -- center of first subpixel = 1 / (2 * subpixel_scale)
  // +-+-+-+ -- 0 = 0 / (2 * subpixel_scale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (begin.y() % subpixel_scale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'subpixel_scale'.
  const int first_pixel =
      2 * subpixel_scale - 2 * (begin.x() % subpixel_scale) - 1;
  // The same from the left pixel border to 'end'.
  const int last_pixel = 2 * (end.x() % subpixel_scale) + 1;

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / subpixel_scale;

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      if (!ray.back().isApprox(current)) ray.push_back(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        if (!ray.back().isApprox(current)) ray.push_back(current);
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
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    if (!ray.back().isApprox(current)) ray.push_back(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      if (!ray.back().isApprox(current)) ray.push_back(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / subpixel_scale);
    return ray;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    if (!ray.back().isApprox(current)) ray.push_back(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      if (!ray.back().isApprox(current)) ray.push_back(current);
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
  if (!ray.back().isApprox(current)) ray.push_back(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    if (!ray.back().isApprox(current)) ray.push_back(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / subpixel_scale);
  return ray;
}

}  // namespace mapping
}  // namespace cartographer
