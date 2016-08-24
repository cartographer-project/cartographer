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

#ifndef CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_
#define CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

// Iterates in row-major order through a range of xy-indices.
class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
 public:
  // Constructs a new iterator for the specified range. The range is cropped to
  // fit within the given 'cell_limits'.
  XYIndexRangeIterator(const CellLimits& cell_limits,
                       const Eigen::Array2i& min_xy_index,
                       const Eigen::Array2i& max_xy_index)
      : XYIndexRangeIterator(
            min_xy_index.max(Eigen::Array2i::Zero()),
            max_xy_index.min(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1))) {}

  // Constructs a new iterator for everything contained in 'cell_limits'.
  explicit XYIndexRangeIterator(const CellLimits& cell_limits)
      : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                             Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator(const XYIndexRangeIterator& other)
      : min_xy_index_(other.min_xy_index_),
        max_xy_index_(other.max_xy_index_),
        xy_index_(other.xy_index_) {}

  XYIndexRangeIterator& operator=(const XYIndexRangeIterator& other) {
    min_xy_index_ = other.min_xy_index_;
    max_xy_index_ = other.max_xy_index_;
    xy_index_ = other.xy_index_;
    return *this;
  }

  XYIndexRangeIterator& operator++() {
    // This is a necessary evil. Bounds checking is very expensive and needs to
    // be avoided in production. We have unit tests that exercise this check
    // in debug mode.
    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i& operator*() { return xy_index_; }

  bool operator==(const XYIndexRangeIterator& other) const {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator& other) const {
    return !operator==(other);
  }

  XYIndexRangeIterator begin() {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }

  XYIndexRangeIterator end() {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

 private:
  XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                       const Eigen::Array2i& max_xy_index)
      : min_xy_index_(min_xy_index),
        max_xy_index_(max_xy_index),
        xy_index_(min_xy_index) {}

  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_XY_INDEX_H_
