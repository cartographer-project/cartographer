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

#include "cartographer/mapping/submaps.h"

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

constexpr uint8 Submaps::kUnknownLogOdds;

Submaps::Submaps() {}

Submaps::~Submaps() {}

int Submaps::matching_index() const {
  if (size() > 1) {
    return size() - 2;
  }
  return size() - 1;
}

std::vector<int> Submaps::insertion_indices() const {
  if (size() > 1) {
    return {size() - 2, size() - 1};
  }
  return {size() - 1};
}

void Submaps::AddProbabilityGridToResponse(
    const transform::Rigid3d& local_submap_pose,
    const mapping_2d::ProbabilityGrid& probability_grid,
    proto::SubmapQuery::Response* response) {
  Eigen::Array2i offset;
  mapping_2d::CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);

  string cells;
  for (const Eigen::Array2i& xy_index :
       mapping_2d::XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      // We would like to add 'delta' but this is not possible using a value and
      // alpha. We use premultiplied alpha, so when 'delta' is positive we can
      // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
      // zero, and use 'alpha' to subtract. This is only correct when the pixel
      // is currently white, so walls will look too gray. This should be hard to
      // detect visually for the user, though.
      const int delta =
          128 - ProbabilityToLogOddsInteger(
                    probability_grid.GetProbability(xy_index + offset));
      const uint8 alpha = delta > 0 ? 0 : -delta;
      const uint8 value = delta > 0 ? delta : 0;
      cells.push_back(value);
      cells.push_back((value || alpha) ? alpha : 1);
    } else {
      cells.push_back(static_cast<uint8>(kUnknownLogOdds));  // value
      cells.push_back(0);                                    // alpha
    }
  }
  common::FastGzipString(cells, response->mutable_cells());

  response->set_width(limits.num_x_cells);
  response->set_height(limits.num_y_cells);
  const double resolution = probability_grid.limits().resolution();
  response->set_resolution(resolution);
  const double max_x =
      probability_grid.limits().max().x() - resolution * offset.y();
  const double max_y =
      probability_grid.limits().max().y() - resolution * offset.x();
  *response->mutable_slice_pose() = transform::ToProto(
      local_submap_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));
}

}  // namespace mapping
}  // namespace cartographer
