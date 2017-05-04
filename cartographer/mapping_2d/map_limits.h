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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/proto/map_limits.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the point ('x', 'y') which may be
  // outside the map, i.e., negative or too large indices that will return
  // false for Contains().
  Eigen::Array2i GetXYIndexOfCellContainingPoint(const double x,
                                                 const double y) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - y) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - x) / resolution_ - 0.5));
  }

  // Returns true of the ProbabilityGrid contains 'xy_index'.
  bool Contains(const Eigen::Array2i& xy_index) const {
    return (Eigen::Array2i(0, 0) <= xy_index).all() &&
           (xy_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

  // Computes MapLimits that contain the origin, and all rays (both returns and
  // misses) in the 'trajectory'.
  static MapLimits ComputeMapLimits(
      const double resolution,
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
    Eigen::AlignedBox2f bounding_box = ComputeMapBoundingBox(trajectory_nodes);
    // Add some padding to ensure all rays are still contained in the map after
    // discretization.
    const float kPadding = 3.f * resolution;
    bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
    bounding_box.max() += kPadding * Eigen::Vector2f::Ones();
    const Eigen::Vector2d pixel_sizes =
        bounding_box.sizes().cast<double>() / resolution;
    return MapLimits(resolution, bounding_box.max().cast<double>(),
                     CellLimits(common::RoundToInt(pixel_sizes.y()),
                                common::RoundToInt(pixel_sizes.x())));
  }

  static Eigen::AlignedBox2f ComputeMapBoundingBox(
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
    Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());
    for (const auto& node : trajectory_nodes) {
      const auto& data = *node.constant_data;
      if (!data.range_data_3d.returns.empty()) {
        const sensor::RangeData range_data = sensor::TransformRangeData(
            Decompress(data.range_data_3d), node.pose.cast<float>());
        bounding_box.extend(range_data.origin.head<2>());
        for (const Eigen::Vector3f& hit : range_data.returns) {
          bounding_box.extend(hit.head<2>());
        }
      } else {
        const sensor::RangeData range_data = sensor::TransformRangeData(
            data.range_data_2d, node.pose.cast<float>());
        bounding_box.extend(range_data.origin.head<2>());
        for (const Eigen::Vector3f& hit : range_data.returns) {
          bounding_box.extend(hit.head<2>());
        }
        for (const Eigen::Vector3f& miss : range_data.misses) {
          bounding_box.extend(miss.head<2>());
        }
      }
    }
    return bounding_box;
  }

 private:
  double resolution_;
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
