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
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::AlignedBox2d& edge_limits) {
    SetLimits(resolution, edge_limits);
  }

  MapLimits(const double resolution, const double max_x, const double max_y,
            const CellLimits& cell_limits) {
    SetLimits(resolution, max_x, max_y, cell_limits);
  }

  MapLimits(const double resolution, const double center_x,
            const double center_y) {
    SetLimits(resolution, center_x + 100 * resolution,
              center_y + 100 * resolution, CellLimits(200, 200));
  }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the limits of the grid from edge to edge in meters.
  const Eigen::AlignedBox2d& edge_limits() const { return edge_limits_; }

  // Returns the limits of the grid between the centers of the edge cells in
  // meters.
  const Eigen::AlignedBox2d& centered_limits() const {
    return centered_limits_;
  }

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
        common::RoundToInt((centered_limits_.max().y() - y) / resolution_),
        common::RoundToInt((centered_limits_.max().x() - x) / resolution_));
  }

  // Returns true of the ProbabilityGrid contains 'xy_index'.
  bool Contains(const Eigen::Array2i& xy_index) const {
    return (Eigen::Array2i(0, 0) <= xy_index).all() &&
           (xy_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

  // Computes MapLimits that contain the origin, and all laser rays (both
  // returns and missing echoes) in the 'trajectory'.
  static MapLimits ComputeMapLimits(
      const double resolution,
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
    Eigen::AlignedBox2f bounding_box = ComputeMapBoundingBox(trajectory_nodes);
    // Add some padding to ensure all rays are still contained in the map after
    // discretization.
    const float kPadding = 3.f * resolution;
    bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
    bounding_box.max() += kPadding * Eigen::Vector2f::Ones();
    return MapLimits(resolution, bounding_box.cast<double>());
  }

  static Eigen::AlignedBox2f ComputeMapBoundingBox(
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
    Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());
    for (const auto& node : trajectory_nodes) {
      const auto& data = *node.constant_data;
      if (!data.laser_fan_3d.returns.empty()) {
        const sensor::LaserFan3D laser_fan_3d = sensor::TransformLaserFan3D(
            Decompress(data.laser_fan_3d), node.pose.cast<float>());
        bounding_box.extend(laser_fan_3d.origin.head<2>());
        for (const Eigen::Vector3f& hit : laser_fan_3d.returns) {
          bounding_box.extend(hit.head<2>());
        }
      } else {
        const sensor::LaserFan laser_fan = sensor::TransformLaserFan(
            data.laser_fan, transform::Project2D(node.pose).cast<float>());
        bounding_box.extend(laser_fan.origin);
        for (const Eigen::Vector2f& hit : laser_fan.point_cloud) {
          bounding_box.extend(hit);
        }
        for (const Eigen::Vector2f& miss : laser_fan.missing_echo_point_cloud) {
          bounding_box.extend(miss);
        }
      }
    }
    return bounding_box;
  }

 private:
  // Returns the center of the resolution interval containing 'x'.
  double Center(const double x) {
    return (std::floor(x / resolution_) + 0.5) * resolution_;
  }

  // Sets the cell size to the specified resolution in meters and the limits of
  // the grid to the specified bounding box in meters.
  void SetLimits(double resolution, const Eigen::AlignedBox2d& limits) {
    CHECK(!limits.isEmpty());
    resolution_ = resolution;
    const int num_x_cells = common::RoundToInt((Center(limits.max().y()) -
                                                Center(limits.min().y())) /
                                               resolution) +
                            1;
    const int num_y_cells = common::RoundToInt((Center(limits.max().x()) -
                                                Center(limits.min().x())) /
                                               resolution) +
                            1;
    SetLimits(resolution, limits.max().x(), limits.max().y(),
              CellLimits(num_x_cells, num_y_cells));
  }

  // Sets the cell size to the specified resolution in meters and the limits of
  // the grid to the specified bounding box.
  //
  // Note that implementing this in terms of the previous SetLimits method
  // results in unnecessary (and expensive?) calls to common::RoundToInt.
  //
  // TODO(whess): Measure whether it really is still too slow. Otherwise,
  // simplify.
  void SetLimits(double resolution, double max_x, double max_y,
                 const CellLimits& limits) {
    CHECK_GT(resolution, 0.);
    CHECK_GT(limits.num_x_cells, 0.);
    CHECK_GT(limits.num_y_cells, 0.);

    resolution_ = resolution;
    cell_limits_ = limits;
    centered_limits_.max().x() = Center(max_x);
    centered_limits_.max().y() = Center(max_y);
    centered_limits_.min().x() = centered_limits_.max().x() -
                                 resolution_ * (cell_limits_.num_y_cells - 1);
    centered_limits_.min().y() = centered_limits_.max().y() -
                                 resolution_ * (cell_limits_.num_x_cells - 1);
    UpdateEdgeLimits();
  }

  // Updates the edge limits from the previously calculated centered limits.
  void UpdateEdgeLimits() {
    const double half_resolution = resolution_ / 2.;
    edge_limits_.min().x() = centered_limits_.min().x() - half_resolution;
    edge_limits_.min().y() = centered_limits_.min().y() - half_resolution;
    edge_limits_.max().x() = centered_limits_.max().x() + half_resolution;
    edge_limits_.max().y() = centered_limits_.max().y() + half_resolution;
  }

  double resolution_;
  Eigen::AlignedBox2d edge_limits_;
  Eigen::AlignedBox2d centered_limits_;
  CellLimits cell_limits_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
