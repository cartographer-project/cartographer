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

#include "cartographer/mapping_3d/submaps.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/sensor/laser.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

namespace {

constexpr float kSliceHalfHeight = 0.1f;

struct LaserSegment {
  Eigen::Vector2f from;
  Eigen::Vector2f to;
  bool hit;  // Whether there is a laser return at 'to'.
};

// We compute a slice around the xy-plane. 'transform' is applied to the laser
// rays in global map frame to allow choosing an arbitrary slice.
void GenerateSegmentForSlice(const sensor::LaserFan3D& laser_fan_3d,
                             const transform::Rigid3f& pose,
                             const transform::Rigid3f& transform,
                             std::vector<LaserSegment>* segments) {
  const sensor::LaserFan3D laser_fan =
      sensor::TransformLaserFan3D(laser_fan_3d, transform * pose);
  segments->reserve(laser_fan.returns.size());
  for (const Eigen::Vector3f& hit : laser_fan.returns) {
    const Eigen::Vector2f laser_origin_xy = laser_fan.origin.head<2>();
    const float laser_origin_z = laser_fan.origin.z();
    const float delta_z = hit.z() - laser_origin_z;
    const Eigen::Vector2f delta_xy = hit.head<2>() - laser_origin_xy;
    if (laser_origin_z < -kSliceHalfHeight) {
      // Laser ray originates below the slice.
      if (hit.z() > kSliceHalfHeight) {
        // Laser ray is cutting through the slice.
        segments->push_back(LaserSegment{
            laser_origin_xy +
                (-kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            laser_origin_xy +
                (kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() > -kSliceHalfHeight) {
        // Laser return is inside the slice.
        segments->push_back(LaserSegment{
            laser_origin_xy +
                (-kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            hit.head<2>(), true});
      }
    } else if (laser_origin_z < kSliceHalfHeight) {
      // Laser ray originates inside the slice.
      if (hit.z() < -kSliceHalfHeight) {
        // Laser hit is below.
        segments->push_back(LaserSegment{
            laser_origin_xy,
            laser_origin_xy +
                (-kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() < kSliceHalfHeight) {
        // Full ray is inside the slice.
        segments->push_back(LaserSegment{laser_origin_xy, hit.head<2>(), true});
      } else {
        // Laser hit is above.
        segments->push_back(LaserSegment{
            laser_origin_xy,
            laser_origin_xy +
                (kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            false});
      }
    } else {
      // Laser ray originates above the slice.
      if (hit.z() < -kSliceHalfHeight) {
        // Laser ray is cutting through the slice.
        segments->push_back(LaserSegment{
            laser_origin_xy +
                (kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            laser_origin_xy +
                (-kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() < kSliceHalfHeight) {
        // Laser return is inside the slice.
        segments->push_back(LaserSegment{
            laser_origin_xy +
                (kSliceHalfHeight - laser_origin_z) / delta_z * delta_xy,
            hit.head<2>(), true});
      }
    }
  }
}

void UpdateFreeSpaceFromSegment(const LaserSegment& segment,
                                const std::vector<uint16>& miss_table,
                                mapping_2d::ProbabilityGrid* result) {
  Eigen::Array2i from = result->limits().GetXYIndexOfCellContainingPoint(
      segment.from.x(), segment.from.y());
  Eigen::Array2i to = result->limits().GetXYIndexOfCellContainingPoint(
      segment.to.x(), segment.to.y());
  bool large_delta_y =
      std::abs(to.y() - from.y()) > std::abs(to.x() - from.x());
  if (large_delta_y) {
    std::swap(from.x(), from.y());
    std::swap(to.x(), to.y());
  }
  if (from.x() > to.x()) {
    std::swap(from, to);
  }
  const int dx = to.x() - from.x();
  const int dy = std::abs(to.y() - from.y());
  int error = dx / 2;
  const int direction = (from.y() < to.y()) ? 1 : -1;

  for (; from.x() < to.x(); ++from.x()) {
    if (large_delta_y) {
      result->ApplyLookupTable(Eigen::Array2i(from.y(), from.x()), miss_table);
    } else {
      result->ApplyLookupTable(from, miss_table);
    }
    error -= dy;
    if (error < 0) {
      from.y() += direction;
      error += dx;
    }
  }
}

void InsertSegmentsIntoProbabilityGrid(
    const std::vector<LaserSegment>& segments,
    const std::vector<uint16>& hit_table, const std::vector<uint16>& miss_table,
    mapping_2d::ProbabilityGrid* result) {
  result->StartUpdate();
  if (segments.empty()) {
    return;
  }
  Eigen::Vector2f min = segments.front().from;
  Eigen::Vector2f max = min;
  for (const LaserSegment& segment : segments) {
    min = min.cwiseMin(segment.from);
    min = min.cwiseMin(segment.to);
    max = max.cwiseMax(segment.from);
    max = max.cwiseMax(segment.to);
  }
  const float padding = 10. * result->limits().resolution();
  max += Eigen::Vector2f(padding, padding);
  min -= Eigen::Vector2f(padding, padding);
  result->GrowLimits(min.x(), min.y());
  result->GrowLimits(max.x(), max.y());

  for (const LaserSegment& segment : segments) {
    if (segment.hit) {
      result->ApplyLookupTable(result->limits().GetXYIndexOfCellContainingPoint(
                                   segment.to.x(), segment.to.y()),
                               hit_table);
    }
  }
  for (const LaserSegment& segment : segments) {
    UpdateFreeSpaceFromSegment(segment, miss_table, result);
  }
}

}  // namespace

void InsertIntoProbabilityGrid(
    const sensor::LaserFan3D& laser_fan_3d, const transform::Rigid3f& pose,
    const float slice_z, const mapping_2d::LaserFanInserter& laser_fan_inserter,
    mapping_2d::ProbabilityGrid* result) {
  std::vector<LaserSegment> segments;
  GenerateSegmentForSlice(
      laser_fan_3d, pose,
      transform::Rigid3f::Translation(-slice_z * Eigen::Vector3f::UnitZ()),
      &segments);
  InsertSegmentsIntoProbabilityGrid(segments, laser_fan_inserter.hit_table(),
                                    laser_fan_inserter.miss_table(), result);
}

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_laser_fans(
      parameter_dictionary->GetNonNegativeInt("num_laser_fans"));
  *options.mutable_laser_fan_inserter_options() = CreateLaserFanInserterOptions(
      parameter_dictionary->GetDictionary("laser_fan_inserter").get());
  CHECK_GT(options.num_laser_fans(), 0);
  return options;
}

Submap::Submap(const float high_resolution, const float low_resolution,
               const Eigen::Vector3f& origin, const int begin_laser_fan_index)
    : mapping::Submap(origin, begin_laser_fan_index),
      high_resolution_hybrid_grid(high_resolution, origin),
      low_resolution_hybrid_grid(low_resolution, origin) {}

Submaps::Submaps(const proto::SubmapsOptions& options)
    : options_(options),
      laser_fan_inserter_(options.laser_fan_inserter_options()) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector3f::Zero());
}

const Submap* Submaps::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

int Submaps::size() const { return submaps_.size(); }

void Submaps::SubmapToProto(
    int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
    const transform::Rigid3d& global_submap_pose,
    mapping::proto::SubmapQuery::Response* const response) {
  // Generate an X-ray view through the 'hybrid_grid', aligned to the xy-plane
  // in the global map frame.
  const HybridGrid& hybrid_grid = Get(index)->high_resolution_hybrid_grid;
  response->set_resolution(hybrid_grid.resolution());

  // Compute a bounding box for the texture.
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  ExtractVoxelData(
      hybrid_grid,
      (global_submap_pose * Get(index)->local_pose().inverse()).cast<float>(),
      &min_index, &max_index);

  const int width = max_index.y() - min_index.y() + 1;
  const int height = max_index.x() - min_index.x() + 1;
  response->set_width(width);
  response->set_height(height);

  AccumulatePixelData(width, height, min_index, max_index);
  ComputePixelValues(width, height);

  common::FastGzipString(celldata_, response->mutable_cells());
  *response->mutable_slice_pose() =
      transform::ToProto(global_submap_pose.inverse() *
                         transform::Rigid3d::Translation(Eigen::Vector3d(
                             max_index.x() * hybrid_grid.resolution(),
                             max_index.y() * hybrid_grid.resolution(),
                             global_submap_pose.translation().z())));
}

void Submaps::InsertLaserFan(const sensor::LaserFan3D& laser_fan) {
  CHECK_LT(num_laser_fans_, std::numeric_limits<int>::max());
  ++num_laser_fans_;
  for (const int index : insertion_indices()) {
    Submap* submap = submaps_[index].get();
    laser_fan_inserter_.Insert(
        sensor::FilterLaserFanByMaxRange(laser_fan,
                                         options_.high_resolution_max_range()),
        &submap->high_resolution_hybrid_grid);
    laser_fan_inserter_.Insert(laser_fan, &submap->low_resolution_hybrid_grid);
    submap->end_laser_fan_index = num_laser_fans_;
  }
  ++num_laser_fans_in_last_submap_;
  if (num_laser_fans_in_last_submap_ == options_.num_laser_fans()) {
    AddSubmap(laser_fan.origin);
  }
}

const HybridGrid& Submaps::high_resolution_matching_grid() const {
  return submaps_[matching_index()]->high_resolution_hybrid_grid;
}

const HybridGrid& Submaps::low_resolution_matching_grid() const {
  return submaps_[matching_index()]->low_resolution_hybrid_grid;
}

void Submaps::AddTrajectoryNodeIndex(const int trajectory_node_index) {
  for (int i = 0; i != size(); ++i) {
    Submap& submap = *submaps_[i];
    if (submap.end_laser_fan_index == num_laser_fans_ &&
        submap.begin_laser_fan_index <= num_laser_fans_ - 1) {
      submap.trajectory_node_indices.push_back(trajectory_node_index);
    }
  }
}

void Submaps::AddSubmap(const Eigen::Vector3f& origin) {
  if (size() > 1) {
    Submap* submap = submaps_[size() - 2].get();
    CHECK(!submap->finished);
    submap->finished = true;
  }
  submaps_.emplace_back(new Submap(options_.high_resolution(),
                                   options_.low_resolution(), origin,
                                   num_laser_fans_));
  LOG(INFO) << "Added submap " << size();
  num_laser_fans_in_last_submap_ = 0;
}

void Submaps::AccumulatePixelData(const int width, const int height,
                                  const Eigen::Array2i& min_index,
                                  const Eigen::Array2i& max_index) {
  accumulated_pixel_data_.clear();
  accumulated_pixel_data_.resize(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities_) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data_[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability =
        mapping::ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
}

void Submaps::ExtractVoxelData(const HybridGrid& hybrid_grid,
                               const transform::Rigid3f& transform,
                               Eigen::Array2i* min_index,
                               Eigen::Array2i* max_index) {
  voxel_indices_and_probabilities_.clear();
  const float resolution_inverse = 1. / hybrid_grid.resolution();

  constexpr double kXrayObstructedCellProbabilityLimit = 0.501;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_local =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_local;
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);

    voxel_indices_and_probabilities_.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
}

void Submaps::ComputePixelValues(const int width, const int height) {
  celldata_.resize(2 * width * height);
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  auto it = celldata_.begin();
  for (size_t i = 0; i < accumulated_pixel_data_.size(); ++i) {
    const PixelData& pixel = accumulated_pixel_data_.at(i);
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      *it = 0;  // value
      ++it;
      *it = 0;  // alpha
      ++it;
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = mapping::ClampProbability(
        (pixel.probability_sum + free_space_probability * free_space_weight) /
        total_weight);
    const int delta =
        128 - mapping::ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    *it = value;  // value
    ++it;
    *it = (value || alpha) ? alpha : 1;  // alpha
    ++it;
  }
}

}  // namespace mapping_3d
}  // namespace cartographer
