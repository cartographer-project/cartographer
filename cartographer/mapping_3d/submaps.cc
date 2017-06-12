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
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

namespace {

constexpr float kSliceHalfHeight = 0.1f;

struct RaySegment {
  Eigen::Vector2f from;
  Eigen::Vector2f to;
  bool hit;  // Whether there is a hit at 'to'.
};

struct PixelData {
  int min_z = INT_MAX;
  int max_z = INT_MIN;
  int count = 0;
  float probability_sum = 0.f;
  float max_probability = 0.5f;
};

// We compute a slice around the xy-plane. 'transform' is applied to the rays in
// global map frame to allow choosing an arbitrary slice.
void GenerateSegmentForSlice(const sensor::RangeData& range_data,
                             const transform::Rigid3f& pose,
                             const transform::Rigid3f& transform,
                             std::vector<RaySegment>* segments) {
  const sensor::RangeData transformed_range_data =
      sensor::TransformRangeData(range_data, transform * pose);
  segments->reserve(transformed_range_data.returns.size());
  for (const Eigen::Vector3f& hit : transformed_range_data.returns) {
    const Eigen::Vector2f origin_xy = transformed_range_data.origin.head<2>();
    const float origin_z = transformed_range_data.origin.z();
    const float delta_z = hit.z() - origin_z;
    const Eigen::Vector2f delta_xy = hit.head<2>() - origin_xy;
    if (origin_z < -kSliceHalfHeight) {
      // Ray originates below the slice.
      if (hit.z() > kSliceHalfHeight) {
        // Ray is cutting through the slice.
        segments->push_back(RaySegment{
            origin_xy + (-kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            origin_xy + (kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() > -kSliceHalfHeight) {
        // Hit is inside the slice.
        segments->push_back(RaySegment{
            origin_xy + (-kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            hit.head<2>(), true});
      }
    } else if (origin_z < kSliceHalfHeight) {
      // Ray originates inside the slice.
      if (hit.z() < -kSliceHalfHeight) {
        // Hit is below.
        segments->push_back(RaySegment{
            origin_xy,
            origin_xy + (-kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() < kSliceHalfHeight) {
        // Full ray is inside the slice.
        segments->push_back(RaySegment{origin_xy, hit.head<2>(), true});
      } else {
        // Hit is above.
        segments->push_back(RaySegment{
            origin_xy,
            origin_xy + (kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            false});
      }
    } else {
      // Ray originates above the slice.
      if (hit.z() < -kSliceHalfHeight) {
        // Ray is cutting through the slice.
        segments->push_back(RaySegment{
            origin_xy + (kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            origin_xy + (-kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            false});
      } else if (hit.z() < kSliceHalfHeight) {
        // Hit is inside the slice.
        segments->push_back(RaySegment{
            origin_xy + (kSliceHalfHeight - origin_z) / delta_z * delta_xy,
            hit.head<2>(), true});
      }
    }
  }
}

void UpdateFreeSpaceFromSegment(const RaySegment& segment,
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

void InsertSegmentsIntoProbabilityGrid(const std::vector<RaySegment>& segments,
                                       const std::vector<uint16>& hit_table,
                                       const std::vector<uint16>& miss_table,
                                       mapping_2d::ProbabilityGrid* result) {
  result->StartUpdate();
  if (segments.empty()) {
    return;
  }
  Eigen::Vector2f min = segments.front().from;
  Eigen::Vector2f max = min;
  for (const RaySegment& segment : segments) {
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

  for (const RaySegment& segment : segments) {
    if (segment.hit) {
      result->ApplyLookupTable(result->limits().GetXYIndexOfCellContainingPoint(
                                   segment.to.x(), segment.to.y()),
                               hit_table);
    }
  }
  for (const RaySegment& segment : segments) {
    UpdateFreeSpaceFromSegment(segment, miss_table, result);
  }
}

// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses and reflectivity
// information.
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
                                            const float max_range) {
  sensor::RangeData result{range_data.origin, {}, {}};
  for (const Eigen::Vector3f& hit : range_data.returns) {
    if ((hit - range_data.origin).norm() <= max_range) {
      result.returns.push_back(hit);
    }
  }
  return result;
}

std::vector<PixelData> AccumulatePixelData(
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability =
        mapping::ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  return accumulated_pixel_data;
}

// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
std::vector<Eigen::Array4i> ExtractVoxelData(
    const HybridGrid& hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = mapping::ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);

    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  return voxel_indices_and_probabilities;
}

// Builds texture data containing interleaved value and alpha for the
// visualization from 'accumulated_pixel_data'.
string ComputePixelValues(
    const std::vector<PixelData>& accumulated_pixel_data) {
  string cell_data;
  cell_data.reserve(2 * accumulated_pixel_data.size());
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      cell_data.push_back(0);  // value
      cell_data.push_back(0);  // alpha
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
    cell_data.push_back(value);                         // value
    cell_data.push_back((value || alpha) ? alpha : 1);  // alpha
  }
  return cell_data;
}

}  // namespace

void InsertIntoProbabilityGrid(
    const sensor::RangeData& range_data, const transform::Rigid3f& pose,
    const float slice_z,
    const mapping_2d::RangeDataInserter& range_data_inserter,
    mapping_2d::ProbabilityGrid* result) {
  std::vector<RaySegment> segments;
  GenerateSegmentForSlice(
      range_data, pose,
      transform::Rigid3f::Translation(-slice_z * Eigen::Vector3f::UnitZ()),
      &segments);
  InsertSegmentsIntoProbabilityGrid(segments, range_data_inserter.hit_table(),
                                    range_data_inserter.miss_table(), result);
}

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap::Submap(const float high_resolution, const float low_resolution,
               const transform::Rigid3d& local_pose)
    : mapping::Submap(local_pose),
      high_resolution_hybrid_grid_(high_resolution),
      low_resolution_hybrid_grid_(low_resolution) {}

void Submap::ToResponseProto(
    const transform::Rigid3d& global_submap_pose,
    mapping::proto::SubmapQuery::Response* const response) const {
  // Generate an X-ray view through the 'hybrid_grid', aligned to the xy-plane
  // in the global map frame.
  const float resolution = high_resolution_hybrid_grid_.resolution();
  response->set_resolution(resolution);

  // Compute a bounding box for the texture.
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ExtractVoxelData(high_resolution_hybrid_grid_,
                       global_submap_pose.cast<float>(), &min_index,
                       &max_index);

  const int width = max_index.y() - min_index.y() + 1;
  const int height = max_index.x() - min_index.x() + 1;
  response->set_width(width);
  response->set_height(height);

  const std::vector<PixelData> accumulated_pixel_data = AccumulatePixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);
  const string cell_data = ComputePixelValues(accumulated_pixel_data);

  common::FastGzipString(cell_data, response->mutable_cells());
  *response->mutable_slice_pose() = transform::ToProto(
      global_submap_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(
          max_index.x() * resolution, max_index.y() * resolution,
          global_submap_pose.translation().z())));
}

Submaps::Submaps(const proto::SubmapsOptions& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one submap which we can return and will
  // create it at the origin in absence of a better choice.
  //
  // TODO(whess): Start with no submaps, so that all of them can be
  // approximately gravity aligned.
  AddSubmap(transform::Rigid3d::Identity());
}

const Submap* Submaps::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

int Submaps::size() const { return submaps_.size(); }

void Submaps::InsertRangeData(const sensor::RangeData& range_data,
                              const Eigen::Quaterniond& gravity_alignment) {
  for (const int index : insertion_indices()) {
    Submap* submap = submaps_[index].get();
    const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
        range_data, submap->local_pose().inverse().cast<float>());
    range_data_inserter_.Insert(
        FilterRangeDataByMaxRange(transformed_range_data,
                                  options_.high_resolution_max_range()),
        &submap->high_resolution_hybrid_grid_);
    range_data_inserter_.Insert(transformed_range_data,
                                &submap->low_resolution_hybrid_grid_);
    ++submap->num_range_data_;
  }
  const Submap* const last_submap = Get(size() - 1);
  if (last_submap->num_range_data_ == options_.num_range_data()) {
    AddSubmap(transform::Rigid3d(range_data.origin.cast<double>(),
                                 gravity_alignment));
  }
}

void Submaps::AddSubmap(const transform::Rigid3d& local_pose) {
  if (size() > 1) {
    Submap* submap = submaps_[size() - 2].get();
    CHECK(!submap->finished_);
    submap->finished_ = true;
  }
  submaps_.emplace_back(new Submap(options_.high_resolution(),
                                   options_.low_resolution(), local_pose));
  LOG(INFO) << "Added submap " << size();
}

}  // namespace mapping_3d
}  // namespace cartographer
