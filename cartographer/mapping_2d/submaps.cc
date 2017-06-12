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

#include "cartographer/mapping_2d/submaps.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"
#include "webp/encode.h"

namespace cartographer {
namespace mapping_2d {

namespace {

void WriteDebugImage(const string& filename,
                     const ProbabilityGrid& probability_grid) {
  constexpr int kUnknown = 128;
  const CellLimits& cell_limits = probability_grid.limits().cell_limits();
  const int width = cell_limits.num_x_cells;
  const int height = cell_limits.num_y_cells;
  std::vector<uint8_t> rgb;
  for (const Eigen::Array2i& xy_index :
       XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
    CHECK(probability_grid.limits().Contains(xy_index));
    const uint8_t value =
        probability_grid.IsKnown(xy_index)
            ? common::RoundToInt(
                  (1. - probability_grid.GetProbability(xy_index)) * 255 + 0)
            : kUnknown;
    rgb.push_back(value);
    rgb.push_back(value);
    rgb.push_back(value);
  }
  uint8_t* output = nullptr;
  size_t output_size =
      WebPEncodeLosslessRGB(rgb.data(), width, height, 3 * width, &output);
  std::unique_ptr<uint8_t, void (*)(void*)> output_deleter(output, std::free);
  std::ofstream output_file(filename, std::ios::out | std::ios::binary);
  output_file.write(reinterpret_cast<char*>(output), output_size);
  output_file.close();
  CHECK(output_file) << "Writing " << filename << " failed.";
}

}  // namespace

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid) {
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  const double resolution = probability_grid.limits().resolution();
  const Eigen::Vector2d max =
      probability_grid.limits().max() -
      resolution * Eigen::Vector2d(offset.y(), offset.x());
  ProbabilityGrid cropped_grid(MapLimits(resolution, max, limits));
  cropped_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  options.set_half_length(parameter_dictionary->GetDouble("half_length"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  options.set_output_debug_images(
      parameter_dictionary->GetBool("output_debug_images"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap::Submap(const MapLimits& limits, const Eigen::Vector2f& origin)
    : mapping::Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      probability_grid_(limits) {}

void Submap::ToResponseProto(
    const transform::Rigid3d&,
    mapping::proto::SubmapQuery::Response* const response) const {
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid_.ComputeCroppedLimits(&offset, &limits);

  string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid_.IsKnown(xy_index + offset)) {
      // We would like to add 'delta' but this is not possible using a value and
      // alpha. We use premultiplied alpha, so when 'delta' is positive we can
      // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
      // zero, and use 'alpha' to subtract. This is only correct when the pixel
      // is currently white, so walls will look too gray. This should be hard to
      // detect visually for the user, though.
      const int delta =
          128 - mapping::ProbabilityToLogOddsInteger(
                    probability_grid_.GetProbability(xy_index + offset));
      const uint8 alpha = delta > 0 ? 0 : -delta;
      const uint8 value = delta > 0 ? delta : 0;
      cells.push_back(value);
      cells.push_back((value || alpha) ? alpha : 1);
    } else {
      constexpr uint8 kUnknownLogOdds = 0;
      cells.push_back(static_cast<uint8>(kUnknownLogOdds));  // value
      cells.push_back(0);                                    // alpha
    }
  }
  common::FastGzipString(cells, response->mutable_cells());

  response->set_width(limits.num_x_cells);
  response->set_height(limits.num_y_cells);
  const double resolution = probability_grid_.limits().resolution();
  response->set_resolution(resolution);
  const double max_x =
      probability_grid_.limits().max().x() - resolution * offset.y();
  const double max_y =
      probability_grid_.limits().max().y() - resolution * offset.x();
  *response->mutable_slice_pose() = transform::ToProto(
      local_pose().inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));
}

Submaps::Submaps(const proto::SubmapsOptions& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

void Submaps::InsertRangeData(const sensor::RangeData& range_data) {
  for (const int index : insertion_indices()) {
    Submap* submap = submaps_[index].get();
    CHECK(submap->finished_probability_grid_ == nullptr);
    range_data_inserter_.Insert(range_data, &submap->probability_grid_);
    ++submap->num_range_data_;
  }
  const Submap* const last_submap = Get(size() - 1);
  if (last_submap->num_range_data_ == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
}

const Submap* Submaps::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

int Submaps::size() const { return submaps_.size(); }

void Submaps::FinishSubmap(int index) {
  // Crop the finished Submap before inserting a new Submap to reduce peak
  // memory usage a bit.
  Submap* submap = submaps_[index].get();
  CHECK(submap->finished_probability_grid_ == nullptr);
  submap->probability_grid_ =
      ComputeCroppedProbabilityGrid(submap->probability_grid_);
  submap->finished_probability_grid_ = &submap->probability_grid_;
  if (options_.output_debug_images()) {
    // Output the Submap that won't be changed from now on.
    WriteDebugImage("submap" + std::to_string(index) + ".webp",
                    submap->probability_grid_);
  }
}

void Submaps::AddSubmap(const Eigen::Vector2f& origin) {
  if (size() > 1) {
    FinishSubmap(size() - 2);
  }
  const int num_cells_per_dimension =
      common::RoundToInt(2. * options_.half_length() / options_.resolution()) +
      1;
  submaps_.push_back(common::make_unique<Submap>(
      MapLimits(options_.resolution(),
                origin.cast<double>() +
                    options_.half_length() * Eigen::Vector2d::Ones(),
                CellLimits(num_cells_per_dimension, num_cells_per_dimension)),
      origin));
  LOG(INFO) << "Added submap " << size();
}

}  // namespace mapping_2d
}  // namespace cartographer
