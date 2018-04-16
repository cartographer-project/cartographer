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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

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
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions2D(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap2D::Submap2D(const MapLimits& limits, const Eigen::Vector2f& origin)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      probability_grid_(common::make_unique<ProbabilityGrid>(limits)) {}

Submap2D::Submap2D(const proto::Submap2D& proto)
    : Submap(transform::ToRigid3(proto.local_pose())) {
  if (proto.has_grid()) {
    CHECK(proto.grid().has_probability_grid_2d());
    probability_grid_ = common::make_unique<ProbabilityGrid>(proto.grid());
  }
  set_num_range_data(proto.num_range_data());
  set_finished(proto.finished());
}

void Submap2D::ToProto(proto::Submap* const proto,
                       bool include_probability_grid_data) const {
  auto* const submap_2d = proto->mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(finished());
  if (include_probability_grid_data) {
    CHECK(probability_grid_);
    *submap_2d->mutable_grid() = probability_grid_->ToProto();
  }
}

void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    CHECK(proto.submap_2d().grid().has_probability_grid_2d());
    probability_grid_ = common::make_unique<ProbabilityGrid>(submap_2d.grid());
  }
}

void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  CHECK(probability_grid_);
  response->set_submap_version(num_range_data());

  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid_->ComputeCroppedLimits(&offset, &limits);

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid_->IsKnown(xy_index + offset)) {
      // We would like to add 'delta' but this is not possible using a value and
      // alpha. We use premultiplied alpha, so when 'delta' is positive we can
      // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
      // zero, and use 'alpha' to subtract. This is only correct when the pixel
      // is currently white, so walls will look too gray. This should be hard to
      // detect visually for the user, though.
      const int delta =
          128 - ProbabilityToLogOddsInteger(
                    probability_grid_->GetProbability(xy_index + offset));
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
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  common::FastGzipString(cells, texture->mutable_cells());

  texture->set_width(limits.num_x_cells);
  texture->set_height(limits.num_y_cells);
  const double resolution = probability_grid_->limits().resolution();
  texture->set_resolution(resolution);
  const double max_x =
      probability_grid_->limits().max().x() - resolution * offset.y();
  const double max_y =
      probability_grid_->limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose().inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));
}

void Submap2D::InsertRangeData(const sensor::RangeData& range_data,
                               const RangeDataInserter2D& range_data_inserter) {
  CHECK(probability_grid_);
  CHECK(!finished());
  range_data_inserter.Insert(range_data, probability_grid_.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  CHECK(probability_grid_);
  CHECK(!finished());
  *probability_grid_ = ComputeCroppedProbabilityGrid(*probability_grid_);
  set_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

void ActiveSubmaps2D::InsertRangeData(const sensor::RangeData& range_data) {
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_);
  }
  if (submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
}

std::vector<std::shared_ptr<Submap2D>> ActiveSubmaps2D::submaps() const {
  return submaps_;
}

int ActiveSubmaps2D::matching_index() const { return matching_submap_index_; }

void ActiveSubmaps2D::FinishSubmap() {
  Submap2D* submap = submaps_.front().get();
  submap->Finish();
  ++matching_submap_index_;
  submaps_.erase(submaps_.begin());
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() > 1) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    FinishSubmap();
  }
  constexpr int kInitialSubmapSize = 100;
  submaps_.push_back(common::make_unique<Submap2D>(
      MapLimits(options_.resolution(),
                origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                            options_.resolution() *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      origin));
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping
}  // namespace cartographer
