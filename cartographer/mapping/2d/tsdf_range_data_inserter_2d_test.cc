/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class RangeDataInserterTest2DTSDF : public ::testing::Test {
 protected:
  RangeDataInserterTest2DTSDF()
      : tsdf_(MapLimits(1., Eigen::Vector2d(0., 7.), CellLimits(8, 1)), 2.0,
              10.0) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "truncation_distance = 2.0,"
        "maximum_weight = 10.,"
        "update_free_space = false,"
        "normal_estimation_options = {"
        "num_normal_samples = 2,"
        "sample_radius = 10.,"
        "},"
        "project_sdf_distance_to_scan_normal = false,"
        "update_weight_range_exponent = 0,"
        "update_weight_angle_scan_normal_to_ray_kernel_bandwith = 0,"
        "update_weight_distance_cell_to_hit_kernel_bandwith = 0,"
        "}");
    options_ = CreateTSDFRangeDataInserterOptions2D(parameter_dictionary.get());
    range_data_inserter_ =
        common::make_unique<TSDFRangeDataInserter2D>(options_);
  }

  void InsertPoint() {
    sensor::RangeData range_data;
    range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
    range_data.origin.x() = -0.5f;
    range_data.origin.y() = -0.5f;
    range_data_inserter_->Insert(range_data, &tsdf_);
    tsdf_.FinishUpdate();
  }

  proto::TSDFRangeDataInserterOptions2D options_;
  TSDF2D tsdf_;
  std::unique_ptr<TSDFRangeDataInserter2D> range_data_inserter_;
};

TEST_F(RangeDataInserterTest2DTSDF, InsertPoint) {
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  const float maximum_weight = static_cast<float>(options_.maximum_weight());

  for (float y = 1.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f;
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
    // Cells next to ray
    x = 0.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    expected_tsdf = -truncation_distance;
    expected_weight = 0.;
    EXPECT_FALSE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
    x = 1.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    EXPECT_FALSE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }

  // Cells next to ray
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(0.5, 6.5));
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(-truncation_distance, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(0., tsdf_.GetWeight(cell_index), 1e-3);

  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, -1.5));
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(-truncation_distance, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(0., tsdf_.GetWeight(cell_index), 1e-3);

  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = 1.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(maximum_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointWithFreeSpaceUpdate) {
  options_.set_update_free_space(true);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  const float maximum_weight = static_cast<float>(options_.maximum_weight());

  for (float y = -0.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f;
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
    // Cells next to ray
    x = 0.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    expected_tsdf = -truncation_distance;
    expected_weight = 0.;
    EXPECT_FALSE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
    x = 1.5f;
    cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    EXPECT_FALSE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }

  // Cells next to ray
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, 6.5));
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(-truncation_distance, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(0., tsdf_.GetWeight(cell_index), 1e-3);

  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(-0.5, -1.5));
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(-truncation_distance, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(0., tsdf_.GetWeight(cell_index), 1e-3);

  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = -0.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(maximum_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointLinearWeight) {
  options_.set_update_weight_range_exponent(1);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (float y = 1.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f / 4.f;
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointQuadraticWeight) {
  options_.set_update_weight_range_exponent(2);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (float y = 1.5; y < 6.; ++y) {
    // Cell on ray
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, truncation_distance), -truncation_distance);
    float expected_weight = 1.f / std::pow(4.f, 2);
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }
}

TEST_F(RangeDataInserterTest2DTSDF,
       InsertSmallAnglePointWithoutNormalProjection) {
  sensor::RangeData range_data;
  range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
  range_data.returns.emplace_back(5.5f, 3.5f, 0.f);
  range_data.returns.emplace_back(10.5f, 3.5f, 0.f);
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);
  tsdf_.FinishUpdate();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  float x = 4.5f;
  float y = 2.5f;
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  Eigen::Vector2f ray =
      Eigen::Vector2f(-0.5f, -0.5f) - Eigen::Vector2f(5.5f, 3.5f);
  float ray_length = ray.norm();
  Eigen::Vector2f origin_to_cell =
      Eigen::Vector2f(x, y) - Eigen::Vector2f(-0.5f, -0.5f);
  float distance_origin_to_cell = origin_to_cell.norm();
  float expected_tsdf = ray_length - distance_origin_to_cell;
  float expected_weight = 1.f;
  EXPECT_TRUE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
}

TEST_F(RangeDataInserterTest2DTSDF, InsertSmallAnglePointWitNormalProjection) {
  options_.set_project_sdf_distance_to_scan_normal(true);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  sensor::RangeData range_data;
  range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
  range_data.returns.emplace_back(5.5f, 3.5f, 0.f);
  range_data.returns.emplace_back(10.5f, 3.5f, 0.f);
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);
  tsdf_.FinishUpdate();
  float x = 4.5f;
  float y = 2.5f;
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  float expected_tsdf = 1.f;
  float expected_weight = 1.f;
  EXPECT_TRUE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  x = 6.5f;
  y = 4.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  expected_tsdf = -1.f;
  expected_weight = 1.f;
  EXPECT_TRUE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSD(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer