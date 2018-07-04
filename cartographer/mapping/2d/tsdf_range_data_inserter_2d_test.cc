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
        "update_free_space = true,"
        "normal_estimation_options = {"
        "num_normal_samples = 4,"
        "sample_radius = 2.,"
        "},"
        "project_sdf_distance_to_scan_normal = true,"
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

TEST_F(RangeDataInserterTest2DTSDF, InsertPointConstantIntegrator) {
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

/*

TEST_F(RangeDataInserterTest2DTSDF, InsertPointLinearIntegrator) {
  options_.mutable_tsdf()->set_range_data_inserter_type(proto::LINEAR_WEIGHT);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  for (float y = -0.5; y < 5.; ++y) {
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, tsdf_.GetMaxTSDF()), tsdf_.GetMinTSDF());
    float expected_weight = options_.tsdf().update_weight() / 4.f;
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }

  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = -0.5; y < 5.; ++y) {
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, tsdf_.GetMaxTSDF()), tsdf_.GetMinTSDF());
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
    EXPECT_NEAR(tsdf_.GetMaxWeight(), tsdf_.GetWeight(cell_index), 1e-3);
  }
}

TEST_F(RangeDataInserterTest2DTSDF, InsertPointQuadraticIntegrator) {
  options_.mutable_tsdf()->set_range_data_inserter_type(
      proto::QUADRATIC_WEIGHT);
  range_data_inserter_ = common::make_unique<TSDFRangeDataInserter2D>(options_);
  InsertPoint();
  for (float y = -0.5; y < 5.; ++y) {
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, tsdf_.GetMaxTSDF()), tsdf_.GetMinTSDF());
    float expected_weight = options_.tsdf().update_weight() / 16.f;
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
    EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  }

  for (int i = 0; i < 1000; ++i) {
    InsertPoint();
  }
  for (float y = -0.5; y < 5.; ++y) {
    float x = -0.5f;
    Eigen::Array2i cell_index =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
    float expected_tsdf =
        std::max(std::min(3.5f - y, tsdf_.GetMaxTSDF()), tsdf_.GetMinTSDF());
    EXPECT_TRUE(tsdf_.IsKnown(cell_index));
    EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
    EXPECT_NEAR(tsdf_.GetMaxWeight(), tsdf_.GetWeight(cell_index), 1e-3);
  }
}



TEST_F(RangeDataInserterTest2DTSDF, RayCastTest) {
  InsertPoint();

  sensor::RangeData range_data;
  range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
  range_data.origin.x() = -0.5f;
  range_data.origin.y() = -0.5f;
  range_data_inserter_->Insert(range_data, &tsdf_);

  float y = -0.5f;
  float x = -0.5f;
  Eigen::Array2i cell_index =
      tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  float expected_tsdf =
      std::max(std::min(3.5f - y, tsdf_.GetMaxTSDF()), tsdf_.GetMinTSDF());
  float expected_weight = options_.tsdf().update_weight();
  EXPECT_TRUE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  x = 0.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  expected_tsdf = tsdf_.GetMaxTSDF();
  expected_weight = 0.;
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
  x = 1.5f;
  cell_index = tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y));
  EXPECT_FALSE(tsdf_.IsKnown(cell_index));
  EXPECT_NEAR(expected_tsdf, tsdf_.GetTSDF(cell_index), 1e-4);
  EXPECT_NEAR(expected_weight, tsdf_.GetWeight(cell_index), 1e-3);
}


*/

}  // namespace
}  // namespace mapping
}  // namespace cartographer