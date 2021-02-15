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

#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"

#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;

class TSDFSpaceCostFunction2DTest : public ::testing::Test {
 protected:
  TSDFSpaceCostFunction2DTest()
      : tsdf_(MapLimits(0.1, Eigen::Vector2d(2.05, 2.05), CellLimits(40, 40)),
              0.3, 1.0, &conversion_tables_) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "truncation_distance = 0.3,"
        "maximum_weight = 1.0,"
        "update_free_space = false,"
        "normal_estimation_options = {"
        "num_normal_samples = 2,"
        "sample_radius = 10.,"
        "},"
        "project_sdf_distance_to_scan_normal = true,"
        "update_weight_range_exponent = 0,"
        "update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0,"
        "update_weight_distance_cell_to_hit_kernel_bandwidth = 0,"
        "}");
    options_ = CreateTSDFRangeDataInserterOptions2D(parameter_dictionary.get());
    range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(options_);
  }

  void InsertPointcloud() {
    auto range_data = sensor::RangeData();
    for (float x = -.5; x < 0.5f; x += 0.1) {
      range_data.returns.push_back({Eigen::Vector3f{x, 1.0f, 0.f}});
    }
    range_data.origin.x() = -0.5f;
    range_data.origin.y() = -0.5f;
    range_data_inserter_->Insert(range_data, &tsdf_);
    tsdf_.FinishUpdate();
  }

  ValueConversionTables conversion_tables_;
  proto::TSDFRangeDataInserterOptions2D options_;
  TSDF2D tsdf_;
  std::unique_ptr<TSDFRangeDataInserter2D> range_data_inserter_;
};

TEST_F(TSDFSpaceCostFunction2DTest, MatchEmptyTSDF) {
  const sensor::PointCloud matching_cloud({{Eigen::Vector3f{0.f, 0.f, 0.f}}});
  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateTSDFMatchCostFunction2D(1.f, matching_cloud, tsdf_));
  const std::array<double, 3> pose_estimate{{0., 0., 0.}};
  const std::array<const double*, 1> parameter_blocks{{pose_estimate.data()}};
  std::array<double, 1> residuals;
  std::array<std::array<double, 3>, 1> jacobians;
  std::array<double*, 1> jacobians_ptrs;
  for (int i = 0; i < 1; ++i) jacobians_ptrs[i] = jacobians[i].data();
  bool valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_FALSE(valid_result);
}

TEST_F(TSDFSpaceCostFunction2DTest, ExactInitialPose) {
  InsertPointcloud();
  const sensor::PointCloud matching_cloud({{Eigen::Vector3f{0.f, 1.0f, 0.f}}});
  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateTSDFMatchCostFunction2D(1.f, matching_cloud, tsdf_));
  const std::array<double, 3> pose_estimate{{0., 0., 0.}};
  const std::array<const double*, 1> parameter_blocks{{pose_estimate.data()}};
  std::array<double, 1> residuals;
  std::array<std::array<double, 3>, 1> jacobians;
  std::array<double*, 1> jacobians_ptrs;
  for (int i = 0; i < 1; ++i) jacobians_ptrs[i] = jacobians[i].data();
  const bool valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_TRUE(valid_result);
  EXPECT_THAT(residuals, ElementsAre(DoubleNear(0., 1e-3)));
  EXPECT_THAT(jacobians[0],
              ElementsAre(DoubleNear(0., 1e-3), DoubleNear(-1., 1e-3),
                          DoubleNear(0., 1e-3)));
}

TEST_F(TSDFSpaceCostFunction2DTest, PertubatedInitialPose) {
  InsertPointcloud();
  sensor::PointCloud matching_cloud({{Eigen::Vector3f{0.f, 1.0f, 0.f}}});
  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateTSDFMatchCostFunction2D(1.f, matching_cloud, tsdf_));
  std::array<double, 3> pose_estimate{{0., 0.1, 0.}};
  std::array<const double*, 1> parameter_blocks{{pose_estimate.data()}};
  std::array<double, 1> residuals;
  std::array<std::array<double, 3>, 1> jacobians;
  std::array<double*, 1> jacobians_ptrs;
  for (int i = 0; i < 1; ++i) jacobians_ptrs[i] = jacobians[i].data();

  bool valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_TRUE(valid_result);
  EXPECT_THAT(residuals, ElementsAre(DoubleNear(-0.1, 1e-3)));
  EXPECT_THAT(jacobians[0],
              ElementsAre(DoubleNear(0., 1e-3), DoubleNear(-1., 1e-3),
                          DoubleNear(0., 1e-3)));

  pose_estimate[1] = -0.1;
  parameter_blocks = {{pose_estimate.data()}};
  valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_TRUE(valid_result);
  EXPECT_THAT(residuals, ElementsAre(DoubleNear(0.1, 1e-3)));
  EXPECT_THAT(jacobians[0],
              ElementsAre(DoubleNear(0., 1e-3), DoubleNear(-1., 1e-3),
                          DoubleNear(0., 1e-3)));
}

TEST_F(TSDFSpaceCostFunction2DTest, InvalidInitialPose) {
  InsertPointcloud();
  sensor::PointCloud matching_cloud({{Eigen::Vector3f{0.f, 1.0f, 0.f}}});
  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateTSDFMatchCostFunction2D(1.f, matching_cloud, tsdf_));
  std::array<double, 3> pose_estimate{{0., 0.4, 0.}};
  std::array<const double*, 1> parameter_blocks{{pose_estimate.data()}};
  std::array<double, 1> residuals;
  std::array<std::array<double, 3>, 1> jacobians;
  std::array<double*, 1> jacobians_ptrs;
  for (int i = 0; i < 1; ++i) jacobians_ptrs[i] = jacobians[i].data();

  bool valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_FALSE(valid_result);

  pose_estimate[1] = -0.4;
  parameter_blocks = {{pose_estimate.data()}};
  valid_result = cost_function->Evaluate(
      parameter_blocks.data(), residuals.data(), jacobians_ptrs.data());
  EXPECT_FALSE(valid_result);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
