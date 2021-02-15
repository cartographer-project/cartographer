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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <cmath>
#include <memory>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherTestOptions2D() {
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "linear_search_window = 0.6, "
      "angular_search_window = 0.16, "
      "translation_delta_cost_weight = 0., "
      "rotation_delta_cost_weight = 0., "
      "}");
  return CreateRealTimeCorrelativeScanMatcherOptions(
      parameter_dictionary.get());
}

class RealTimeCorrelativeScanMatcherTest : public ::testing::Test {
 protected:
  RealTimeCorrelativeScanMatcherTest() {
    point_cloud_.push_back({Eigen::Vector3f{0.025f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.025f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.075f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.125f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.075f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.025f, 0.f}});
    real_time_correlative_scan_matcher_ =
        absl::make_unique<RealTimeCorrelativeScanMatcher2D>(
            CreateRealTimeCorrelativeScanMatcherTestOptions2D());
  }

  void SetUpTSDF() {
    grid_ = absl::make_unique<TSDF2D>(
        MapLimits(0.05, Eigen::Vector2d(0.3, 0.5), CellLimits(20, 20)), 0.3,
        1.0, &conversion_tables_);
    {
      auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      })text");
      range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(
          CreateTSDFRangeDataInserterOptions2D(parameter_dictionary.get()));
    }
    range_data_inserter_->Insert(
        sensor::RangeData{Eigen::Vector3f(0.5f, -0.5f, 0.f), point_cloud_, {}},
        grid_.get());
    grid_->FinishUpdate();
  }

  void SetUpProbabilityGrid() {
    grid_ = absl::make_unique<ProbabilityGrid>(
        MapLimits(0.05, Eigen::Vector2d(0.05, 0.25), CellLimits(6, 6)),
        &conversion_tables_);
    {
      auto parameter_dictionary = common::MakeDictionary(
          "return { "
          "insert_free_space = true, "
          "hit_probability = 0.7, "
          "miss_probability = 0.4, "
          "}");
      range_data_inserter_ =
          absl::make_unique<ProbabilityGridRangeDataInserter2D>(
              CreateProbabilityGridRangeDataInserterOptions2D(
                  parameter_dictionary.get()));
    }
    range_data_inserter_->Insert(
        sensor::RangeData{Eigen::Vector3f::Zero(), point_cloud_, {}},
        grid_.get());
    grid_->FinishUpdate();
  }

  ValueConversionTables conversion_tables_;
  std::unique_ptr<Grid2D> grid_;
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<RealTimeCorrelativeScanMatcher2D>
      real_time_correlative_scan_matcher_;
};

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePerfectHighResolutionCandidateProbabilityGrid) {
  SetUpProbabilityGrid();
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan2D> discrete_scans =
      DiscretizeScans(grid_->limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate2D> candidates;
  candidates.emplace_back(0, 0, 0, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      *grid_, discrete_scans, SearchParameters(0, 0, 0., 0.), &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(0, candidates[0].y_index_offset);
  // Every point should align perfectly.
  EXPECT_NEAR(0.7, candidates[0].score, 1e-2);
}

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePerfectHighResolutionCandidateTSDF) {
  SetUpTSDF();
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan2D> discrete_scans =
      DiscretizeScans(grid_->limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate2D> candidates;
  candidates.emplace_back(0, 0, 0, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      *grid_, discrete_scans, SearchParameters(0, 0, 0., 0.), &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(0, candidates[0].y_index_offset);
  // Every point should align perfectly.
  EXPECT_NEAR(1.0, candidates[0].score, 1e-1);
  EXPECT_LT(0.95, candidates[0].score);
}

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePartiallyCorrectHighResolutionCandidateProbabilityGrid) {
  SetUpProbabilityGrid();
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan2D> discrete_scans =
      DiscretizeScans(grid_->limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate2D> candidates;
  candidates.emplace_back(0, 0, 1, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      *grid_, discrete_scans, SearchParameters(0, 0, 0., 0.), &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(1, candidates[0].y_index_offset);
  // 3 points should align perfectly.
  EXPECT_LT(0.7 * 3. / 7., candidates[0].score);
  EXPECT_GT(0.7, candidates[0].score);
}

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePartiallyCorrectHighResolutionCandidateTSDF) {
  SetUpTSDF();
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan2D> discrete_scans =
      DiscretizeScans(grid_->limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate2D> candidates;
  candidates.emplace_back(0, 0, 1, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      *grid_, discrete_scans, SearchParameters(0, 0, 0., 0.), &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(1, candidates[0].y_index_offset);
  // 3 points should align perfectly.
  EXPECT_LT(1.0 - 4. / (7. * 6.), candidates[0].score);
  EXPECT_GT(1.0, candidates[0].score);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
