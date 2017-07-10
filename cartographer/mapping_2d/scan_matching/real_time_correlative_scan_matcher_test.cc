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

#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"

#include <cmath>
#include <memory>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {
namespace {

class RealTimeCorrelativeScanMatcherTest : public ::testing::Test {
 protected:
  RealTimeCorrelativeScanMatcherTest()
      : probability_grid_(
            MapLimits(0.05, Eigen::Vector2d(0.05, 0.25), CellLimits(6, 6))) {
    {
      auto parameter_dictionary = common::MakeDictionary(
          "return { "
          "insert_free_space = true, "
          "hit_probability = 0.7, "
          "miss_probability = 0.4, "
          "}");
      range_data_inserter_ = common::make_unique<RangeDataInserter>(
          CreateRangeDataInserterOptions(parameter_dictionary.get()));
    }
    point_cloud_.emplace_back(0.025f, 0.175f, 0.f);
    point_cloud_.emplace_back(-0.025f, 0.175f, 0.f);
    point_cloud_.emplace_back(-0.075f, 0.175f, 0.f);
    point_cloud_.emplace_back(-0.125f, 0.175f, 0.f);
    point_cloud_.emplace_back(-0.125f, 0.125f, 0.f);
    point_cloud_.emplace_back(-0.125f, 0.075f, 0.f);
    point_cloud_.emplace_back(-0.125f, 0.025f, 0.f);
    range_data_inserter_->Insert(
        sensor::RangeData{Eigen::Vector3f::Zero(), point_cloud_, {}},
        &probability_grid_);
    probability_grid_.FinishUpdate();
    {
      auto parameter_dictionary = common::MakeDictionary(
          "return {"
          "linear_search_window = 0.6, "
          "angular_search_window = 0.16, "
          "translation_delta_cost_weight = 0., "
          "rotation_delta_cost_weight = 0., "
          "}");
      real_time_correlative_scan_matcher_ =
          common::make_unique<RealTimeCorrelativeScanMatcher>(
              CreateRealTimeCorrelativeScanMatcherOptions(
                  parameter_dictionary.get()));
    }
  }

  ProbabilityGrid probability_grid_;
  std::unique_ptr<RangeDataInserter> range_data_inserter_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<RealTimeCorrelativeScanMatcher>
      real_time_correlative_scan_matcher_;
};

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePerfectHighResolutionCandidate) {
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      probability_grid_.limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate> candidates;
  candidates.emplace_back(0, 0, 0, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      probability_grid_, discrete_scans, SearchParameters(0, 0, 0., 0.),
      &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(0, candidates[0].y_index_offset);
  // Every point should align perfectly.
  EXPECT_NEAR(0.7, candidates[0].score, 1e-2);
}

TEST_F(RealTimeCorrelativeScanMatcherTest,
       ScorePartiallyCorrectHighResolutionCandidate) {
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud_, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      probability_grid_.limits(), scans, Eigen::Translation2f::Identity());
  std::vector<Candidate> candidates;
  candidates.emplace_back(0, 0, 1, SearchParameters(0, 0, 0., 0.));
  real_time_correlative_scan_matcher_->ScoreCandidates(
      probability_grid_, discrete_scans, SearchParameters(0, 0, 0., 0.),
      &candidates);
  EXPECT_EQ(0, candidates[0].scan_index);
  EXPECT_EQ(0, candidates[0].x_index_offset);
  EXPECT_EQ(1, candidates[0].y_index_offset);
  // 3 points should align perfectly.
  EXPECT_LT(0.7 * 3. / 7., candidates[0].score);
  EXPECT_GT(0.7, candidates[0].score);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
