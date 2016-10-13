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

#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"

#include "cartographer/sensor/point_cloud.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {
namespace {

TEST(SearchParameters, Construction) {
  const SearchParameters search_parameters(4, 5, 0.03, 0.05);
  EXPECT_EQ(5, search_parameters.num_angular_perturbations);
  EXPECT_NEAR(0.03, search_parameters.angular_perturbation_step_size, 1e-9);
  EXPECT_NEAR(0.05, search_parameters.resolution, 1e-9);
  EXPECT_EQ(11, search_parameters.num_scans);
  EXPECT_EQ(11, search_parameters.linear_bounds.size());
  for (const SearchParameters::LinearBounds linear_bounds :
       search_parameters.linear_bounds) {
    EXPECT_EQ(-4, linear_bounds.min_x);
    EXPECT_EQ(4, linear_bounds.max_x);
    EXPECT_EQ(-4, linear_bounds.min_y);
    EXPECT_EQ(4, linear_bounds.max_y);
  }
}

TEST(Candidate, Construction) {
  const SearchParameters search_parameters(4, 5, 0.03, 0.05);
  const Candidate candidate(3, 4, -5, search_parameters);
  EXPECT_EQ(3, candidate.scan_index);
  EXPECT_EQ(4, candidate.x_index_offset);
  EXPECT_EQ(-5, candidate.y_index_offset);
  EXPECT_NEAR(0.25, candidate.x, 1e-9);
  EXPECT_NEAR(-0.2, candidate.y, 1e-9);
  EXPECT_NEAR(-0.06, candidate.orientation, 1e-9);
  EXPECT_NEAR(0., candidate.score, 1e-9);

  Candidate bigger_candidate(3, 4, 5, search_parameters);
  bigger_candidate.score = 1.;
  EXPECT_LT(candidate, bigger_candidate);
}

TEST(GenerateRotatedScans, GenerateRotatedScans) {
  sensor::PointCloud point_cloud;
  point_cloud.emplace_back(-1.f, 1.f, 0.f);
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud, SearchParameters(0, 1, M_PI / 2., 0.));
  EXPECT_EQ(3, scans.size());
  EXPECT_NEAR(1., scans[0][0].x(), 1e-6);
  EXPECT_NEAR(1., scans[0][0].y(), 1e-6);
  EXPECT_NEAR(-1., scans[1][0].x(), 1e-6);
  EXPECT_NEAR(1., scans[1][0].y(), 1e-6);
  EXPECT_NEAR(-1., scans[2][0].x(), 1e-6);
  EXPECT_NEAR(-1., scans[2][0].y(), 1e-6);
}

TEST(DiscretizeScans, DiscretizeScans) {
  sensor::PointCloud point_cloud;
  point_cloud.emplace_back(0.025f, 0.175f, 0.f);
  point_cloud.emplace_back(-0.025f, 0.175f, 0.f);
  point_cloud.emplace_back(-0.075f, 0.175f, 0.f);
  point_cloud.emplace_back(-0.125f, 0.175f, 0.f);
  point_cloud.emplace_back(-0.125f, 0.125f, 0.f);
  point_cloud.emplace_back(-0.125f, 0.075f, 0.f);
  point_cloud.emplace_back(-0.125f, 0.025f, 0.f);
  const MapLimits map_limits(0.05, Eigen::Vector2d(0.05, 0.25),
                             CellLimits(6, 6));
  const std::vector<sensor::PointCloud> scans =
      GenerateRotatedScans(point_cloud, SearchParameters(0, 0, 0., 0.));
  const std::vector<DiscreteScan> discrete_scans =
      DiscretizeScans(map_limits, scans, Eigen::Translation2f::Identity());
  EXPECT_EQ(1, discrete_scans.size());
  EXPECT_EQ(7, discrete_scans[0].size());
  EXPECT_TRUE((Eigen::Array2i(1, 0) == discrete_scans[0][0]).all());
  EXPECT_TRUE((Eigen::Array2i(1, 1) == discrete_scans[0][1]).all());
  EXPECT_TRUE((Eigen::Array2i(1, 2) == discrete_scans[0][2]).all());
  EXPECT_TRUE((Eigen::Array2i(1, 3) == discrete_scans[0][3]).all());
  EXPECT_TRUE((Eigen::Array2i(2, 3) == discrete_scans[0][4]).all());
  EXPECT_TRUE((Eigen::Array2i(3, 3) == discrete_scans[0][5]).all());
  EXPECT_TRUE((Eigen::Array2i(4, 3) == discrete_scans[0][6]).all());
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
