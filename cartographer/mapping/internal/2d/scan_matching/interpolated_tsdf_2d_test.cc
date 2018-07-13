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

#include "cartographer/mapping/internal/2d/scan_matching/interpolated_tsdf_2d.h"

#include "Eigen/Core"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class InterpolatedTSDF2DTest : public ::testing::Test {
 protected:
  InterpolatedTSDF2DTest()
      : tsdf_(MapLimits(1., Eigen::Vector2d(5.5, 5.5), CellLimits(10, 10)),
              1.0f, 10.0f, &conversion_tables_),
        interpolated_tsdf_(tsdf_) {}

  float GetUninterpolatedCorrespondenceCost(float x, float y) const {
    return tsdf_.GetCorrespondenceCost(
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y)));
  }

  float GetUninterpolatedWeight(float x, float y) const {
    return tsdf_.GetWeight(tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y)));
  }

  ValueConversionTables conversion_tables_;
  TSDF2D tsdf_;
  InterpolatedTSDF2D interpolated_tsdf_;
};

TEST_F(InterpolatedTSDF2DTest, InterpolatesGridPoints) {
  std::vector<Eigen::Vector2f> inner_points = {
      Eigen::Vector2f(1.f, 1.f), Eigen::Vector2f(2.f, 1.f),
      Eigen::Vector2f(1.f, 2.f), Eigen::Vector2f(2.f, 2.f)};
  for (const Eigen::Vector2f& point : inner_points) {
    tsdf_.SetCell(tsdf_.limits().GetCellIndex(point), 0.1f, 1.0f);
  }
  // Insert surrounding points.
  for (size_t x = 0; x < 4; ++x) {
    tsdf_.SetCell(tsdf_.limits().GetCellIndex(
                      Eigen::Vector2f(static_cast<float>(x), 0.f)),
                  0.1f, 1.0f);
    tsdf_.SetCell(tsdf_.limits().GetCellIndex(
                      Eigen::Vector2f(static_cast<float>(x), 3.f)),
                  0.1f, 1.0f);
  }
  for (size_t y = 1; y < 3; ++y) {
    tsdf_.SetCell(tsdf_.limits().GetCellIndex(
                      Eigen::Vector2f(0.f, static_cast<float>(y))),
                  0.1f, 1.0f);
    tsdf_.SetCell(tsdf_.limits().GetCellIndex(
                      Eigen::Vector2f(3.f, static_cast<float>(y))),
                  0.1f, 1.0f);
  }
  for (const Eigen::Vector2f& point : inner_points) {
    EXPECT_NEAR(
        GetUninterpolatedCorrespondenceCost(point[0], point[1]),
        interpolated_tsdf_.GetCorrespondenceCost(static_cast<double>(point[0]),
                                                 static_cast<double>(point[1])),
        1e-4);
    EXPECT_NEAR(GetUninterpolatedWeight(point[0], point[1]),
                interpolated_tsdf_.GetWeight(static_cast<double>(point[0]),
                                             static_cast<double>(point[1])),
                1e-4);
  }
  // Check unknown cell.
  EXPECT_NEAR(tsdf_.GetMaxCorrespondenceCost(),
              interpolated_tsdf_.GetCorrespondenceCost(3.0, 2.0), 1e-4);
  EXPECT_NEAR(GetUninterpolatedWeight(3.f, 2.f),
              interpolated_tsdf_.GetWeight(3.0, 2.0), 1e-4);
}

TEST_F(InterpolatedTSDF2DTest, InterpolatesWithinCell) {
  const float tsd_00 = 0.1f;
  const float tsd_01 = 0.2f;
  const float tsd_10 = 0.3f;
  const float tsd_11 = 0.4f;
  const float w_00 = 1.f;
  const float w_01 = 2.f;
  const float w_10 = 3.f;
  const float w_11 = 4.f;

  tsdf_.SetCell(tsdf_.limits().GetCellIndex(Eigen::Vector2f(0.f, 0.f)), tsd_00,
                w_00);
  tsdf_.SetCell(tsdf_.limits().GetCellIndex(Eigen::Vector2f(0.f, 1.f)), tsd_01,
                w_01);
  tsdf_.SetCell(tsdf_.limits().GetCellIndex(Eigen::Vector2f(1.f, 0.f)), tsd_10,
                w_10);
  tsdf_.SetCell(tsdf_.limits().GetCellIndex(Eigen::Vector2f(1.f, 1.f)), tsd_11,
                w_11);

  const double kSampleStep = tsdf_.limits().resolution() / 100.;
  for (double x = 0. + kSampleStep; x < 1.; x += tsdf_.limits().resolution()) {
    for (double y = 0. + kSampleStep; y < 1.;
         y += tsdf_.limits().resolution()) {
      const float tsd_expected = (x * tsd_10 + (1.f - x) * tsd_00) * (1.f - y) +
                                 (x * tsd_11 + (1.f - x) * tsd_01) * y;
      EXPECT_NEAR(interpolated_tsdf_.GetCorrespondenceCost(x, y), tsd_expected,
                  1e-3);
      const float w_expected = (x * w_10 + (1.f - x) * w_00) * (1.f - y) +
                               (x * w_11 + (1.f - x) * w_01) * y;
      EXPECT_NEAR(interpolated_tsdf_.GetWeight(x, y), w_expected, 1e-3);
    }
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
