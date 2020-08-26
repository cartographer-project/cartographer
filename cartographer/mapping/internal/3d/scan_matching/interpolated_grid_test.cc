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

#include "cartographer/mapping/internal/3d/scan_matching/interpolated_grid.h"

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class InterpolatedGridTest : public ::testing::Test {
 protected:
  InterpolatedGridTest()
      : hybrid_grid_(0.1f), interpolated_grid_(hybrid_grid_) {
    for (const Eigen::Vector3f& point :
         {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
          Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
          Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
          Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
      hybrid_grid_.SetProbability(hybrid_grid_.GetCellIndex(point), 1.);
    }
  }

  float GetHybridGridProbability(float x, float y, float z) const {
    return hybrid_grid_.GetProbability(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  HybridGrid hybrid_grid_;
  InterpolatedProbabilityGrid interpolated_grid_;
};

TEST_F(InterpolatedGridTest, InterpolatesGridPoints) {
  for (double z = -1.; z < 3.; z += hybrid_grid_.resolution()) {
    for (double y = 1.; y < 5.; y += hybrid_grid_.resolution()) {
      for (double x = -8.; x < -2.; x += hybrid_grid_.resolution()) {
        EXPECT_NEAR(GetHybridGridProbability(x, y, z),
                    interpolated_grid_.GetInterpolatedValue(x, y, z), 1e-6);
      }
    }
  }
}

TEST_F(InterpolatedGridTest, MonotonicBehaviorBetweenGridPointsInX) {
  const double kSampleStep = hybrid_grid_.resolution() / 10.;
  for (double z = -1.; z < 3.; z += hybrid_grid_.resolution()) {
    for (double y = 1.; y < 5.; y += hybrid_grid_.resolution()) {
      for (double x = -8.; x < -2.; x += hybrid_grid_.resolution()) {
        const float start_probability = GetHybridGridProbability(x, y, z);
        const float next_probability =
            GetHybridGridProbability(x + hybrid_grid_.resolution(), y, z);
        const float grid_difference = next_probability - start_probability;
        if (std::abs(grid_difference) < 1e-6f) {
          continue;
        }
        for (double sample = kSampleStep;
             sample < hybrid_grid_.resolution() - 2 * kSampleStep;
             sample += kSampleStep) {
          EXPECT_LT(0.,
                    grid_difference * (interpolated_grid_.GetInterpolatedValue(
                                           x + sample + kSampleStep, y, z) -
                                       interpolated_grid_.GetInterpolatedValue(
                                           x + sample, y, z)));
        }
      }
    }
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
