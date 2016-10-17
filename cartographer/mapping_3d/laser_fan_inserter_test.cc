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

#include "cartographer/mapping_3d/laser_fan_inserter.h"

#include <memory>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_3d {
namespace {

class LaserFanInserterTest : public ::testing::Test {
 protected:
  LaserFanInserterTest()
      : hybrid_grid_(1.f, Eigen::Vector3f(0.5f, 0.5f, 0.5f)) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "num_free_space_voxels = 1000, "
        "}");
    options_ = CreateLaserFanInserterOptions(parameter_dictionary.get());
    laser_fan_inserter_.reset(new LaserFanInserter(options_));
  }

  void InsertPointCloud() {
    const Eigen::Vector3f origin = Eigen::Vector3f(0.5f, 0.5f, -3.5f);
    sensor::PointCloud laser_returns = {{-2.5f, -0.5f, 4.5f},
                                        {-1.5f, 0.5f, 4.5f},
                                        {-0.5f, 1.5f, 4.5f},
                                        {0.5f, 2.5f, 4.5f}};
    laser_fan_inserter_->Insert(sensor::LaserFan{origin, laser_returns, {}},
                                &hybrid_grid_);
  }

  float GetProbability(float x, float y, float z) const {
    return hybrid_grid_.GetProbability(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  float IsKnown(float x, float y, float z) const {
    return hybrid_grid_.IsKnown(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  const proto::LaserFanInserterOptions& options() const { return options_; }

 private:
  HybridGrid hybrid_grid_;
  std::unique_ptr<LaserFanInserter> laser_fan_inserter_;
  proto::LaserFanInserterOptions options_;
};

TEST_F(LaserFanInserterTest, InsertPointCloud) {
  InsertPointCloud();
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.5f, 0.5f, -3.5f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.5f, 0.5f, -2.5f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.5f, 0.5f, -1.5f),
              1e-4);
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      if (x < -3 || x > 0 || y != x + 2) {
        EXPECT_FALSE(IsKnown(x + 0.5f, y + 0.5f, 4.5f));
      } else {
        EXPECT_NEAR(options().hit_probability(),
                    GetProbability(x + 0.5f, y + 0.5f, 4.5f), 1e-4);
      }
    }
  }
}

TEST_F(LaserFanInserterTest, ProbabilityProgression) {
  InsertPointCloud();
  EXPECT_NEAR(options().hit_probability(), GetProbability(-1.5f, 0.5f, 4.5f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(-1.5f, 0.5f, 3.5f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.5f, 0.5f, -2.5f),
              1e-4);

  for (int i = 0; i < 1000; ++i) {
    InsertPointCloud();
  }
  EXPECT_NEAR(mapping::kMaxProbability, GetProbability(-1.5f, 0.5f, 4.5f),
              1e-3);
  EXPECT_NEAR(mapping::kMinProbability, GetProbability(-1.5f, 0.5f, 3.5f),
              1e-3);
  EXPECT_NEAR(mapping::kMinProbability, GetProbability(0.5f, 0.5f, -2.5f),
              1e-3);
}

}  // namespace
}  // namespace mapping_3d
}  // namespace cartographer
