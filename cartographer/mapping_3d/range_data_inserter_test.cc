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

#include "cartographer/mapping_3d/range_data_inserter.h"

#include <memory>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_3d {
namespace {

class RangeDataInserterTest : public ::testing::Test {
 protected:
  RangeDataInserterTest() : hybrid_grid_(1.f) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "num_free_space_voxels = 1000, "
        "}");
    options_ = CreateRangeDataInserterOptions(parameter_dictionary.get());
    range_data_inserter_.reset(new RangeDataInserter(options_));
  }

  void InsertPointCloud() {
    const Eigen::Vector3f origin = Eigen::Vector3f(0.f, 0.f, -4.f);
    sensor::PointCloud returns = {
        {-3.f, -1.f, 4.f}, {-2.f, 0.f, 4.f}, {-1.f, 1.f, 4.f}, {0.f, 2.f, 4.f}};
    range_data_inserter_->Insert(sensor::RangeData{origin, returns, {}},
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

  const proto::RangeDataInserterOptions& options() const { return options_; }

 private:
  HybridGrid hybrid_grid_;
  std::unique_ptr<RangeDataInserter> range_data_inserter_;
  proto::RangeDataInserterOptions options_;
};

TEST_F(RangeDataInserterTest, InsertPointCloud) {
  InsertPointCloud();
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.f, 0.f, -4.f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.f, 0.f, -3.f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.f, 0.f, -2.f),
              1e-4);
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      if (x < -3 || x > 0 || y != x + 2) {
        EXPECT_FALSE(IsKnown(x, y, 4.f));
      } else {
        EXPECT_NEAR(options().hit_probability(), GetProbability(x, y, 4.f),
                    1e-4);
      }
    }
  }
}

TEST_F(RangeDataInserterTest, ProbabilityProgression) {
  InsertPointCloud();
  EXPECT_NEAR(options().hit_probability(), GetProbability(-2.f, 0.f, 4.f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(-2.f, 0.f, 3.f),
              1e-4);
  EXPECT_NEAR(options().miss_probability(), GetProbability(0.f, 0.f, -3.f),
              1e-4);

  for (int i = 0; i < 1000; ++i) {
    InsertPointCloud();
  }
  EXPECT_NEAR(mapping::kMaxProbability, GetProbability(-2.f, 0.f, 4.f), 1e-3);
  EXPECT_NEAR(mapping::kMinProbability, GetProbability(-2.f, 0.f, 3.f), 1e-3);
  EXPECT_NEAR(mapping::kMinProbability, GetProbability(0.f, 0.f, -3.f), 1e-3);
}

}  // namespace
}  // namespace mapping_3d
}  // namespace cartographer
