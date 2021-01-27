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

#include "cartographer/mapping/3d/range_data_inserter_3d.h"

#include <memory>
#include <vector>

#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class RangeDataInserter3DTest : public ::testing::Test {
 protected:
  RangeDataInserter3DTest() : hybrid_grid_(1.f), intensity_hybrid_grid_(1.f) {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "num_free_space_voxels = 1000, "
        "intensity_threshold = 100, "
        "}");
    options_ = CreateRangeDataInserterOptions3D(parameter_dictionary.get());
    range_data_inserter_.reset(new RangeDataInserter3D(options_));
  }

  void InsertPointCloud() {
    const Eigen::Vector3f origin = Eigen::Vector3f(0.f, 0.f, -4.f);
    const std::vector<sensor::RangefinderPoint> returns = {
        {Eigen::Vector3f{-3.f, -1.f, 4.f}},
        {Eigen::Vector3f{-2.f, 0.f, 4.f}},
        {Eigen::Vector3f{-1.f, 1.f, 4.f}},
        {Eigen::Vector3f{0.f, 2.f, 4.f}}};
    range_data_inserter_->Insert(
        sensor::RangeData{origin, sensor::PointCloud(returns), {}},
        &hybrid_grid_,
        /*intensity_hybrid_grid=*/nullptr);
  }

  void InsertPointCloudWithIntensities() {
    const Eigen::Vector3f origin = Eigen::Vector3f(0.f, 0.f, -4.f);
    const std::vector<sensor::RangefinderPoint> returns = {
        {Eigen::Vector3f{-3.f, -1.f, 4.f}},
        {Eigen::Vector3f{-2.f, 0.f, 4.f}},
        {Eigen::Vector3f{-1.f, 1.f, 4.f}},
        {Eigen::Vector3f{0.f, 2.f, 4.f}}};
    const std::vector<float> intensities{7.f, 8.f, 9.f, 10.f};
    range_data_inserter_->Insert(
        sensor::RangeData{origin, sensor::PointCloud(returns, intensities), {}},
        &hybrid_grid_, &intensity_hybrid_grid_);
  }

  float GetProbability(float x, float y, float z) const {
    return hybrid_grid_.GetProbability(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  float GetIntensity(float x, float y, float z) const {
    return intensity_hybrid_grid_.GetIntensity(
        intensity_hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  float IsKnown(float x, float y, float z) const {
    return hybrid_grid_.IsKnown(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
  }

  const proto::RangeDataInserterOptions3D& options() const { return options_; }

 private:
  HybridGrid hybrid_grid_;
  IntensityHybridGrid intensity_hybrid_grid_;
  std::unique_ptr<RangeDataInserter3D> range_data_inserter_;
  proto::RangeDataInserterOptions3D options_;
};

TEST_F(RangeDataInserter3DTest, InsertPointCloud) {
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

TEST_F(RangeDataInserter3DTest, InsertPointCloudWithIntensities) {
  InsertPointCloudWithIntensities();
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
        EXPECT_NEAR(0.f, GetIntensity(x, y, 4.f), 1e-6);
      } else {
        EXPECT_NEAR(options().hit_probability(), GetProbability(x, y, 4.f),
                    1e-4);
        EXPECT_NEAR(10 + x, GetIntensity(x, y, 4.f), 1e-6);
      }
    }
  }
}

TEST_F(RangeDataInserter3DTest, ProbabilityProgression) {
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
  EXPECT_NEAR(kMaxProbability, GetProbability(-2.f, 0.f, 4.f), 1e-3);
  EXPECT_NEAR(kMinProbability, GetProbability(-2.f, 0.f, 3.f), 1e-3);
  EXPECT_NEAR(kMinProbability, GetProbability(0.f, 0.f, -3.f), 1e-3);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
