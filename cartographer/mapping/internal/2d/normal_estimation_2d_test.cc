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

#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/math.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

using ::testing::TestWithParam;
using ::testing::Values;

TEST(NormalEstimation2DTest, SinglePoint) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = 2, "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  auto range_data = sensor::RangeData();
  const size_t num_angles = 100;
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                             static_cast<double>(num_angles) * 2. * M_PI -
                         M_PI;
    range_data.returns.clear();
    range_data.returns.push_back(
        {Eigen::Vector3d{std::cos(angle), std::sin(angle), 0.}.cast<float>()});
    std::vector<float> normals;
    normals = EstimateNormals(range_data, options);
    EXPECT_NEAR(common::NormalizeAngleDifference(angle - normals[0] - M_PI),
                0.0, 2.0 * M_PI / num_angles + 1e-4);
  }
}

TEST(NormalEstimation2DTest, StraightLineGeometry) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = 2, "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  auto range_data = sensor::RangeData();
  range_data.returns.push_back({Eigen::Vector3f{-1.f, 1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{0.f, 1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{1.f, 1.f, 0.f}});
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  std::vector<float> normals;
  normals = EstimateNormals(range_data, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, -M_PI_2, 1e-4);
  }
  normals.clear();
  range_data.returns.clear();
  range_data.returns.push_back({Eigen::Vector3f{1.f, 1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{1.f, 0.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{1.f, -1.f, 0.f}});
  normals = EstimateNormals(range_data, options);
  for (const float normal : normals) {
    EXPECT_NEAR(std::abs(normal), M_PI, 1e-4);
  }

  normals.clear();
  range_data.returns.clear();
  range_data.returns.push_back({Eigen::Vector3f{1.f, -1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{0.f, -1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{-1.f, -1.f, 0.f}});
  normals = EstimateNormals(range_data, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, M_PI_2, 1e-4);
  }

  normals.clear();
  range_data.returns.clear();
  range_data.returns.push_back({Eigen::Vector3f{-1.f, -1.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{-1.f, 0.f, 0.f}});
  range_data.returns.push_back({Eigen::Vector3f{-1.f, 1.f, 0.f}});
  normals = EstimateNormals(range_data, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, 0, 1e-4);
  }
}

class CircularGeometry2DTest : public TestWithParam<int> {};

TEST_P(CircularGeometry2DTest, NumSamplesPerNormal) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = " +
      std::to_string(GetParam()) +
      ", "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  auto range_data = sensor::RangeData();
  const size_t num_angles = 100;
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                             static_cast<double>(num_angles) * 2. * M_PI -
                         M_PI;
    range_data.returns.push_back(
        {Eigen::Vector3d{std::cos(angle), std::sin(angle), 0.}.cast<float>()});
  }
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  std::vector<float> normals;
  normals = EstimateNormals(range_data, options);
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                         static_cast<double>(num_angles) * 2. * M_PI;
    EXPECT_NEAR(common::NormalizeAngleDifference(normals[angle_idx] - angle),
                0.0, 2.0 * M_PI / num_angles * GetParam() / 2.0 + 1e-4);
  }
}

INSTANTIATE_TEST_CASE_P(InstantiationName, CircularGeometry2DTest,
                        ::testing::Values(1, 2, 4, 5, 8));

}  // namespace
}  // namespace mapping
}  // namespace cartographer
