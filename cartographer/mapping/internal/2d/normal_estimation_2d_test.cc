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

TEST(NormalEstimation2DTest, StraightLineGeometry) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = 2, "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  sensor::RangeData range_data;
  range_data.returns.emplace_back(-1.f, 1.f, 0.f);
  range_data.returns.emplace_back(0.f, 1.f, 0.f);
  range_data.returns.emplace_back(1.f, 1.f, 0.f);
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  std::vector<float> normals;
  EstimateNormals(range_data, &normals, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, -M_PI_2, 1e-4);
  }

  normals.clear();
  range_data.returns.clear();
  range_data.returns.emplace_back(1.f, 1.f, 0.f);
  range_data.returns.emplace_back(1.f, 0.f, 0.f);
  range_data.returns.emplace_back(1.f, -1.f, 0.f);
  EstimateNormals(range_data, &normals, options);
  for (const float normal : normals) {
    EXPECT_NEAR(std::abs(normal), M_PI, 1e-4);
  }

  normals.clear();
  range_data.returns.clear();
  range_data.returns.emplace_back(1.f, -1.f, 0.f);
  range_data.returns.emplace_back(0.f, -1.f, 0.f);
  range_data.returns.emplace_back(-1.f, -1.f, 0.f);
  EstimateNormals(range_data, &normals, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, M_PI_2, 1e-4);
  }

  normals.clear();
  range_data.returns.clear();
  range_data.returns.emplace_back(-1.f, -1.f, 0.f);
  range_data.returns.emplace_back(-1.f, 0.f, 0.f);
  range_data.returns.emplace_back(-1.f, 1.f, 0.f);
  EstimateNormals(range_data, &normals, options);
  for (const float normal : normals) {
    EXPECT_NEAR(normal, 0, 1e-4);
  }
}

TEST(NormalEstimation2DTest, CircularGeometry) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = 2, "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  sensor::RangeData range_data;
  const size_t num_angles = 100;
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                             static_cast<double>(num_angles) * 2. * M_PI -
                         M_PI;
    range_data.returns.emplace_back(std::cos(angle), std::sin(angle), 0.f);
  }
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  std::vector<float> normals;
  EstimateNormals(range_data, &normals, options);
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                         static_cast<double>(num_angles) * 2. * M_PI;
    EXPECT_NEAR(common::NormalizeAngleDifference(normals[angle_idx] - angle),
                0.0, 5e-2);
  }
}

TEST(NormalEstimation2DTest, CircularGeometrySingleSample) {
  const auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "num_normal_samples = 1, "
      "sample_radius = 10.0, "
      "}");
  const proto::NormalEstimationOptions2D options =
      CreateNormalEstimationOptions2D(parameter_dictionary.get());
  sensor::RangeData range_data;
  const size_t num_angles = 100;
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                             static_cast<double>(num_angles) * 2. * M_PI -
                         M_PI;
    range_data.returns.emplace_back(std::cos(angle), std::sin(angle), 0.f);
  }
  range_data.origin.x() = 0.f;
  range_data.origin.y() = 0.f;
  std::vector<float> normals;
  EstimateNormals(range_data, &normals, options);
  for (size_t angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
    const double angle = static_cast<double>(angle_idx) /
                         static_cast<double>(num_angles) * 2. * M_PI;
    EXPECT_NEAR(common::NormalizeAngleDifference(normals[angle_idx] - angle),
                0.0, 5e-2);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
