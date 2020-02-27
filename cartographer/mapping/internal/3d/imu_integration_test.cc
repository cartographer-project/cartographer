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

#include "cartographer/mapping/internal/3d/imu_integration.h"

#include <tuple>

#include "gmock/gmock.h"

#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {
namespace {
constexpr double kPrecision = 1e-8;

TEST(IMUIntegrationTest, ZeroIntegration) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 0);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;
  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);
    imu_data_deque.push_back(
        {current_time, initial_gravity_acceleration, initial_angular_velocity});
  }

  auto it = --imu_data_deque.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_deque.cbegin());
    --it;
  }
  IntegrateImuWithTranslationResult<double> result =
      IntegrateImuWithTranslation(imu_data_deque, start_time, current_time,
                                  &it);

  EXPECT_NEAR(0.f, result.delta_velocity.norm(), kPrecision);
  EXPECT_NEAR(0.f, result.delta_translation.norm(), kPrecision);
  EXPECT_NEAR(
      0., result.delta_rotation.angularDistance(Eigen::Quaterniond::Identity()),
      kPrecision);
}

TEST(IMUIntegrationTest, ConstantAcceleration) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 10);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time start_time = common::FromUniversal(0);
  common::Time current_time = start_time;
  std::deque<sensor::ImuData> imu_data_deque;
  imu_data_deque.push_back(
      {current_time, initial_gravity_acceleration, initial_angular_velocity});
  for (int i = 0; i < 100; ++i) {
    current_time += common::FromSeconds(1);
    imu_data_deque.push_back(
        {current_time, initial_gravity_acceleration, initial_angular_velocity});

    auto it = --imu_data_deque.cend();
    while (it->time > start_time) {
      CHECK(it != imu_data_deque.cbegin());
      --it;
    }
    IntegrateImuWithTranslationResult<double> result =
        IntegrateImuWithTranslation(imu_data_deque, start_time, current_time,
                                    &it);
    double delta_time = common::ToSeconds(current_time - start_time);
    Eigen::Vector3d expected_velocity =
        delta_time * initial_gravity_acceleration;
    Eigen::Vector3d expected_translation =
        0.5 * delta_time * delta_time * initial_gravity_acceleration;
    EXPECT_NEAR(0.f, (result.delta_velocity - expected_velocity).norm(),
                kPrecision);
    EXPECT_NEAR(0.f, (result.delta_translation - expected_translation).norm(),
                delta_time * 10);
    EXPECT_NEAR(
        0.,
        result.delta_rotation.angularDistance(Eigen::Quaterniond::Identity()),
        kPrecision);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
