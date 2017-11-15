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

#include "cartographer/sensor/collator.h"

#include <array>
#include <memory>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(Collator, Ordering) {
  const std::array<std::string, 4> kSensorId = {
      {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}};
  DispatchableRangefinderData zero(common::FromUniversal(0),
                                   Eigen::Vector3f::Zero(), {});
  DispatchableRangefinderData first(common::FromUniversal(100),
                                    Eigen::Vector3f::Zero(), {});
  DispatchableRangefinderData second(common::FromUniversal(200),
                                     Eigen::Vector3f::Zero(), {});
  ImuData third{common::FromUniversal(300)};
  DispatchableRangefinderData fourth(common::FromUniversal(400),
                                     Eigen::Vector3f::Zero(), {});
  DispatchableRangefinderData fifth(common::FromUniversal(500),
                                    Eigen::Vector3f::Zero(), {});
  OdometryData sixth{common::FromUniversal(600),
                     transform::Rigid3d::Identity()};

  std::vector<std::pair<std::string, common::Time>> received;
  Collator collator;
  collator.AddTrajectory(
      0, std::unordered_set<std::string>(kSensorId.begin(), kSensorId.end()),
      [&received](const std::string& sensor_id, std::unique_ptr<Data> data) {
        received.push_back(std::make_pair(sensor_id, data->GetTime()));
      });

  constexpr int kTrajectoryId = 0;

  // Establish a common start time.
  collator.AddSensorData(
      kTrajectoryId, kSensorId[0],
      common::make_unique<DispatchableRangefinderData>(zero));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[1],
      common::make_unique<DispatchableRangefinderData>(zero));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[2],
      common::make_unique<DispatchableRangefinderData>(zero));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[3],
      common::make_unique<DispatchableRangefinderData>(zero));

  collator.AddSensorData(
      kTrajectoryId, kSensorId[0],
      common::make_unique<DispatchableRangefinderData>(first));
  collator.AddSensorData(kTrajectoryId, kSensorId[3], MakeDispatchable(sixth));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[0],
      common::make_unique<DispatchableRangefinderData>(fourth));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[1],
      common::make_unique<DispatchableRangefinderData>(second));
  collator.AddSensorData(
      kTrajectoryId, kSensorId[1],
      common::make_unique<DispatchableRangefinderData>(fifth));
  collator.AddSensorData(kTrajectoryId, kSensorId[2], MakeDispatchable(third));

  ASSERT_EQ(7, received.size());
  EXPECT_EQ(100, common::ToUniversal(received[4].second));
  EXPECT_EQ(kSensorId[0], received[4].first);
  EXPECT_EQ(200, common::ToUniversal(received[5].second));
  EXPECT_EQ(kSensorId[1], received[5].first);
  EXPECT_EQ(300, common::ToUniversal(received[6].second));
  EXPECT_EQ(kSensorId[2], received[6].first);

  collator.Flush();

  ASSERT_EQ(10, received.size());
  EXPECT_EQ(kSensorId[0], received[7].first);
  EXPECT_EQ(500, common::ToUniversal(received[8].second));
  EXPECT_EQ(kSensorId[1], received[8].first);
  EXPECT_EQ(600, common::ToUniversal(received[9].second));
  EXPECT_EQ(kSensorId[3], received[9].first);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
