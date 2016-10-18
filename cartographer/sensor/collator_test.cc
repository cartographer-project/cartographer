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
  Data first(common::FromUniversal(100), "horizontal_laser", sensor::LaserFan{});
  Data second(common::FromUniversal(200),"vertical_laser", sensor::LaserFan{});
  Data third(common::FromUniversal(300),"imu", Data::Imu{});
  Data fourth(common::FromUniversal(400),"horizontal_laser", sensor::LaserFan{});
  Data fifth(common::FromUniversal(500),"vertical_laser", sensor::LaserFan{});
  Data sixth(common::FromUniversal(600),"odometry", Data::Odometry{});

  const std::unordered_set<string> frame_ids = {
      "horizontal_laser", "vertical_laser", "imu", "odometry"};
  std::vector<Data> received;
  Collator collator;
  collator.AddTrajectory(
      0, frame_ids,
      [&received](std::unique_ptr<Data> data) {
        received.push_back(*data);
      });

  collator.AddSensorData(0, first.frame_id, common::make_unique<Data>(first));
  collator.AddSensorData(0, sixth.frame_id, common::make_unique<Data>(sixth));
  collator.AddSensorData(0, fourth.frame_id, common::make_unique<Data>(fourth));
  collator.AddSensorData(0, second.frame_id, common::make_unique<Data>(second));
  collator.AddSensorData(0, fifth.frame_id, common::make_unique<Data>(fifth));
  collator.AddSensorData(0, third.frame_id, common::make_unique<Data>(third));

  EXPECT_EQ(3, received.size());
  EXPECT_EQ(100, common::ToUniversal(received[0].time));
  EXPECT_EQ("horizontal_laser", received[0].frame_id);
  EXPECT_EQ(200, common::ToUniversal(received[1].time));
  EXPECT_EQ("vertical_laser", received[1].frame_id);
  EXPECT_EQ(300, common::ToUniversal(received[2].time));
  EXPECT_EQ("imu", received[2].frame_id);

  collator.Flush();

  ASSERT_EQ(6, received.size());
  EXPECT_EQ("horizontal_laser", received[3].frame_id);
  EXPECT_EQ(500, common::ToUniversal(received[4].time));
  EXPECT_EQ("vertical_laser", received[4].frame_id);
  EXPECT_EQ(600, common::ToUniversal(received[5].time));
  EXPECT_EQ("odometry", received[5].frame_id);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
