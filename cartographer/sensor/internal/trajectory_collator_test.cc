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

#include "cartographer/sensor/internal/trajectory_collator.h"

#include <array>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/test_helpers.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

using testing::CollatorInput;
using testing::CollatorOutput;

TEST(TrajectoryCollator, OrderingMultipleTrajectories) {
  const int kTrajectoryId[] = {4, 7};
  const std::array<std::string, 2> kSensorId = {{"my_points", "some_imu"}};

  std::vector<CollatorInput> input_data;

  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[0], kSensorId[0], 0));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 0));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[1], kSensorId[1], 0));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[0], kSensorId[1], 0));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 100));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[0], kSensorId[0], 50));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[0], kSensorId[1], 60));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 150));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[1], kSensorId[1], 120));

  std::vector<CollatorOutput> received;
  TrajectoryCollator collator;
  collator.AddTrajectory(
      kTrajectoryId[0],
      absl::flat_hash_set<std::string>(kSensorId.begin(), kSensorId.end()),
      [&received, kTrajectoryId](const std::string& sensor_id,
                                 std::unique_ptr<Data> data) {
        received.push_back(CollatorOutput(kTrajectoryId[0], data->GetSensorId(),
                                          data->GetTime()));
      });
  collator.AddTrajectory(
      kTrajectoryId[1],
      absl::flat_hash_set<std::string>(kSensorId.begin(), kSensorId.end()),
      [&received, kTrajectoryId](const std::string& sensor_id,
                                 std::unique_ptr<Data> data) {
        received.push_back(CollatorOutput(kTrajectoryId[1], data->GetSensorId(),
                                          data->GetTime()));
      });

  // Send each sensor_id once to establish a common start time.
  input_data[0].MoveToCollator(&collator);
  input_data[1].MoveToCollator(&collator);
  EXPECT_EQ(0, received.size());
  input_data[2].MoveToCollator(&collator);
  input_data[3].MoveToCollator(&collator);
  EXPECT_EQ(2, received.size());
  EXPECT_EQ(input_data[1].expected_output, received[0]);
  EXPECT_EQ(input_data[0].expected_output, received[1]);

  // Does not wait for other trajectory.
  input_data[4].MoveToCollator(&collator);
  EXPECT_EQ(3, received.size());
  EXPECT_EQ(input_data[2].expected_output, received[2]);

  input_data[5].MoveToCollator(&collator);
  EXPECT_EQ(4, received.size());
  EXPECT_EQ(input_data[3].expected_output, received[3]);
  input_data[6].MoveToCollator(&collator);
  EXPECT_EQ(5, received.size());
  EXPECT_EQ(input_data[5].expected_output, received[4]);

  // Sorts different sensors.
  input_data[7].MoveToCollator(&collator);
  EXPECT_EQ(5, received.size());
  input_data[8].MoveToCollator(&collator);
  EXPECT_EQ(7, received.size());
  EXPECT_EQ(input_data[4].expected_output, received[5]);
  EXPECT_EQ(input_data[8].expected_output, received[6]);

  EXPECT_FALSE(collator.GetBlockingTrajectoryId().has_value());

  collator.FinishTrajectory(kTrajectoryId[0]);
  collator.FinishTrajectory(kTrajectoryId[1]);
  collator.Flush();
  ASSERT_EQ(input_data.size(), received.size());
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
