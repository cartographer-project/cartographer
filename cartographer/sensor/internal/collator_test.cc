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

#include "cartographer/sensor/internal/collator.h"

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

TEST(Collator, Ordering) {
  const int kTrajectoryId = 0;
  const std::array<std::string, 4> kSensorId = {
      {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}};

  std::vector<CollatorInput> input_data;
  // Send each sensor_id once to establish a common start time.
  input_data.push_back(
      CollatorInput::CreateTimedPointCloudData(kTrajectoryId, kSensorId[0], 0));
  input_data.push_back(
      CollatorInput::CreateTimedPointCloudData(kTrajectoryId, kSensorId[1], 0));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId, kSensorId[2], 0));
  input_data.push_back(
      CollatorInput::CreateOdometryData(kTrajectoryId, kSensorId[3], 0));

  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId, kSensorId[0], 100));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId, kSensorId[1], 200));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId, kSensorId[2], 300));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId, kSensorId[0], 400));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId, kSensorId[1], 500));
  input_data.push_back(
      CollatorInput::CreateOdometryData(kTrajectoryId, kSensorId[3], 600));

  std::vector<CollatorOutput> received;
  Collator collator;
  collator.AddTrajectory(
      kTrajectoryId,
      absl::flat_hash_set<std::string>(kSensorId.begin(), kSensorId.end()),
      [&received, kTrajectoryId](const std::string& sensor_id,
                                 std::unique_ptr<Data> data) {
        received.push_back(CollatorOutput(kTrajectoryId, data->GetSensorId(),
                                          data->GetTime()));
      });

  input_data[0].MoveToCollator(&collator);
  input_data[1].MoveToCollator(&collator);
  input_data[2].MoveToCollator(&collator);
  input_data[3].MoveToCollator(&collator);

  input_data[4].MoveToCollator(&collator);
  input_data[9].MoveToCollator(&collator);
  input_data[7].MoveToCollator(&collator);
  input_data[5].MoveToCollator(&collator);
  input_data[8].MoveToCollator(&collator);
  input_data[6].MoveToCollator(&collator);
  EXPECT_EQ(kTrajectoryId, collator.GetBlockingTrajectoryId().value());

  ASSERT_EQ(7, received.size());
  EXPECT_EQ(input_data[4].expected_output, received[4]);
  EXPECT_EQ(input_data[5].expected_output, received[5]);
  EXPECT_EQ(input_data[6].expected_output, received[6]);

  collator.FinishTrajectory(kTrajectoryId);
  collator.Flush();
  ASSERT_EQ(input_data.size(), received.size());
  for (size_t i = 4; i < input_data.size(); ++i) {
    EXPECT_EQ(input_data[i].expected_output, received[i]);
  }
}

TEST(Collator, OrderingMultipleTrajectories) {
  const int kTrajectoryId[] = {8, 5};
  const std::array<std::string, 2> kSensorId = {{"my_points", "some_imu"}};

  std::vector<CollatorInput> input_data;
  // Send each sensor_id once to establish a common start time.
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[0], kSensorId[0], 0));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[0], kSensorId[1], 0));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 0));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[1], kSensorId[1], 0));

  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[0], kSensorId[0], 100));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[1], kSensorId[1], 200));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[0], kSensorId[1], 300));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 400));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 400));
  input_data.push_back(CollatorInput::CreateTimedPointCloudData(
      kTrajectoryId[1], kSensorId[0], 500));
  input_data.push_back(
      CollatorInput::CreateImuData(kTrajectoryId[1], kSensorId[1], 600));

  std::vector<CollatorOutput> received;
  Collator collator;
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

  input_data[0].MoveToCollator(&collator);
  input_data[1].MoveToCollator(&collator);
  input_data[2].MoveToCollator(&collator);
  input_data[3].MoveToCollator(&collator);

  input_data[4].MoveToCollator(&collator);
  input_data[6].MoveToCollator(&collator);
  EXPECT_EQ(kTrajectoryId[1], collator.GetBlockingTrajectoryId().value());
  input_data[7].MoveToCollator(&collator);
  input_data[8].MoveToCollator(&collator);
  EXPECT_EQ(kTrajectoryId[1], collator.GetBlockingTrajectoryId().value());
  input_data[5].MoveToCollator(&collator);
  EXPECT_EQ(kTrajectoryId[0], collator.GetBlockingTrajectoryId().value());
  input_data[10].MoveToCollator(&collator);
  input_data[9].MoveToCollator(&collator);
  EXPECT_EQ(kTrajectoryId[0], collator.GetBlockingTrajectoryId().value());

  ASSERT_EQ(5, received.size());
  EXPECT_EQ(input_data[4].expected_output, received[4]);

  collator.FinishTrajectory(kTrajectoryId[0]);
  collator.FinishTrajectory(kTrajectoryId[1]);
  collator.Flush();
  ASSERT_EQ(input_data.size(), received.size());
  for (size_t i = 4; i < input_data.size(); ++i) {
    EXPECT_EQ(input_data[i].expected_output, received[i]);
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
