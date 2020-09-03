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

#include "cartographer/cloud/internal/local_trajectory_uploader.h"

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using SensorId = ::cartographer::mapping::TrajectoryBuilderInterface::SensorId;

namespace cartographer {
namespace cloud {
namespace {

constexpr char kClientId[] = "CLIENT_ID";
const SensorId kImuSensorId{SensorId::SensorType::IMU, "imu"};
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const int kLocalTrajectoryId = 3;

TEST(LocalTrajectoryUploaderTest, HandlesInvalidUplink) {
  auto uploader = CreateLocalTrajectoryUploader("invalid-uplink-address:50051",
                                                /*batch_size=*/1, false, false);
  uploader->Start();
  mapping::proto::TrajectoryBuilderOptions options;
  auto status = uploader->AddTrajectory(
      kClientId, kLocalTrajectoryId, {kRangeSensorId, kImuSensorId}, options);
  EXPECT_FALSE(status.ok());
  auto sensor_data = absl::make_unique<proto::SensorData>();
  sensor_data->mutable_sensor_metadata()->set_client_id(kClientId);
  sensor_data->mutable_sensor_metadata()->set_sensor_id(kImuSensorId.id);
  sensor_data->mutable_sensor_metadata()->set_trajectory_id(kLocalTrajectoryId);
  sensor_data->mutable_imu_data()->set_timestamp(1);
  uploader->EnqueueSensorData(std::move(sensor_data));
  auto sensor_id = uploader->GetLocalSlamResultSensorId(kLocalTrajectoryId);
  EXPECT_THAT(sensor_id.id, ::testing::Not(::testing::IsEmpty()));
  status = uploader->FinishTrajectory(kClientId, kLocalTrajectoryId);
  EXPECT_FALSE(status.ok());
  uploader->Shutdown();
}

}  // namespace
}  // namespace cloud
}  // namespace cartographer
