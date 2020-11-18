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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H

#include "cartographer/cloud/internal/local_trajectory_uploader.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace testing {

class MockLocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  MOCK_METHOD1(DoEnqueueSensorData, void(proto::SensorData *));
  void EnqueueSensorData(
      std::unique_ptr<proto::SensorData> data_request) override {
    DoEnqueueSensorData(data_request.get());
  }
  MOCK_METHOD0(Start, void());
  MOCK_METHOD0(Shutdown, void());
  MOCK_METHOD4(AddTrajectory,
               grpc::Status(const std::string &, int,
                            const std::set<SensorId> &,
                            const mapping::proto::TrajectoryBuilderOptions &));
  MOCK_METHOD2(FinishTrajectory, grpc::Status(const std::string &, int));
  MOCK_CONST_METHOD1(GetLocalSlamResultSensorId, SensorId(int));
};

}  // namespace testing
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H
