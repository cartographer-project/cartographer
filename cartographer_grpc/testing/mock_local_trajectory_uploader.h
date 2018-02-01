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

#ifndef CARTOGRAPHER_GRPC_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H
#define CARTOGRAPHER_GRPC_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H

#include "cartographer_grpc/local_trajectory_uploader.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace testing {

class MockLocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  MOCK_METHOD1(DoEnqueueDataRequest, void(google::protobuf::Message *));
  void EnqueueDataRequest(
      std::unique_ptr<google::protobuf::Message> data_request) override {
    DoEnqueueDataRequest(data_request.get());
  }
  MOCK_METHOD3(
      AddTrajectory,
      void(int, const std::set<SensorId> &,
           const cartographer::mapping::proto::TrajectoryBuilderOptions &));
  MOCK_METHOD1(FinishTrajectory, void(int));
  MOCK_CONST_METHOD1(GetLocalSlamResultSensorId, SensorId(int));
};

}  // namespace testing
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_TESTING_MOCK_LOCAL_TRAJECTORY_UPLOADER_H
