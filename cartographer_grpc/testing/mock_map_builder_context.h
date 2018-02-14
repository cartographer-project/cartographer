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

#ifndef CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
#define CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H

#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer_grpc/map_builder_context_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace testing {

class MockMapBuilderContext : public MapBuilderContextInterface {
 public:
  MOCK_METHOD0(map_builder, cartographer::mapping::MapBuilderInterface &());
  MOCK_METHOD0(sensor_data_queue,
               cartographer::common::BlockingQueue<
                   std::unique_ptr<MapBuilderContextInterface::Data>> &());
  MOCK_METHOD0(GetLocalSlamResultCallbackForSubscriptions,
               cartographer::mapping::TrajectoryBuilderInterface::
                   LocalSlamResultCallback());
  MOCK_METHOD1(AddSensorDataToTrajectory,
               void(const MapBuilderContextInterface::Data &));
  MOCK_METHOD2(SubscribeLocalSlamResults,
               MapBuilderContextInterface::SubscriptionId(
                   int,
                   MapBuilderContextInterface::LocalSlamSubscriptionCallback));
  MOCK_METHOD1(UnsubscribeLocalSlamResults,
               void(const MapBuilderContextInterface::SubscriptionId &));
  MOCK_METHOD1(NotifyFinishTrajectory, void(int));
  MOCK_METHOD3(DoProcessLocalSlamResultData,
               cartographer::mapping::LocalSlamResultData *(
                   const std::string &, cartographer::common::Time,
                   const cartographer::mapping::proto::LocalSlamResultData &));
  std::unique_ptr<cartographer::mapping::LocalSlamResultData>
  ProcessLocalSlamResultData(
      const std::string &sensor_id, cartographer::common::Time time,
      const cartographer::mapping::proto::LocalSlamResultData &proto) override {
    return std::unique_ptr<cartographer::mapping::LocalSlamResultData>(
        DoProcessLocalSlamResultData(sensor_id, time, proto));
  }
  MOCK_METHOD0(local_trajectory_uploader, LocalTrajectoryUploaderInterface *());

  MOCK_METHOD2(DoEnqueueSensorData, void(int, cartographer::sensor::Data *));
  void EnqueueSensorData(
      int trajectory_id,
      std::unique_ptr<cartographer::sensor::Data> data) override {
    DoEnqueueSensorData(trajectory_id, data.get());
  }
  MOCK_METHOD3(DoEnqueueLocalSlamResultData,
               void(int, const std::string &,
                    cartographer::mapping::LocalSlamResultData *));
  void EnqueueLocalSlamResultData(
      int trajectory_id, const std::string &sensor_id,
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
          local_slam_result_data) override {
    DoEnqueueLocalSlamResultData(trajectory_id, sensor_id,
                                 local_slam_result_data.get());
  }
};

}  // namespace testing
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
