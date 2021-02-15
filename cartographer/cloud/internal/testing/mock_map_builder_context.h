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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
#define CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_MAP_BUILDER_CONTEXT_H

#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace testing {

class MockMapBuilderContext : public MapBuilderContextInterface {
 public:
  MOCK_METHOD0(map_builder, mapping::MapBuilderInterface &());
  MOCK_METHOD0(
      sensor_data_queue,
      common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>
          &());
  MOCK_METHOD0(GetLocalSlamResultCallbackForSubscriptions,
               mapping::TrajectoryBuilderInterface::LocalSlamResultCallback());
  MOCK_METHOD1(AddSensorDataToTrajectory,
               void(const MapBuilderContextInterface::Data &));
  MOCK_METHOD2(SubscribeLocalSlamResults,
               MapBuilderContextInterface::LocalSlamSubscriptionId(
                   int,
                   MapBuilderContextInterface::LocalSlamSubscriptionCallback));
  MOCK_METHOD1(
      UnsubscribeLocalSlamResults,
      void(const MapBuilderContextInterface::LocalSlamSubscriptionId &));
  MOCK_METHOD1(SubscribeGlobalSlamOptimizations,
               int(GlobalSlamOptimizationCallback));
  MOCK_METHOD1(UnsubscribeGlobalSlamOptimizations, void(int));
  MOCK_METHOD1(NotifyFinishTrajectory, void(int));
  MOCK_METHOD0(local_trajectory_uploader, LocalTrajectoryUploaderInterface *());

  MOCK_METHOD2(DoEnqueueSensorData, void(int, sensor::Data *));
  void EnqueueSensorData(int trajectory_id,
                         std::unique_ptr<sensor::Data> data) override {
    DoEnqueueSensorData(trajectory_id, data.get());
  }
  MOCK_METHOD3(EnqueueLocalSlamResultData,
               void(int, const std::string &,
                    const mapping::proto::LocalSlamResultData &));
  MOCK_METHOD2(RegisterClientIdForTrajectory, void(const std::string &, int));
  MOCK_METHOD2(CheckClientIdForTrajectory, bool(const std::string &, int));
};

}  // namespace testing
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_TESTING_MOCK_MAP_BUILDER_CONTEXT_H
