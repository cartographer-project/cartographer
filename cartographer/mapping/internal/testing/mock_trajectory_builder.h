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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_TRAJECTORY_BUILDER_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace testing {

class MockTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  MockTrajectoryBuilder() = default;
  ~MockTrajectoryBuilder() override = default;

  MOCK_METHOD2(AddSensorData,
               void(const std::string &, const sensor::TimedPointCloudData &));
  MOCK_METHOD2(AddSensorData,
               void(const std::string &, const sensor::ImuData &));
  MOCK_METHOD2(AddSensorData,
               void(const std::string &, const sensor::OdometryData &));
  MOCK_METHOD2(AddSensorData,
               void(const std::string &, const sensor::FixedFramePoseData &));
  MOCK_METHOD2(AddSensorData,
               void(const std::string &, const sensor::LandmarkData &));

  // Some of the platforms we run on may ship with a version of gmock which does
  // not yet support move-only types.
  MOCK_METHOD1(DoAddLocalSlamResultData, void(mapping::LocalSlamResultData *));
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    DoAddLocalSlamResultData(local_slam_result_data.get());
  }
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_TRAJECTORY_BUILDER_H_
