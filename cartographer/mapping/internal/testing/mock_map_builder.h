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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using testing::_;

namespace cartographer {
namespace mapping {
namespace testing {

class MockMapBuilder : public mapping::MapBuilderInterface {
 public:
  MOCK_METHOD3(
      AddTrajectoryBuilder,
      int(const std::set<SensorId> &expected_sensor_ids,
          const mapping::proto::TrajectoryBuilderOptions &trajectory_options,
          mapping::MapBuilderInterface::LocalSlamResultCallback
              local_slam_result_callback));
  MOCK_METHOD1(AddTrajectoryForDeserialization,
               int(const mapping::proto::TrajectoryBuilderOptionsWithSensorIds
                       &options_with_sensor_ids_proto));
  MOCK_CONST_METHOD1(GetTrajectoryBuilder,
                     mapping::TrajectoryBuilderInterface *(int trajectory_id));
  MOCK_METHOD1(FinishTrajectory, void(int trajectory_id));
  MOCK_METHOD2(SubmapToProto,
               std::string(const mapping::SubmapId &,
                           mapping::proto::SubmapQuery::Response *));
  MOCK_METHOD1(SerializeState, void(io::ProtoStreamWriterInterface *));
  MOCK_METHOD2(LoadState, void(io::ProtoStreamReaderInterface *, bool));
  MOCK_CONST_METHOD0(num_trajectory_builders, int());
  MOCK_METHOD0(pose_graph, mapping::PoseGraphInterface *());
  MOCK_CONST_METHOD0(
      GetAllTrajectoryBuilderOptions,
      const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
          &());
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_
