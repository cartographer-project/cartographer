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

#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"

#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/cloud/internal/testing/handler_test.h"
#include "cartographer/cloud/internal/testing/test_helpers.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Eq;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::Test;
using ::testing::Truly;

const std::string kMessage = R"(
    client_id: "CLIENT_ID"
    expected_sensor_ids {
      id: "range_sensor"
      type: RANGE
    }
    expected_sensor_ids {
      id: "imu_sensor"
      type: IMU
    }
    trajectory_builder_options {
      trajectory_builder_2d_options {
        min_range: 20
        max_range: 30
      }
      pure_localization_trimmer {
        max_submaps_to_keep: 3
      }
      initial_trajectory_pose {
        relative_pose {
          translation {
            x: 1 y: 2 z: 3
          }
          rotation {
            w: 4 x: 5 y: 6 z: 7
          }
        }
        to_trajectory_id: 8
        timestamp: 9
      }
    }
  )";

class AddTrajectoryHandlerTest
    : public testing::HandlerTest<AddTrajectorySignature,
                                  AddTrajectoryHandler> {
 public:
  void SetUp() override {
    testing::HandlerTest<AddTrajectorySignature, AddTrajectoryHandler>::SetUp();
    mock_map_builder_ = absl::make_unique<mapping::testing::MockMapBuilder>();
    EXPECT_CALL(*mock_map_builder_context_,
                GetLocalSlamResultCallbackForSubscriptions())
        .WillOnce(Return(nullptr));
    EXPECT_CALL(*mock_map_builder_context_, map_builder())
        .WillOnce(ReturnRef(*mock_map_builder_));
  }

 protected:
  std::set<mapping::TrajectoryBuilderInterface::SensorId> ParseSensorIds(
      const proto::AddTrajectoryRequest &request) {
    std::set<mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids;
    for (const auto &sensor_id : request.expected_sensor_ids()) {
      expected_sensor_ids.insert(cloud::FromProto(sensor_id));
    }
    return expected_sensor_ids;
  }

  std::unique_ptr<mapping::testing::MockMapBuilder> mock_map_builder_;
};

TEST_F(AddTrajectoryHandlerTest, NoLocalSlamUploader) {
  SetNoLocalTrajectoryUploader();
  proto::AddTrajectoryRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  EXPECT_CALL(*mock_map_builder_,
              AddTrajectoryBuilder(ContainerEq(ParseSensorIds(request)),
                                   Truly(testing::BuildProtoPredicateEquals(
                                       &request.trajectory_builder_options())),
                                   _))
      .WillOnce(Return(13));
  EXPECT_CALL(*mock_map_builder_context_,
              RegisterClientIdForTrajectory(Eq("CLIENT_ID"), Eq(13)));
  test_server_->SendWrite(request);
  EXPECT_EQ(test_server_->response().trajectory_id(), 13);
}

TEST_F(AddTrajectoryHandlerTest, WithLocalSlamUploader) {
  SetMockLocalTrajectoryUploader();
  proto::AddTrajectoryRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  EXPECT_CALL(*mock_map_builder_,
              AddTrajectoryBuilder(ContainerEq(ParseSensorIds(request)),
                                   Truly(testing::BuildProtoPredicateEquals(
                                       &request.trajectory_builder_options())),
                                   _))
      .WillOnce(Return(13));
  EXPECT_CALL(*mock_map_builder_context_,
              RegisterClientIdForTrajectory(Eq("CLIENT_ID"), Eq(13)));
  auto upstream_trajectory_builder_options =
      request.trajectory_builder_options();
  // Reproduce the changes to the request as done by the handler.
  upstream_trajectory_builder_options.clear_trajectory_builder_2d_options();
  upstream_trajectory_builder_options.clear_trajectory_builder_3d_options();
  upstream_trajectory_builder_options.clear_pure_localization_trimmer();
  upstream_trajectory_builder_options.clear_initial_trajectory_pose();
  EXPECT_CALL(*mock_local_trajectory_uploader_,
              AddTrajectory(Eq("CLIENT_ID"), Eq(13), ParseSensorIds(request),
                            Truly(testing::BuildProtoPredicateEquals(
                                &upstream_trajectory_builder_options))))
      .WillOnce(Return(grpc::Status::OK));
  test_server_->SendWrite(request);
  EXPECT_EQ(test_server_->response().trajectory_id(), 13);
}

}  // namespace
}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
