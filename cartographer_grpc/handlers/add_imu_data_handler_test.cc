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

#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/dispatchable.h"
#include "cartographer_grpc/framework/testing/rpc_handler_test_server.h"
#include "cartographer_grpc/testing/mock_local_trajectory_uploader.h"
#include "cartographer_grpc/testing/mock_map_builder_context.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace handlers {
namespace {

using ::testing::_;
using ::testing::Eq;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::Test;
using ::testing::Truly;

const std::string kMessage = R"PROTO(
  sensor_metadata {
    trajectory_id: 1
    sensor_id: "sensor_id"
  }
  imu_data {
    timestamp: 2
    linear_acceleration {
      x: 3
      y: 4
      z: 5
    }
    angular_velocity {
      x: 6
      y: 7
      z: 8
    }
  })PROTO";

using DataPredicateType =
    std::function<bool(const cartographer::sensor::Data &)>;
using ProtoPredicateType =
    std::function<bool(const google::protobuf::Message &)>;

class AddImuDataHandlerTest : public Test {
 public:
  void SetUp() override {
    test_server_ = cartographer::common::make_unique<
        framework::testing::RpcHandlerTestServer<AddImuDataHandler>>(
        cartographer::common::make_unique<testing::MockMapBuilderContext>());
    mock_map_builder_context_ =
        test_server_
            ->GetUnsynchronizedContext<testing::MockMapBuilderContext>();
    mock_local_trajectory_uploader_ = cartographer::common::make_unique<
        testing::MockLocalTrajectoryUploader>();
    EXPECT_TRUE(
        google::protobuf::TextFormat::ParseFromString(kMessage, &request_));
  }

  void SetNoLocalTrajectoryUploader() {
    EXPECT_CALL(*mock_map_builder_context_, local_trajectory_uploader())
        .WillOnce(Return(nullptr));
  }

  void SetMockLocalTrajectoryUploader() {
    EXPECT_CALL(*mock_map_builder_context_, local_trajectory_uploader())
        .WillRepeatedly(Return(mock_local_trajectory_uploader_.get()));
  }

 protected:
  std::unique_ptr<framework::testing::RpcHandlerTestServer<AddImuDataHandler>>
      test_server_;
  testing::MockMapBuilderContext *mock_map_builder_context_;
  std::unique_ptr<testing::MockLocalTrajectoryUploader>
      mock_local_trajectory_uploader_;
  proto::AddImuDataRequest request_;
};

DataPredicateType BuildDataPredicateEquals(
    const proto::AddImuDataRequest &proto) {
  return [proto](const cartographer::sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const cartographer::sensor::Dispatchable<
            cartographer::sensor::ImuData> *>(&data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               cartographer::sensor::ToProto(dispatchable->data()),
               proto.imu_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

ProtoPredicateType BuildProtoPredicateEquals(
    const google::protobuf::Message *proto) {
  return [proto](const google::protobuf::Message &message) {
    return google::protobuf::util::MessageDifferencer::Equals(*proto, message);
  };
}

TEST_F(AddImuDataHandlerTest, NoLocalSlamUploader) {
  SetNoLocalTrajectoryUploader();
  EXPECT_CALL(
      *mock_map_builder_context_,
      DoEnqueueSensorData(Eq(request_.sensor_metadata().trajectory_id()),
                          Pointee(Truly(BuildDataPredicateEquals(request_)))));
  test_server_->SendWrite(request_);
  test_server_->SendWritesDone();
  test_server_->SendFinish();
}

TEST_F(AddImuDataHandlerTest, WithMockLocalSlamUploader) {
  SetMockLocalTrajectoryUploader();
  EXPECT_CALL(
      *mock_map_builder_context_,
      DoEnqueueSensorData(Eq(request_.sensor_metadata().trajectory_id()),
                          Pointee(Truly(BuildDataPredicateEquals(request_)))));
  EXPECT_CALL(*mock_local_trajectory_uploader_,
              DoEnqueueDataRequest(
                  Pointee(Truly(BuildProtoPredicateEquals(&request_)))));
  test_server_->SendWrite(request_);
  test_server_->SendWritesDone();
  test_server_->SendFinish();
}

}  // namespace
}  // namespace handlers
}  // namespace cartographer_grpc
