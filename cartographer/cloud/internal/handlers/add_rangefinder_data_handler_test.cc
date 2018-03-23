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

#include "cartographer/cloud/internal/handlers/add_rangefinder_data_handler.h"
#include "cartographer/cloud/internal/testing/handler_test.h"
#include "cartographer/cloud/internal/testing/test_helpers.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

using ::testing::_;
using ::testing::Eq;
using ::testing::Pointee;
using ::testing::Truly;

const std::string kMessage = R"(
  sensor_metadata {
    trajectory_id: 1
    sensor_id: "sensor_id"
  }
  timed_point_cloud_data {
    timestamp: 2
    origin {
      x: 3.f y: 4.f z: 5.f
    }
    point_data {
      x: 6.f y: 7.f z: 8.f t: 9.f
    }
  })";

using AddRangefinderDataHandlerTest =
    testing::HandlerTest<AddRangefinderDataSignature,
                         AddRangefinderDataHandler>;

TEST_F(AddRangefinderDataHandlerTest, NoLocalSlamUploader) {
  proto::AddRangefinderDataRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  EXPECT_CALL(*mock_map_builder_context_,
              DoEnqueueSensorData(
                  Eq(request.sensor_metadata().trajectory_id()),
                  Pointee(Truly(testing::BuildDataPredicateEquals(request)))));
  test_server_->SendWrite(request);
  test_server_->SendWritesDone();
  test_server_->SendFinish();
}

}  // namespace
}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
