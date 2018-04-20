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

#include "cartographer/cloud/internal/handlers/set_landmark_pose_handler.h"
#include "cartographer/cloud/internal/testing/handler_test.h"
#include "cartographer/cloud/internal/testing/test_helpers.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

const std::string kMessage = R"(
  landmark_pose {
    landmark_id: "landmark_1"
    global_pose {
      translation {
        x: 1 y: 2 z: 3
      }
      rotation {
        w: 4 x: 5 y: 6 z: 7
      }
    }
  })";

using SetLandmarkPoseHandlerTest =
    testing::HandlerTest<SetLandmarkPoseSignature, SetLandmarkPoseHandler>;

TEST_F(SetLandmarkPoseHandlerTest, SetLandmarkPose) {
  constexpr double kEps = 1e-10;
  proto::SetLandmarkPoseRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  EXPECT_CALL(
      *mock_pose_graph_,
      SetLandmarkPose("landmark_1",
                      transform::IsNearly(
                          transform::Rigid3d(Eigen::Vector3d(1, 2, 3),
                                             Eigen::Quaterniond(4, 5, 6, 7)),
                          kEps)));
  test_server_->SendWrite(request);
}

}  // namespace
}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
