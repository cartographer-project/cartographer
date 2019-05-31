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

#include "cartographer/cloud/internal/handlers/get_landmark_poses_handler.h"

#include "cartographer/cloud/internal/testing/handler_test.h"
#include "cartographer/cloud/internal/testing/test_helpers.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

using ::cartographer::transform::Rigid3d;
using ::testing::_;
using ::testing::Eq;
using ::testing::Pointee;
using ::testing::Truly;

const std::string kMessage = R"(
  landmark_poses {
    landmark_id: "landmark_1"
    global_pose {
      translation {
        x: 1 y: 2 z: 3
      }
      rotation {
        w: 1 x: 0 y: 0 z: 0
      }
    }
  }
  landmark_poses {
    landmark_id: "landmark_2"
    global_pose {
      translation {
        x: 3 y: 2 z: 1
      }
      rotation {
        w: 0 x: 1 y: 0 z: 0
      }
    }
  }
)";

using GetLandmarkPosesHandlerTest =
    testing::HandlerTest<GetLandmarkPosesSignature, GetLandmarkPosesHandler>;

TEST_F(GetLandmarkPosesHandlerTest, NoLocalSlamUploader) {
  std::map<std::string, Rigid3d> landmark_poses{
      {"landmark_1", Rigid3d(Eigen::Vector3d(1., 2., 3.),
                             Eigen::Quaterniond(1., 0., 0., 0.))},
      {"landmark_2", Rigid3d(Eigen::Vector3d(3., 2., 1.),
                             Eigen::Quaterniond(0., 1., 0., 0.))}};
  EXPECT_CALL(*mock_pose_graph_, GetLandmarkPoses())
      .WillOnce(::testing::Return(landmark_poses));
  test_server_->SendWrite(google::protobuf::Empty());

  proto::GetLandmarkPosesResponse expected_response;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(
      kMessage, &expected_response));
  EXPECT_THAT(
      test_server_->response(),
      ::testing::Truly(testing::BuildProtoPredicateEquals(&expected_response)));
}

}  // namespace
}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
