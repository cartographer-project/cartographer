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

#include "cartographer/pose_graph/pose_2d.h"

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace pose_graph {
namespace {

constexpr char kExpectedNode[] = R"PROTO(
  id { object_id: "flat_world" }
  constant: true
  parameters {
    pose_2d {
      translation { x: 1 y: 2 }
      rotation: 5
    }
  }
)PROTO";

// TODO(pifon): Move this to a separate header.
MATCHER_P(EqualsProto, expected_proto_string, "") {
  using ConstProtoType = typename std::remove_reference<decltype(arg)>::type;

  typename std::remove_cv<ConstProtoType>::type expected_proto;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(
      expected_proto_string, &expected_proto));
  return google::protobuf::util::MessageDifferencer::Equals(arg,
                                                            expected_proto);
}

TEST(Pose2DTest, SerializeToProto) {
  Pose2D pose_2d("flat_world", true, Eigen::Vector2d(1., 2.), 5.);
  EXPECT_THAT(pose_2d.ToProto(), EqualsProto(kExpectedNode));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
