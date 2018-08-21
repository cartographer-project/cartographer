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

#include "cartographer/pose_graph/node/pose_3d.h"

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

using testing::ParseProto;

constexpr char kExpectedNode[] = R"PROTO(
  id { object_id: "bumpy_world" timestamp: 1 }
  constant: true
  parameters {
    pose_3d {
      translation { x: 1 y: 2 z: 3 }
      translation_parameterization: FIX_Z
      rotation: { w: 0 x: 1 y: 2 z: 3 }
    }
  }
)PROTO";

TEST(Pose3DTest, ToProto) {
  Pose3D pose_3d({"bumpy_world", common::FromUniversal(1)}, true,
                 ParseProto<proto::Pose3D>(R"(
                   translation { x: 1 y: 2 z: 3 }
                   translation_parameterization: FIX_Z
                   rotation: { w: 0 x: 1 y: 2 z: 3 }
                   rotation_parameterization: NONE
                 )"));
  EXPECT_THAT(pose_3d.ToProto(), testing::EqualsProto(kExpectedNode));
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
