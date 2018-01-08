/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/sensor/fixed_frame_pose_data.h"

#include "cartographer/common/optional.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::FixedFramePoseData ToProto(const FixedFramePoseData& pose_data) {
  proto::FixedFramePoseData proto;
  proto.set_timestamp(common::ToUniversal(pose_data.time));
  if (pose_data.pose.has_value()) {
    *proto.mutable_pose() = transform::ToProto(pose_data.pose.value());
  }
  return proto;
}

FixedFramePoseData FromProto(const proto::FixedFramePoseData& proto) {
  return FixedFramePoseData{common::FromUniversal(proto.timestamp()),
                            proto.has_pose()
                                ? common::optional<transform::Rigid3d>(
                                      transform::ToRigid3(proto.pose()))
                                : common::optional<transform::Rigid3d>()};
}

}  // namespace sensor
}  // namespace cartographer
