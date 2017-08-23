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

#include "cartographer/sensor/absolute_pose_data.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::AbsolutePoseData ToProto(const AbsolutePoseData& absolute_pose_data) {
  proto::AbsolutePoseData proto;
  proto.set_timestamp(common::ToUniversal(absolute_pose_data.time));
  *proto.mutable_pose() = transform::ToProto(absolute_pose_data.pose);
  return proto;
}

AbsolutePoseData FromProto(const proto::AbsolutePoseData& proto) {
  return AbsolutePoseData{common::FromUniversal(proto.timestamp()),
                          transform::ToRigid3(proto.pose())};
}

}  // namespace sensor
}  // namespace cartographer
