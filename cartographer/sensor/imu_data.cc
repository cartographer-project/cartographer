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

#include "cartographer/sensor/imu_data.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::ImuData ToProto(const ImuData& imu_data) {
  proto::ImuData proto;
  proto.set_timestamp(common::ToUniversal(imu_data.time));
  *proto.mutable_linear_acceleration() =
      transform::ToProto(imu_data.linear_acceleration);
  *proto.mutable_angular_velocity() =
      transform::ToProto(imu_data.angular_velocity);
  return proto;
}

ImuData FromProto(const proto::ImuData& proto) {
  return ImuData{common::FromUniversal(proto.timestamp()),
                 transform::ToEigen(proto.linear_acceleration()),
                 transform::ToEigen(proto.angular_velocity())};
}

}  // namespace sensor
}  // namespace cartographer
