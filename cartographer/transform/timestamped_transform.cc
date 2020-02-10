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

#include "cartographer/transform/timestamped_transform.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace transform {

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time) {
  CHECK_LE(start.time, time);
  CHECK_GE(end.time, time);
  return TimestampedTransform{
      time, InterpolateTransform(start.transform, end.transform, start.time,
                                 end.time, time)};
}

TimestampedTransform FromProto(const proto::TimestampedTransform& proto) {
  return TimestampedTransform{common::FromUniversal(proto.time()),
                              ToRigid3(proto.transform())};
}

proto::TimestampedTransform ToProto(const TimestampedTransform& transform) {
  proto::TimestampedTransform proto;
  proto.set_time(common::ToUniversal(transform.time));
  *proto.mutable_transform() = ToProto(transform.transform);
  return proto;
}

}  // namespace transform
}  // namespace cartographer
