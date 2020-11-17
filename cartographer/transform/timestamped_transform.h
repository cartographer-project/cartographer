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

#ifndef CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "cartographer/common/time.h"
#include "cartographer/transform/proto/timestamped_transform.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

struct TimestampedTransform {
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform FromProto(const proto::TimestampedTransform& proto);
proto::TimestampedTransform ToProto(const TimestampedTransform& transform);

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

template <typename T>
transform::Rigid3<T> InterpolateTransform(const transform::Rigid3<T>& start,
                                          const transform::Rigid3<T>& end,
                                          const double factor) {
  const Eigen::Matrix<T, 3, 1> origin =
      start.translation() + (end.translation() - start.translation()) * factor;
  const Eigen::Quaternion<T> rotation =
      start.rotation().slerp(T(factor), end.rotation());
  return transform::Rigid3<T>(origin, rotation);
}

template <typename T>
transform::Rigid3<T> InterpolateTransform(const transform::Rigid3<T>& start,
                                          const transform::Rigid3<T>& end,
                                          const common::Time time_start,
                                          const common::Time time_end,
                                          const common::Time time) {
  CHECK_LE(time_start, time);
  CHECK_GE(time_end, time);
  const double duration = common::ToSeconds(time_end - time_start);
  const double factor = common::ToSeconds(time - time_start) / duration;
  return InterpolateTransform(start, end, factor);
}

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
