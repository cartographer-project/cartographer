/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
  };
}

TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform) {
  return TimedRangeData{
      transform * range_data.origin,
      TransformTimedPointCloud(range_data.returns, transform),
      TransformTimedPointCloud(range_data.misses, transform),
  };
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin,
                   CropPointCloud(range_data.returns, min_z, max_z),
                   CropPointCloud(range_data.misses, min_z, max_z)};
}

TimedRangeData CropTimedRangeData(const TimedRangeData& range_data,
                                  const float min_z, const float max_z) {
  return TimedRangeData{range_data.origin,
                        CropTimedPointCloud(range_data.returns, min_z, max_z),
                        CropTimedPointCloud(range_data.misses, min_z, max_z)};
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  proto.mutable_returns()->Reserve(range_data.returns.size());
  for (const Eigen::Vector3f& point : range_data.returns) {
    *proto.add_returns() = transform::ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const Eigen::Vector3f& point : range_data.misses) {
    *proto.add_misses() = transform::ToProto(point);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  PointCloud returns;
  returns.reserve(proto.returns().size());
  std::transform(
      proto.returns().begin(), proto.returns().end(),
      std::back_inserter(returns),
      static_cast<Eigen::Vector3f (*)(const transform::proto::Vector3f&)>(
          transform::ToEigen));
  PointCloud misses;
  misses.reserve(proto.misses().size());
  std::transform(
      proto.misses().begin(), proto.misses().end(), std::back_inserter(misses),
      static_cast<Eigen::Vector3f (*)(const transform::proto::Vector3f&)>(
          transform::ToEigen));
  return RangeData{transform::ToEigen(proto.origin()), returns, misses};
}

}  // namespace sensor
}  // namespace cartographer
