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
  for (const RangefinderPoint& point : range_data.returns) {
    *proto.add_returns() = ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const RangefinderPoint& point : range_data.misses) {
    *proto.add_misses() = ToProto(point);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  PointCloud returns;
  if (proto.returns_size() > 0) {
    returns.reserve(proto.returns().size());
    for (const auto& point_proto : proto.returns()) {
      returns.push_back(FromProto(point_proto));
    }
  } else {
    returns.reserve(proto.returns_legacy().size());
    for (const auto& point_proto : proto.returns_legacy()) {
      returns.push_back({transform::ToEigen(point_proto)});
    }
  }
  PointCloud misses;
  if (proto.misses_size() > 0) {
    misses.reserve(proto.misses().size());
    for (const auto& point_proto : proto.misses()) {
      misses.push_back(FromProto(point_proto));
    }
  } else {
    misses.reserve(proto.misses_legacy().size());
    for (const auto& point_proto : proto.misses_legacy()) {
      misses.push_back({transform::ToEigen(point_proto)});
    }
  }
  return RangeData{transform::ToEigen(proto.origin()), returns, misses};
}

}  // namespace sensor
}  // namespace cartographer
