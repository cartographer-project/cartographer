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

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin, Crop(range_data.returns, min_z, max_z),
                   Crop(range_data.misses, min_z, max_z)};
}

proto::CompressedRangeData ToProto(
    const CompressedRangeData& compressed_range_data) {
  proto::CompressedRangeData proto;
  *proto.mutable_origin() = transform::ToProto(compressed_range_data.origin);
  *proto.mutable_returns() = compressed_range_data.returns.ToProto();
  *proto.mutable_misses() = compressed_range_data.misses.ToProto();
  return proto;
}

CompressedRangeData FromProto(const proto::CompressedRangeData& proto) {
  return CompressedRangeData{
      transform::ToEigen(proto.origin()),
      CompressedPointCloud(proto.returns()),
      CompressedPointCloud(proto.misses()),
  };
}

CompressedRangeData Compress(const RangeData& range_data) {
  return CompressedRangeData{
      range_data.origin,
      CompressedPointCloud(range_data.returns),
      CompressedPointCloud(range_data.misses),
  };
}

RangeData Decompress(const CompressedRangeData& compressed_range_data) {
  return RangeData{compressed_range_data.origin,
                   compressed_range_data.returns.Decompress(),
                   compressed_range_data.misses.Decompress()};
}

}  // namespace sensor
}  // namespace cartographer
