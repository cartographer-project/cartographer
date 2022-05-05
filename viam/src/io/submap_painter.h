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

#ifndef VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
#define VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/io/submap_painter.h"

namespace viam {
namespace io {

cartographer::io::PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId,
          cartographer::io::SubmapSlice>& submaps,
    double resolution);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
