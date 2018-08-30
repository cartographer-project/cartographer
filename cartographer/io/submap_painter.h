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

#ifndef CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
#define CARTOGRAPHER_IO_SUBMAP_PAINTER_H_

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

struct PaintSubmapSlicesResult {
  PaintSubmapSlicesResult(::cartographer::io::UniqueCairoSurfacePtr surface,
                          Eigen::Array2f origin)
      : surface(std::move(surface)), origin(origin) {}
  ::cartographer::io::UniqueCairoSurfacePtr surface;

  // Top left pixel of 'surface' in map frame.
  Eigen::Array2f origin;
};

struct SubmapSlice {
  SubmapSlice()
      : surface(::cartographer::io::MakeUniqueCairoSurfacePtr(nullptr)) {}

  // Texture data.
  int width;
  int height;
  int version;
  double resolution;
  ::cartographer::transform::Rigid3d slice_pose;
  ::cartographer::io::UniqueCairoSurfacePtr surface;
  // Pixel data used by 'surface'. Must outlive 'surface'.
  std::vector<uint32_t> cairo_data;

  // Metadata.
  ::cartographer::transform::Rigid3d pose;
  int metadata_version = -1;
};

struct SubmapTexture {
  struct Pixels {
    std::vector<char> intensity;
    std::vector<char> alpha;
  };
  Pixels pixels;
  int width;
  int height;
  double resolution;
  ::cartographer::transform::Rigid3d slice_pose;
};

struct SubmapTextures {
  int version;
  std::vector<SubmapTexture> textures;
};

PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId, SubmapSlice>& submaps,
    double resolution);

void FillSubmapSlice(
    const ::cartographer::transform::Rigid3d& global_submap_pose,
    const ::cartographer::mapping::proto::Submap& proto,
    SubmapSlice* const submap_slice,
    mapping::ValueConversionTables* conversion_tables);

void DeserializeAndFillSubmapSlices(
    ProtoStreamDeserializer* deserializer,
    std::map<::cartographer::mapping::SubmapId, SubmapSlice>* submap_slices,
    mapping::ValueConversionTables* conversion_tables);

// Unpacks cell data as provided by the backend into 'intensity' and 'alpha'.
SubmapTexture::Pixels UnpackTextureData(const std::string& compressed_cells,
                                        int width, int height);

// Draw a texture into a cairo surface. 'cairo_data' will store the pixel data
// for the surface and must therefore outlive the use of the surface.
UniqueCairoSurfacePtr DrawTexture(const std::vector<char>& intensity,
                                  const std::vector<char>& alpha, int width,
                                  int height,
                                  std::vector<uint32_t>* cairo_data);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
