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

#include "cartographer/io/xray_points_processor.h"

#include <cmath>
#include <string>

#include "Eigen/Core"
#include "cairo/cairo.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/cairo_types.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {
namespace {

// Takes the logarithm of each value in 'mat', clamping to 0 as smallest value.
void TakeLogarithm(Eigen::MatrixXf* mat) {
  for (int y = 0; y < mat->rows(); ++y) {
    for (int x = 0; x < mat->cols(); ++x) {
      const float value = (*mat)(y, x);
      if (value == 0.f) {
        continue;
      }
      const float new_value = std::log(value);
      (*mat)(y, x) = new_value;
    }
  }
}

// Write 'mat' as a pleasing-to-look-at PNG into 'filename'
void WritePng(const string& filename, const Eigen::MatrixXf& mat) {
  const int stride =
      cairo_format_stride_for_width(CAIRO_FORMAT_ARGB32, mat.cols());
  std::vector<uint32_t> pixels(stride * mat.rows(), 0.);

  const float max = mat.maxCoeff();
  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const float value = mat(y, x);
      uint8_t shade = common::RoundToInt(255.f * (1.f - value / max));
      pixels[y * stride / 4 + x] =
          (255 << 24) | (shade << 16) | (shade << 8) | shade;
    }
  }

  // TODO(hrapp): cairo_image_surface_create_for_data does not take ownership of
  // the data until the surface is finalized. Once it is finalized though,
  // cairo_surface_write_to_png fails, complaining that the surface is already
  // finalized. This makes it pretty hard to pass back ownership of the image to
  // the caller.
  cairo::UniqueSurfacePtr surface(
      cairo_image_surface_create_for_data(
          reinterpret_cast<unsigned char*>(pixels.data()), CAIRO_FORMAT_ARGB32,
          mat.cols(), mat.rows(), stride),
      cairo_surface_destroy);
  cairo_surface_write_to_png(surface.get(), filename.c_str());
}

}  // namespace

XRayPointsProcessor::XRayPointsProcessor(const double voxel_size,
                                         const transform::Rigid3f& transform,
                                         const string& output_filename,
                                         PointsProcessor* next)
    : next_(next),
      output_filename_(output_filename),
      transform_(transform),
      voxels_(voxel_size, Eigen::Vector3f::Zero()) {}

std::unique_ptr<XRayPointsProcessor> XRayPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* dictionary, PointsProcessor* next) {
  return common::make_unique<XRayPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      transform::FromDictionary(dictionary->GetDictionary("transform").get())
          .cast<float>(),
      dictionary->GetString("filename"), next);
}

void XRayPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  for (const auto& point : batch->points) {
    const Eigen::Vector3f camera_point = transform_ * point;
    *voxels_.mutable_value(voxels_.GetCellIndex(camera_point)) = true;
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult XRayPointsProcessor::Flush() {
  WriteImage();
  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "X-Ray generation must be configured to occur after any "
                    "stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

void XRayPointsProcessor::WriteImage() {
  Eigen::Array3i min(std::numeric_limits<int>::max(),
                     std::numeric_limits<int>::max(),
                     std::numeric_limits<int>::max());
  Eigen::Array3i max(std::numeric_limits<int>::min(),
                     std::numeric_limits<int>::min(),
                     std::numeric_limits<int>::min());

  // Find the maximum and minimum cells.
  for (Voxels::Iterator it(voxels_); !it.Done(); it.Next()) {
    const Eigen::Array3i idx = it.GetCellIndex();
    min = min.min(idx);
    max = max.max(idx);
  }

  // Returns the (x, y) pixel of the given 'index'.
  const auto voxel_index_to_pixel = [&max, &min](const Eigen::Array3i& index) {
    // We flip the y axis, since matrices rows are counted from the top.
    return Eigen::Array2i(max[1] - index[1], max[2] - index[2]);
  };

  // Hybrid grid uses X: forward, Y: left, Z: up.
  // For the screen we are using. X: right, Y: up
  const int xsize = max[1] - min[1] + 1;
  const int ysize = max[2] - min[2] + 1;
  Eigen::MatrixXf image = Eigen::MatrixXf::Zero(ysize, xsize);
  for (Voxels::Iterator it(voxels_); !it.Done(); it.Next()) {
    const Eigen::Array2i pixel = voxel_index_to_pixel(it.GetCellIndex());
    ++image(pixel.y(), pixel.x());
  }
  TakeLogarithm(&image);
  WritePng(output_filename_, image);
}

}  // namespace io
}  // namespace cartographer
