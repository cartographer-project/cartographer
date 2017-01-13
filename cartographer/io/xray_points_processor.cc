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
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {
namespace {

using Voxels = mapping_3d::HybridGridBase<bool>;

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
  CHECK_EQ(stride % 4, 0);
  std::vector<uint32_t> pixels(stride / 4 * mat.rows(), 0.);

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
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
  CHECK_EQ(cairo_surface_write_to_png(surface.get(), filename.c_str()),
           CAIRO_STATUS_SUCCESS);
}

void WriteVoxels(const string& filename, const Voxels& voxels) {
  Eigen::Array3i min(std::numeric_limits<int>::max(),
                     std::numeric_limits<int>::max(),
                     std::numeric_limits<int>::max());
  Eigen::Array3i max(std::numeric_limits<int>::min(),
                     std::numeric_limits<int>::min(),
                     std::numeric_limits<int>::min());

  // Find the maximum and minimum cells.
  for (Voxels::Iterator it(voxels); !it.Done(); it.Next()) {
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
  for (Voxels::Iterator it(voxels); !it.Done(); it.Next()) {
    const Eigen::Array2i pixel = voxel_index_to_pixel(it.GetCellIndex());
    ++image(pixel.y(), pixel.x());
  }
  TakeLogarithm(&image);
  WritePng(filename, image);
}

bool ContainedIn(
    const common::Time& time,
    const std::vector<common::Interval<common::Time>>& time_intervals) {
  for (const auto& interval : time_intervals) {
    if (interval.start <= time && time <= interval.end) {
      return true;
    }
  }
  return false;
}

void Insert(const PointsBatch& batch, const transform::Rigid3f& transform,
            Voxels* voxels) {
  for (const auto& point : batch.points) {
    const Eigen::Vector3f camera_point = transform * point;
    *voxels->mutable_value(voxels->GetCellIndex(camera_point)) = true;
  }
}

}  // namespace

XRayPointsProcessor::XRayPointsProcessor(
    const double voxel_size, const transform::Rigid3f& transform,
    const std::vector<mapping::Floor>& floors, const string& output_filename,
    PointsProcessor* next)
    : next_(next),
      floors_(floors),
      output_filename_(output_filename),
      transform_(transform) {
  for (size_t i = 0; i < (floors_.empty() ? 1 : floors.size()); ++i) {
    voxels_.emplace_back(voxel_size, Eigen::Vector3f::Zero());
  }
}

std::unique_ptr<XRayPointsProcessor> XRayPointsProcessor::FromDictionary(
    const mapping::proto::Trajectory& trajectory,
    common::LuaParameterDictionary* dictionary, PointsProcessor* next) {
  std::vector<mapping::Floor> floors;
  if (dictionary->HasKey("separate_floors") &&
      dictionary->GetBool("separate_floors")) {
    floors = mapping::DetectFloors(trajectory);
  }

  return common::make_unique<XRayPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      transform::FromDictionary(dictionary->GetDictionary("transform").get())
          .cast<float>(),
      floors, dictionary->GetString("filename"), next);
}

void XRayPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (floors_.empty()) {
    CHECK_EQ(voxels_.size(), 1);
    Insert(*batch, transform_, &voxels_[0]);
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      if (!ContainedIn(batch->time, floors_[i].timespans)) {
        continue;
      }
      Insert(*batch, transform_, &voxels_[i]);
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult XRayPointsProcessor::Flush() {
  if (floors_.empty()) {
    CHECK_EQ(voxels_.size(), 1);
    WriteVoxels(output_filename_ + ".png", voxels_[0]);
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      WriteVoxels(output_filename_ + std::to_string(i) + ".png", voxels_[i]);
    }
  }

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "X-Ray generation must be configured to occur after any "
                    "stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
