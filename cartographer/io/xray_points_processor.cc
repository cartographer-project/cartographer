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
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/draw_trajectories.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {
namespace {

struct PixelData {
  size_t num_occupied_cells_in_column = 0;
  float mean_r = 0.;
  float mean_g = 0.;
  float mean_b = 0.;
};

using PixelDataMatrix =
    Eigen::Matrix<PixelData, Eigen::Dynamic, Eigen::Dynamic>;

float Mix(const float a, const float b, const float t) {
  return a * (1. - t) + t * b;
}

// Convert 'mat' into a pleasing-to-look-at image.
Image IntoImage(const PixelDataMatrix& mat) {
  Image image(mat.cols(), mat.rows());
  float max = std::numeric_limits<float>::min();
  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        continue;
      }
      max = std::max<float>(max, std::log(cell.num_occupied_cells_in_column));
    }
  }

  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        image.SetPixel(x, y, {{255, 255, 255}});
        continue;
      }

      // We use a logarithmic weighting for how saturated a pixel will be. The
      // basic idea here was that walls (full height) are fully saturated, but
      // details like chairs and tables are still well visible.
      const float saturation =
          std::log(cell.num_occupied_cells_in_column) / max;
      const FloatColor color = {{Mix(1.f, cell.mean_r, saturation),
                                 Mix(1.f, cell.mean_g, saturation),
                                 Mix(1.f, cell.mean_b, saturation)}};
      image.SetPixel(x, y, ToUint8Color(color));
    }
  }
  return image;
}

bool ContainedIn(const common::Time& time,
                 const std::vector<mapping::Timespan>& timespans) {
  for (const mapping::Timespan& timespan : timespans) {
    if (timespan.start <= time && time <= timespan.end) {
      return true;
    }
  }
  return false;
}

}  // namespace

XRayPointsProcessor::XRayPointsProcessor(
    const double voxel_size, const transform::Rigid3f& transform,
    const std::vector<mapping::Floor>& floors,
    const DrawTrajectories& draw_trajectories,
    const std::string& output_filename,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    FileWriterFactory file_writer_factory, PointsProcessor* const next)
    : draw_trajectories_(draw_trajectories),
      trajectories_(trajectories),
      file_writer_factory_(file_writer_factory),
      next_(next),
      floors_(floors),
      output_filename_(output_filename),
      transform_(transform) {
  for (size_t i = 0; i < (floors_.empty() ? 1 : floors.size()); ++i) {
    aggregations_.emplace_back(
        Aggregation{mapping_3d::HybridGridBase<bool>(voxel_size), {}});
  }
}

std::unique_ptr<XRayPointsProcessor> XRayPointsProcessor::FromDictionary(
    const std::vector<mapping::proto::Trajectory>& trajectories,
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  std::vector<mapping::Floor> floors;
  const bool separate_floor = dictionary->HasKey("separate_floors") &&
                              dictionary->GetBool("separate_floors");
  const auto draw_trajectories = (!dictionary->HasKey("draw_trajectories") ||
                                  dictionary->GetBool("draw_trajectories"))
                                     ? DrawTrajectories::kYes
                                     : DrawTrajectories::kNo;
  if (separate_floor) {
    CHECK_EQ(trajectories.size(), 1)
        << "Can only detect floors with a single trajectory.";
    floors = mapping::DetectFloors(trajectories.at(0));
  }

  return common::make_unique<XRayPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      transform::FromDictionary(dictionary->GetDictionary("transform").get())
          .cast<float>(),
      floors, draw_trajectories, dictionary->GetString("filename"),
      trajectories, file_writer_factory, next);
}

void XRayPointsProcessor::WriteVoxels(const Aggregation& aggregation,
                                      FileWriter* const file_writer) {
  if (bounding_box_.isEmpty()) {
    LOG(WARNING) << "Not writing output: bounding box is empty.";
    return;
  }

  // Returns the (x, y) pixel of the given 'index'.
  const auto voxel_index_to_pixel = [this](const Eigen::Array3i& index) {
    // We flip the y axis, since matrices rows are counted from the top.
    return Eigen::Array2i(bounding_box_.max()[1] - index[1],
                          bounding_box_.max()[2] - index[2]);
  };

  // Hybrid grid uses X: forward, Y: left, Z: up.
  // For the screen we are using. X: right, Y: up
  const int xsize = bounding_box_.sizes()[1] + 1;
  const int ysize = bounding_box_.sizes()[2] + 1;
  PixelDataMatrix pixel_data_matrix = PixelDataMatrix(ysize, xsize);
  for (mapping_3d::HybridGridBase<bool>::Iterator it(aggregation.voxels);
       !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const Eigen::Array2i pixel = voxel_index_to_pixel(cell_index);
    PixelData& pixel_data = pixel_data_matrix(pixel.y(), pixel.x());
    const auto& column_data = aggregation.column_data.at(
        std::make_pair(cell_index[1], cell_index[2]));
    pixel_data.mean_r = column_data.sum_r / column_data.count;
    pixel_data.mean_g = column_data.sum_g / column_data.count;
    pixel_data.mean_b = column_data.sum_b / column_data.count;
    ++pixel_data.num_occupied_cells_in_column;
  }

  Image image = IntoImage(pixel_data_matrix);
  if (draw_trajectories_ == DrawTrajectories::kYes) {
    for (size_t i = 0; i < trajectories_.size(); ++i) {
      DrawTrajectory(
          trajectories_[i], GetColor(i),
          [&voxel_index_to_pixel, &aggregation,
           this](const transform::Rigid3d& pose) -> Eigen::Array2i {
            return voxel_index_to_pixel(aggregation.voxels.GetCellIndex(
                (transform_ * pose.cast<float>()).translation()));
          },
          image.GetCairoSurface().get());
    }
  }

  image.WritePng(file_writer);
  CHECK(file_writer->Close());
}

void XRayPointsProcessor::Insert(const PointsBatch& batch,
                                 Aggregation* const aggregation) {
  constexpr FloatColor kDefaultColor = {{0.f, 0.f, 0.f}};
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f camera_point = transform_ * batch.points[i];
    const Eigen::Array3i cell_index =
        aggregation->voxels.GetCellIndex(camera_point);
    *aggregation->voxels.mutable_value(cell_index) = true;
    bounding_box_.extend(cell_index.matrix());
    ColumnData& column_data =
        aggregation->column_data[std::make_pair(cell_index[1], cell_index[2])];
    const auto& color =
        batch.colors.empty() ? kDefaultColor : batch.colors.at(i);
    column_data.sum_r += color[0];
    column_data.sum_g += color[1];
    column_data.sum_b += color[2];
    ++column_data.count;
  }
}

void XRayPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    Insert(*batch, &aggregations_[0]);
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      if (!ContainedIn(batch->start_time, floors_[i].timespans)) {
        continue;
      }
      Insert(*batch, &aggregations_[i]);
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult XRayPointsProcessor::Flush() {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    WriteVoxels(aggregations_[0],
                file_writer_factory_(output_filename_ + ".png").get());
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      WriteVoxels(
          aggregations_[i],
          file_writer_factory_(output_filename_ + std::to_string(i) + ".png")
              .get());
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
