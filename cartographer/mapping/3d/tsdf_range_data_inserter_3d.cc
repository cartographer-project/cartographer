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

#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {
}  // namespace


TSDFRangeDataInserter3D::TSDFRangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options) {}

void TSDFRangeDataInserter3D::InsertHit(const Eigen::Vector3f& hit,
                                        const Eigen::Vector3f& origin,
                                        HybridGridTSDF* tsdf) const {
  LOG(INFO) << "start InsertHit";
  const Eigen::Vector3f ray = hit - origin;
  const float range = ray.norm();
  const float truncation_distance = 0.3f;
  //      static_cast<float>(options_.truncation_distance());
  //  LOG(INFO)<<"tau "<<truncation_distance;
  if (range < truncation_distance) return;
  const float truncation_ratio = truncation_distance / range;
  bool update_free_space = false;
  const Eigen::Vector3f ray_begin =
      update_free_space ? origin : origin + (1.0f - truncation_ratio) * ray;
  const Eigen::Vector3f ray_end = origin + (1.0f + truncation_ratio) * ray;

  const Eigen::Array3i begin_cell = tsdf->GetCellIndex(ray_begin);
  const Eigen::Array3i end_cell = tsdf->GetCellIndex(ray_end);
  const Eigen::Array3i delta = end_cell - begin_cell;
  const int num_samples = delta.cwiseAbs().maxCoeff();
  CHECK_LT(num_samples, 1 << 15);
  // 'num_samples' is the number of samples we equi-distantly place on the
  // line between 'origin' and 'hit'. (including a fractional part for sub-
  // voxels) It is chosen so that between two samples we change from one voxel
  // to the next on the fastest changing dimension.

  for (int position = 0; position < num_samples; ++position) {
    const Eigen::Array3i update_cell_index =
        begin_cell + delta * position / num_samples;
    //    if (tsdf->CellIsUpdated(update_cell)) continue;
    Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
    float distance_cell_to_origin = (cell_center - origin).norm();
    float update_tsd = range - distance_cell_to_origin;
    update_tsd =
        common::Clamp(update_tsd, -truncation_distance, truncation_distance);
    float update_weight = 1.0;

    LOG(INFO) << "start UpdateCell";
    UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);
    LOG(INFO) << "finish UpdateCell";
  }
  LOG(INFO) << "finish InsertHit";
}

void TSDFRangeDataInserter3D::Insert(const sensor::RangeData& range_data,
                                              GridInterface* grid) const {
  LOG(INFO) << "start insert";
  CHECK(grid != nullptr);
  //  const float truncation_distance =
  //      static_cast<float>(options_.truncation_distance());
  HybridGridTSDF* tsdf = static_cast<HybridGridTSDF*>(grid);

  const Eigen::Vector3f origin = range_data.origin.head<3>();
  for (const sensor::RangefinderPoint& hit_point : range_data.returns) {
    const Eigen::Vector3f hit = hit_point.position.head<3>();
    InsertHit(hit, origin, tsdf);
  }
  LOG(INFO) << "finish insert";

  tsdf->FinishUpdate();
  LOG(INFO) << "start update";
}

void TSDFRangeDataInserter3D::UpdateCell(const Eigen::Array3i& cell,
                                         float update_sdf, float update_weight,
                                         HybridGridTSDF* tsdf) const {
  if (update_weight == 0.f) return;

  LOG(INFO) << "start GetWeight";
  const float old_weight = tsdf->GetWeight(cell);
  LOG(INFO) << "finish GetWeight";
  LOG(INFO) << "start GetTSD";
  const float old_sdf = tsdf->GetTSD(cell);
  LOG(INFO) << "start GetTSD";
  float updated_weight = old_weight + update_weight;
  float updated_sdf =
      (old_sdf * old_weight + update_sdf * update_weight) / updated_weight;
  float maximum_weight = 1000;  // static_cast<float>(options_.maximum_weight())
  updated_weight = std::min(updated_weight, maximum_weight);
  LOG(INFO) << "start SetCell";
  tsdf->SetCell(cell, updated_sdf, updated_weight);
  LOG(INFO) << "finish SetCell";
}

}  // namespace mapping
}  // namespace cartographer
