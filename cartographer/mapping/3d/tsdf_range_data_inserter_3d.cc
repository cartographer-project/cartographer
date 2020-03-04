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

#include "cairo/cairo.h"

#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

//#define USE_PCL
#ifdef USE_PCL
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#endif

//#define USE_OPEN3D
#ifdef USE_OPEN3D
#include "Open3D/Open3D.h"
#endif

namespace cartographer {
namespace mapping {
namespace {
}  // namespace

proto::TSDFRangeDataInserterOptions3D CreateTSDFRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::TSDFRangeDataInserterOptions3D options;
  options.set_relative_truncation_distance(
      parameter_dictionary->GetDouble("relative_truncation_distance"));
  options.set_maximum_weight(parameter_dictionary->GetDouble("maximum_weight"));
  options.set_project_sdf_distance_to_scan_normal(
      parameter_dictionary->GetBool("project_sdf_distance_to_scan_normal"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  options.set_weight_function_epsilon(
      parameter_dictionary->GetDouble("weight_function_epsilon"));
  options.set_weight_function_sigma(
      parameter_dictionary->GetDouble("weight_function_sigma"));
  return options;
}

TSDFRangeDataInserter3D::TSDFRangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options) {}

void TSDFRangeDataInserter3D::InsertHitWithNormal(const Eigen::Vector3f& hit,
                                                  const Eigen::Vector3f& origin,
                                                  const Eigen::Vector3f& normal,
                                                  HybridGridTSDF* tsdf) const {
  const Eigen::Vector3f ray = hit - origin;
  const float range = ray.norm();
  const float truncation_distance =
      options_.tsdf_range_data_inserter_options_3d()
          .relative_truncation_distance() *
      tsdf->resolution();
  if (range < truncation_distance) return;
  bool update_free_space =
      options_.tsdf_range_data_inserter_options_3d().num_free_space_voxels() >
      0;  // todo(kdaun) use num free space
  // cells value instead of bool
  float normal_direction = 1.f;
  if (normal.dot(ray) < 0.f) normal_direction = -1.f;
  const Eigen::Vector3f ray_begin =
      hit - normal_direction * truncation_distance * normal;
  const Eigen::Vector3f ray_end =
      hit + normal_direction * truncation_distance * normal;

  bool use_default_raycast = true;

  if (use_default_raycast) {
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
      float update_tsd = normal_direction * (cell_center - hit).dot(normal);
      update_tsd =
          common::Clamp(update_tsd, -truncation_distance, truncation_distance);
      float update_weight = 1.0;
      UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);
    }
  } else {
    // Based on Amanatides, John, and Andrew Woo. "A fast voxel traversal
    // algorithm for ray tracing." Eurographics. Vol. 87. No. 3. 1987.
    Eigen::Array3i update_cell = tsdf->GetCellIndex(ray_begin);
    const Eigen::Array3i end_cell = tsdf->GetCellIndex(ray_end);
    const Eigen::Vector3f ray_begin_to_end = ray_end - ray_begin;
    const Eigen::Vector3f begin_cell_center =
        tsdf->GetCenterOfCell(update_cell);
    int step_x = update_cell[0] <= end_cell[0] ? 1 : -1;
    int step_y = update_cell[1] <= end_cell[1] ? 1 : -1;
    int step_z = update_cell[2] <= end_cell[2] ? 1 : -1;

    const Eigen::Vector3f step_direction({step_x, step_y, step_z});
    Eigen::Vector3f t_max = begin_cell_center - ray_begin +
                            0.5f * float(tsdf->resolution()) *
                                step_direction.cwiseQuotient(ray_begin_to_end);
    const Eigen::Vector3f t_delta =
        float(tsdf->resolution()) *
        (ray_begin_to_end.cwiseInverse()).cwiseAbs();
    while (t_max[0] < 1.0 || t_max[1] < 1.0 || t_max[2] < 1.0) {
      const Eigen::Array3i update_cell_index(
          {update_cell[0], update_cell[1], update_cell[2]});
      Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
      float distance_cell_to_origin = (cell_center - origin).norm();
      float update_tsd = range - distance_cell_to_origin;
      update_tsd =
          common::Clamp(update_tsd, -truncation_distance, truncation_distance);
      float update_weight = 1.0;
      UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);

      if (t_max[0] < t_max[1]) {
        if (t_max[0] < t_max[2]) {
          update_cell[0] = update_cell[0] + step_x;
          t_max[0] = t_max[0] + t_delta[0];
        } else {
          update_cell[2] = update_cell[2] + step_z;
          t_max[2] = t_max[2] + t_delta[2];
        }
      } else {
        if (t_max[1] < t_max[2]) {
          update_cell[1] = update_cell[1] + step_y;
          t_max[1] = t_max[1] + t_delta[1];
        } else {
          update_cell[2] = update_cell[2] + step_z;
          t_max[2] = t_max[2] + t_delta[2];
        }
      }
    }
  }
}

void TSDFRangeDataInserter3D::InsertHit(const Eigen::Vector3f& hit,
                                        const Eigen::Vector3f& origin,
                                        HybridGridTSDF* tsdf) const {
  const Eigen::Vector3f ray = hit - origin;
  const float range = ray.norm();
  const float truncation_distance =
      options_.tsdf_range_data_inserter_options_3d()
          .relative_truncation_distance() *
      tsdf->resolution();
  if (range < truncation_distance) return;
  const float truncation_ratio = truncation_distance / range;
  bool update_free_space =
      options_.tsdf_range_data_inserter_options_3d().num_free_space_voxels() >
      0;  // todo(kdaun) use num free space
          // cells value instead of bool
  const Eigen::Vector3f ray_begin =
      update_free_space ? origin : origin + (1.0f - truncation_ratio) * ray;
  const Eigen::Vector3f ray_end = origin + (1.0f + truncation_ratio) * ray;

  bool use_default_raycast = true;

  if (use_default_raycast) {
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
      float epsilon = options_.tsdf_range_data_inserter_options_3d()
                          .weight_function_epsilon();
      float normalized_update_tsd = update_tsd / truncation_distance;
      // Exponential weight drop-off behind surface
      if (normalized_update_tsd < -epsilon) {
        float sigma = options_.tsdf_range_data_inserter_options_3d()
                          .weight_function_sigma();
        update_weight = float(
            std::exp(-sigma * std::pow(-normalized_update_tsd - epsilon, 2)));
      }
      UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);
    }
  } else {
    // Based on Amanatides, John, and Andrew Woo. "A fast voxel traversal
    // algorithm for ray tracing." Eurographics. Vol. 87. No. 3. 1987.
    Eigen::Array3i update_cell = tsdf->GetCellIndex(ray_begin);
    const Eigen::Array3i end_cell = tsdf->GetCellIndex(ray_end);
    const Eigen::Vector3f ray_begin_to_end = ray_end - ray_begin;
    const Eigen::Vector3f begin_cell_center =
        tsdf->GetCenterOfCell(update_cell);
    int step_x = update_cell[0] <= end_cell[0] ? 1 : -1;
    int step_y = update_cell[1] <= end_cell[1] ? 1 : -1;
    int step_z = update_cell[2] <= end_cell[2] ? 1 : -1;

    const Eigen::Vector3f step_direction({step_x, step_y, step_z});
    Eigen::Vector3f t_max = begin_cell_center - ray_begin +
                            0.5f * float(tsdf->resolution()) *
                                step_direction.cwiseQuotient(ray_begin_to_end);
    const Eigen::Vector3f t_delta =
        float(tsdf->resolution()) *
        (ray_begin_to_end.cwiseInverse()).cwiseAbs();
    while (t_max[0] < 1.0 || t_max[1] < 1.0 || t_max[2] < 1.0) {
      const Eigen::Array3i update_cell_index(
          {update_cell[0], update_cell[1], update_cell[2]});
      Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
      float distance_cell_to_origin = (cell_center - origin).norm();
      float update_tsd = range - distance_cell_to_origin;
      update_tsd =
          common::Clamp(update_tsd, -truncation_distance, truncation_distance);
      float update_weight = 1.0;
      UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);

      if (t_max[0] < t_max[1]) {
        if (t_max[0] < t_max[2]) {
          update_cell[0] = update_cell[0] + step_x;
          t_max[0] = t_max[0] + t_delta[0];
        } else {
          update_cell[2] = update_cell[2] + step_z;
          t_max[2] = t_max[2] + t_delta[2];
        }
      } else {
        if (t_max[1] < t_max[2]) {
          update_cell[1] = update_cell[1] + step_y;
          t_max[1] = t_max[1] + t_delta[1];
        } else {
          update_cell[2] = update_cell[2] + step_z;
          t_max[2] = t_max[2] + t_delta[2];
        }
      }
    }
  }
}

void TSDFRangeDataInserter3D::Insert(const sensor::RangeData& range_data,
                                              GridInterface* grid) const {
  CHECK(grid != nullptr);
  CHECK(grid->GetGridType() == GridType::TSDF);
  HybridGridTSDF* tsdf = static_cast<HybridGridTSDF*>(grid);

  const Eigen::Vector3f origin = range_data.origin.head<3>();
  bool compute_normals = false;

  //  LOG(INFO) << "start normal computation";
  if (options_.tsdf_range_data_inserter_options_3d()
          .project_sdf_distance_to_scan_normal()) {
#ifdef USE_PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : range_data.returns) {
      cloud->push_back(
          {point.position[0], point.position[1], point.position[2]});
    }

    //    ... read, pass in or create a point cloud ...

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal
    // estimation object. Its content will be filled inside the object, based on
    // the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setKSearch(10);

    // Compute the features
    ne.compute(*cloud_normals);

    //     cloud_normals->points.size () should have the same size as the inpu
    //     cloud->points.size ()*
    LOG(INFO) << "finish normal computation";

    int point_idx = 0;
    //        for (const auto& hit_point : range_data.returns) {
    //          cloud_normals->at(point_idx).x = cloud->at(point_idx).x;
    //          cloud_normals->at(point_idx).y = cloud->at(point_idx).y;
    //          cloud_normals->at(point_idx).z = cloud->at(point_idx).z;
    //          ++point_idx;
    //        }
    //
    //        static int cloud_idx = 0;
    //        pcl::io::savePCDFileASCII(
    //            "test_pcd_normals" + std::to_string(cloud_idx) + ".pcd",
    //            *cloud_normals);
    //        ++cloud_idx;

    point_idx = 0;
    for (const auto& hit_point : range_data.returns) {
      const Eigen::Vector3f hit = hit_point.position.head<3>();
      const Eigen::Vector3f normal = {cloud_normals->at(point_idx).normal_x,
                                      cloud_normals->at(point_idx).normal_y,
                                      cloud_normals->at(point_idx).normal_z};
      InsertHitWithNormal(hit, origin, normal, tsdf);
      ++point_idx;
    }
#endif
#ifdef USE_OPEN3D
    std::shared_ptr<open3d::geometry::PointCloud> cloud =
        std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& point : range_data.returns) {
      cloud->points_.push_back(
          {point.position[0], point.position[1], point.position[2]});
    }
    cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.5, 30));
    //    cloud->OrientNormalsTowardsCameraLocation(range_data.origin.cast<double>());
    //    open3d::visualization::DrawGeometries({cloud});

    int point_idx = 0;
    for (const auto& hit_point : range_data.returns) {
      const Eigen::Vector3f hit = hit_point.position.head<3>();
      const Eigen::Vector3f normal =
          cloud->normals_[point_idx].head<3>().cast<float>();
      InsertHitWithNormal(hit, origin, normal, tsdf);
      ++point_idx;
    }
#endif
  } else {
    for (const sensor::RangefinderPoint& hit_point : range_data.returns) {
      const Eigen::Vector3f hit = hit_point.position.head<3>();
      InsertHit(hit, origin, tsdf);
    }
  }
  tsdf->FinishUpdate();
}

void TSDFRangeDataInserter3D::UpdateCell(const Eigen::Array3i& cell,
                                         float update_sdf, float update_weight,
                                         HybridGridTSDF* tsdf) const {
  if (update_weight == 0.f) return;

  const float old_weight = tsdf->GetWeight(cell);
  const float old_sdf = tsdf->GetTSD(cell);
  float updated_weight = old_weight + update_weight;
  float updated_sdf =
      (old_sdf * old_weight + update_sdf * update_weight) / updated_weight;
  float maximum_weight = static_cast<float>(
      options_.tsdf_range_data_inserter_options_3d().maximum_weight());
  updated_weight = std::min(updated_weight, maximum_weight);
  tsdf->SetCell(cell, updated_sdf, updated_weight);
  if (old_weight == tsdf->GetWeight(cell) &&
      old_weight < 0.999 * maximum_weight) {
    LOG(WARNING) << "Weight underflow " << update_weight;
  }
}

}  // namespace mapping
}  // namespace cartographer
