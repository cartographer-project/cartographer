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

#ifdef WITH_OPEN3D
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
    options.set_normal_estimate_max_nn(
            parameter_dictionary->GetInt("normal_estimate_max_nn"));
    options.set_normal_estimate_radius(
            parameter_dictionary->GetDouble("normal_estimate_radius"));
    const std::string normal_computation_method_string =
        parameter_dictionary->GetString("normal_computation_method");
    proto::TSDFRangeDataInserterOptions3D::NormalComputationMethod
        normal_computation_method;
    CHECK(proto::TSDFRangeDataInserterOptions3D_NormalComputationMethod_Parse(
        normal_computation_method_string, &normal_computation_method))
        << "Unknown TSDFRangeDataInserterOptions3D::NormalComputationMethod: "
        << normal_computation_method_string;
    options.set_normal_computation_method(normal_computation_method);
    return options;
}

TSDFRangeDataInserter3D::TSDFRangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options) {}

void TSDFRangeDataInserter3D::InsertTriangle(const Eigen::Vector3f& v0,
                                             const Eigen::Vector3f& v1,
                                             const Eigen::Vector3f& v2,
                                             const Eigen::Vector3f& origin,
                                             HybridGridTSDF* tsdf) const {
  Eigen::Vector3f d_01 = v1 - v0;
  Eigen::Vector3f d_02 = v2 - v0;
  Eigen::Vector3f d_12 = v2 - v1;
  float max_01 = d_01.cwiseAbs().maxCoeff();
  float max_02 = d_02.cwiseAbs().maxCoeff();
  float max_12 = d_12.cwiseAbs().maxCoeff();

  Eigen::Vector3f triangle_normal = (v0 - v1).cross(v0 - v2).normalized();
  if (triangle_normal.dot(origin - v0) < 0) triangle_normal = -triangle_normal;
  float resolution = 0.05f;
  int max_idx;

  int relative_truncation_distance =
      std::round(options_.tsdf_range_data_inserter_options_3d()
                     .relative_truncation_distance());
  for (int i = -relative_truncation_distance; i <= relative_truncation_distance;
       ++i) {
    float tsd_offset = resolution * i;
    Eigen::Vector3f v_offset = tsd_offset * triangle_normal;

    if ((max_01 > max_02) && (max_01 > max_12)) {
      d_01.cwiseAbs().maxCoeff(&max_idx);
      RasterTriangle(v0 + v_offset, v2 + v_offset, v1 + v_offset,
                     triangle_normal, tsd_offset, tsdf);
    } else if (max_02 > max_12) {
      d_02.cwiseAbs().maxCoeff(&max_idx);
      RasterTriangle(v0 + v_offset, v1 + v_offset, v2 + v_offset,
                     triangle_normal, tsd_offset, tsdf);
    } else {
      d_12.cwiseAbs().maxCoeff(&max_idx);
      RasterTriangle(v1 + v_offset, v0 + v_offset, v2 + v_offset,
                     triangle_normal, tsd_offset, tsdf);
    }
  }
}

void TSDFRangeDataInserter3D::RasterTriangle(
    const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
    const Eigen::Vector3f& v2, const Eigen::Vector3f& triangle_normal,
    float tsd_offset, HybridGridTSDF* tsdf) const {
  const float truncation_distance =
      options_.tsdf_range_data_inserter_options_3d()
          .relative_truncation_distance() *
      tsdf->resolution();

  Eigen::Vector3f v01 = v1 - v0;
  Eigen::Vector3f v02 = v2 - v0;
  CHECK_LE(v01.cwiseAbs().maxCoeff(), v02.cwiseAbs().maxCoeff());

  const Eigen::Array3i i_0 = tsdf->GetCellIndex(v0);
  const Eigen::Array3i i_1 = tsdf->GetCellIndex(v1);
  const Eigen::Array3i i_2 = tsdf->GetCellIndex(v2);
  const Eigen::Array3i delta_i_01 = i_1 - i_0;
  const Eigen::Array3i delta_i_02 = i_2 - i_0;
  const Eigen::Array3i delta_i_12 = i_2 - i_1;
  int max_coeff_idx = 0;
  int num_samples_major = delta_i_02.cwiseAbs().maxCoeff(&max_coeff_idx);
  int num_samples_major_seg_01 = std::abs(delta_i_01[max_coeff_idx]);
  int num_samples_major_seg_12 = std::abs(delta_i_12[max_coeff_idx]);
  if (num_samples_major < 1) num_samples_major = 1;
  if (num_samples_major_seg_01 < 1) num_samples_major_seg_01 = 1;
  if (num_samples_major_seg_12 < 1) num_samples_major_seg_12 = 1;
  CHECK_LT(num_samples_major, 1 << 15);
  CHECK_LT(num_samples_major_seg_01, 1 << 15);
  CHECK_LT(num_samples_major_seg_12, 1 << 15);
  //  CHECK_EQ(num_samples_major_seg_01 + num_samples_major_seg_12,
  //  num_samples_major);

  for (int index_major = 0; index_major <= num_samples_major; ++index_major) {
    const Eigen::Array3i update_cell_index_row_left =
        i_0 + (delta_i_02.cast<float>() * float(index_major) /
               float(num_samples_major))
                  .round()
                  .cast<int>();
    Eigen::Array3i update_cell_index_row_right;
    if (index_major <= num_samples_major_seg_01) {
      update_cell_index_row_right =
          i_0 + (delta_i_01.cast<float>() * float(index_major) /
                 float(num_samples_major_seg_01))
                    .round()
                    .cast<int>();
    } else {
      update_cell_index_row_right =
          i_1 + (delta_i_12.cast<float>() *
                 float(index_major - num_samples_major_seg_01) /
                 float(num_samples_major_seg_12))
                    .round()
                    .cast<int>();
    }
    const Eigen::Array3i delta_col =
        update_cell_index_row_right - update_cell_index_row_left;
    int num_samples_col = delta_col.cwiseAbs().maxCoeff();
    if (num_samples_col == 0) num_samples_col = 1;
    CHECK_LT(num_samples_col, 1 << 15);
    for (int position_index_col = 0; position_index_col <= num_samples_col;
         ++position_index_col) {
      const Eigen::Array3i update_cell_index =
          update_cell_index_row_left +
          (delta_col.cast<float>() * float(position_index_col) /
           float(num_samples_col))
              .round()
              .cast<int>();
      Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
      float update_tsd = tsd_offset + (cell_center - v0).dot(triangle_normal);
      update_tsd =
          common::Clamp(update_tsd, -truncation_distance, truncation_distance);
      float update_weight = 1.0;
      UpdateCell(update_cell_index, update_tsd, update_weight, tsdf);
    }
  }
}

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
  if (normal.dot(ray) > 0.f) normal_direction = -1.f;
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

    for (int position = 0; position <= num_samples; ++position) {
//        LOG(INFO)<<"ratio "<< position / num_samples;
      const Eigen::Array3i update_cell_index =
          begin_cell + (delta.cast<float>() * float(position) / float(num_samples)).round().cast<int>();
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

    for (int position = 0; position <= num_samples; ++position) {
      const Eigen::Array3i update_cell_index =
          begin_cell + (delta.cast<float>() * float(position) / float(num_samples)).round().cast<int>();
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

  //  LOG(INFO) << "start normal computation";
  if (options_.tsdf_range_data_inserter_options_3d()
          .project_sdf_distance_to_scan_normal()) {
    switch (options_.tsdf_range_data_inserter_options_3d()
                .normal_computation_method()) {
      case proto::TSDFRangeDataInserterOptions3D::PCL: {
#ifdef USE_PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : range_data.returns) {
          cloud->push_back(
              {point.position[0], point.position[1], point.position[2]});
        }

        //    ... read, pass in or create a point cloud ...

        // Create the normal estimation class, and pass the input dataset to
        it pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal
        // estimation object. Its content will be filled inside the object,
        based on
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

        //     cloud_normals->points.size () should have the same size as the
        inpu
                //     cloud->points.size ()*
                LOG(INFO)
            << "finish normal computation";

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
          const Eigen::Vector3f normal = {
              cloud_normals->at(point_idx).normal_x,
              cloud_normals->at(point_idx).normal_y,
              cloud_normals->at(point_idx).normal_z};
          InsertHitWithNormal(hit, origin, normal, tsdf);
          ++point_idx;
        }
#else
        LOG(ERROR) << "Built without PCL - PCL-based normal computation not "
                      "available.";
#endif
        break;
    }
    case proto::TSDFRangeDataInserterOptions3D::OPEN3D: {
#ifdef WITH_OPEN3D
      std::shared_ptr<open3d::geometry::PointCloud> cloud =
          std::make_shared<open3d::geometry::PointCloud>();
      for (const auto& point : range_data.returns) {
        cloud->points_.push_back(
            {point.position[0], point.position[1], point.position[2]});
      }
      cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
          options_.tsdf_range_data_inserter_options_3d()
              .normal_estimate_radius(),
          options_.tsdf_range_data_inserter_options_3d()
              .normal_estimate_max_nn()));
      //        cloud->OrientNormalsTowardsCameraLocation(range_data.origin.cast<double>());
      //    open3d::visualization::DrawGeometries({cloud});

      int point_idx = 0;
      for (const auto& hit_point : range_data.returns) {
        const Eigen::Vector3f hit = hit_point.position.head<3>();
        const Eigen::Vector3f normal =
            cloud->normals_[point_idx].head<3>().cast<float>();
        InsertHitWithNormal(hit, origin, normal, tsdf);
        ++point_idx;
      }
#else
      LOG(ERROR) << "Built without Open3D - Open3D-based normal computation "
                    "not available.";
#endif
      break;
    }
    case proto::TSDFRangeDataInserterOptions3D::CLOUD_STRUCTURE: {
      if (range_data.returns.size() == 28800) {
        int NUM_ROWS = 16;
        int NUM_POINTS_PER_LINE = 1800;
        int NUM_POINTS = NUM_POINTS_PER_LINE * NUM_ROWS;
        for (int point_idx = 0; point_idx < NUM_POINTS; ++point_idx) {
          if (range_data.returns[point_idx].position.isZero()) continue;
          int i0 = point_idx;
          int horizontal_stride = 5;
          int i1 = point_idx + horizontal_stride >= NUM_POINTS
                       ? point_idx - horizontal_stride
                       : point_idx + horizontal_stride;
          int i2 = point_idx + NUM_POINTS_PER_LINE >= NUM_POINTS
                       ? point_idx - NUM_POINTS_PER_LINE
                       : point_idx + NUM_POINTS_PER_LINE;
          Eigen::Vector3f p0 = range_data.returns[i0].position;
          Eigen::Vector3f p1 = range_data.returns[i1].position;
          Eigen::Vector3f p2 = range_data.returns[i2].position;
          if (p1.isZero() || p2.isZero()) continue;
          float r0 = p0.norm();
          float r1 = p1.norm();
          float r2 = p2.norm();
          float max_range_delta = 1.f;
          if (std::abs(r0 - r1) > max_range_delta ||
              std::abs(r0 - r2) > max_range_delta ||
              std::abs(r1 - r2) > max_range_delta) {
            continue;
          }
          const Eigen::Vector3f normal = (p0 - p1).cross(p0 - p2).normalized();
          InsertHitWithNormal(p0, origin, normal, tsdf);
          ++point_idx;
        }  // namespace mapping
      }    // namespace cartographer
      break;
    }
    case proto::TSDFRangeDataInserterOptions3D::TRIANGLE_FILL_IN: {
      if (range_data.returns.size() == 28800) {
        int NUM_ROWS = 16;
        int NUM_POINTS_PER_LINE = 1800;
        int NUM_POINTS = NUM_POINTS_PER_LINE * NUM_ROWS;
        for (int point_idx = 0; point_idx < NUM_POINTS; ++point_idx) {
          if (range_data.returns[point_idx].position.isZero()) continue;
          int i0 = point_idx;
          int horizontal_stride = 5;
          int i1 = point_idx + horizontal_stride >= NUM_POINTS
                       ? point_idx - horizontal_stride
                       : point_idx + horizontal_stride;
          int i2 = point_idx + NUM_POINTS_PER_LINE >= NUM_POINTS
                       ? point_idx - NUM_POINTS_PER_LINE
                       : point_idx + NUM_POINTS_PER_LINE;
          Eigen::Vector3f p0 = range_data.returns[i0].position;
          Eigen::Vector3f p1 = range_data.returns[i1].position;
          Eigen::Vector3f p2 = range_data.returns[i2].position;
          if (p1.isZero() || p2.isZero()) continue;
          float r0 = p0.norm();
          float r1 = p1.norm();
          float r2 = p2.norm();
          float max_range_delta = 1.f;
          if (std::abs(r0 - r1) > max_range_delta ||
              std::abs(r0 - r2) > max_range_delta ||
              std::abs(r1 - r2) > max_range_delta) {
            continue;
          }
          const Eigen::Vector3f normal = (p0 - p1).cross(p0 - p2).normalized();
          //            InsertHitWithNormal(p0, origin, normal, tsdf);
          InsertTriangle(p0, p1, p2, origin, tsdf);
          ++point_idx;
        }  // namespace mapping
      }    // namespace cartographer
      break;
    }
    default: {
      LOG(ERROR) << "Invalid TSDFRangeDataInserterOptions3D normal "
                    "computation method.";
      break;
    }
    }
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
