/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;
// Minimum distance between range observation and origin.
constexpr float kMinRange = 1e-6f;

void GrowAsNeeded(const sensor::RangeData& range_data,
                  const float truncation_distance, TSDF2D* const tsdf) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Vector3f direction = (hit - range_data.origin).normalized();
    const Eigen::Vector3f end_position = hit + truncation_distance * direction;
    bounding_box.extend(end_position.head<2>());
  }
  constexpr float kPadding = 1e-6f;
  tsdf->GrowLimits(bounding_box.min() - kPadding * Eigen::Vector2f::Ones());
  tsdf->GrowLimits(bounding_box.max() + kPadding * Eigen::Vector2f::Ones());
}

float GaussianKernel(const float x, const float sigma) {
  return 1.0 / (std::sqrt(2.0 * M_PI) * sigma) *
         std::exp(-0.5 * std::pow(x / sigma, 2));
}

std::pair<Eigen::Array2i, Eigen::Array2i> SuperscaleRay(
    const Eigen::Vector2f& begin, const Eigen::Vector2f& end,
    TSDF2D* const tsdf) {
  const MapLimits& limits = tsdf->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));

  const Eigen::Array2i superscaled_begin =
      superscaled_limits.GetCellIndex(begin);
  const Eigen::Array2i superscaled_end = superscaled_limits.GetCellIndex(end);
  return std::make_pair(superscaled_begin, superscaled_end);
}

struct RangeDataSorter {
  RangeDataSorter(Eigen::Vector3f origin) { origin_ = origin; }
  bool operator()(Eigen::Vector3f lhs, Eigen::Vector3f rhs) {
    const Eigen::Vector2f delta_lhs = (lhs - origin_).head<2>();
    const Eigen::Vector2f delta_rhs = (lhs - origin_).head<2>();
    return common::atan2(delta_lhs) < common::atan2(delta_rhs);
  }

 private:
  Eigen::Vector3f origin_;
};

}  // namespace

proto::TSDFRangeDataInserterOptions2D CreateTSDFRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::TSDFRangeDataInserterOptions2D options;
  options.set_truncation_distance(
      parameter_dictionary->GetDouble("truncation_distance"));
  options.set_maximum_weight(parameter_dictionary->GetDouble("maximum_weight"));
  options.set_update_free_space(
      parameter_dictionary->GetBool("update_free_space"));
  *options
       .mutable_normal_estimation_options() = CreateNormalEstimationOptions2D(
      parameter_dictionary->GetDictionary("normal_estimation_options").get());
  options.set_project_sdf_distance_to_scan_normal(
      parameter_dictionary->GetBool("project_sdf_distance_to_scan_normal"));
  options.set_update_weight_range_exponent(
      parameter_dictionary->GetInt("update_weight_range_exponent"));
  options.set_update_weight_angle_scan_normal_to_ray_kernel_bandwith(
      parameter_dictionary->GetDouble(
          "update_weight_angle_scan_normal_to_ray_kernel_bandwith"));
  options.set_update_weight_distance_cell_to_hit_kernel_bandwith(
      parameter_dictionary->GetDouble(
          "update_weight_distance_cell_to_hit_kernel_bandwith"));
  return options;
}

TSDFRangeDataInserter2D::TSDFRangeDataInserter2D(
    const proto::TSDFRangeDataInserterOptions2D& options)
    : options_(options) {}

void TSDFRangeDataInserter2D::Insert(const sensor::RangeData& range_data,
                                     GridInterface* grid) const {
  // Extend bounding box to fit range data
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Vector2f direction =
        (hit.head<2>() - range_data.origin.head<2>()).normalized();
    const Eigen::Vector2f end_position =
        hit.head<2>() + truncation_distance * direction;
    bounding_box.extend(end_position);
  }
  bool scale_update_weight_angle_scan_normal_to_ray =
      options_.update_weight_angle_scan_normal_to_ray_kernel_bandwith() != 0.f;
  TSDF2D* tsdf = static_cast<TSDF2D*>(grid);
  GrowAsNeeded(range_data, static_cast<float>(options_.truncation_distance()),
               tsdf);

  // Compute normals if needed
  sensor::RangeData sorted_range_data = range_data;
  std::vector<float> normals;
  if (options_.project_sdf_distance_to_scan_normal() ||
      scale_update_weight_angle_scan_normal_to_ray) {
    std::sort(sorted_range_data.returns.begin(),
              sorted_range_data.returns.end(),
              RangeDataSorter(sorted_range_data.origin));
    normals = EstimateNormals(sorted_range_data,
                              options_.normal_estimation_options());
  }

  const Eigen::Vector2f origin = sorted_range_data.origin.head<2>();
  for (size_t hit_index = 0; hit_index < sorted_range_data.returns.size();
       ++hit_index) {
    // Compute pixelmask intersecting with ray
    const Eigen::Vector2f hit = sorted_range_data.returns[hit_index].head<2>();
    const Eigen::Vector2f ray = hit - origin;
    const float range = ray.norm();
    if (range < options_.truncation_distance()) continue;
    const float t_truncation_distance =
        static_cast<float>(options_.truncation_distance()) / range;
    const Eigen::Vector2f ray_begin =
        options_.update_free_space()
            ? origin
            : origin + (1.0f - t_truncation_distance) * ray;
    const Eigen::Vector2f ray_end =
        origin + (1.0f + t_truncation_distance) * ray;
    std::pair<Eigen::Array2i, Eigen::Array2i> superscaled_ray =
        SuperscaleRay(ray_begin, ray_end, tsdf);
    std::vector<Eigen::Array2i> ray_mask = RayToPixelMask(
        superscaled_ray.first, superscaled_ray.second, kSubpixelScale);

    // Precompute weight factors
    float weight_factor_angle_ray_normal = 1.f;
    if (scale_update_weight_angle_scan_normal_to_ray) {
      const Eigen::Vector2f negative_ray = -ray;
      float angle_ray_normal = common::NormalizeAngleDifference(
          normals[hit_index] - common::atan2(negative_ray));
      weight_factor_angle_ray_normal = GaussianKernel(
          angle_ray_normal,
          options_.update_weight_angle_scan_normal_to_ray_kernel_bandwith());
    }
    float weight_factor_range = 1.f;
    if (options_.update_weight_range_exponent() != 0) {
      weight_factor_range =
          ComputeWeight(range, options_.update_weight_range_exponent());
    }

    for (const Eigen::Array2i& cell_index : ray_mask) {
      Eigen::Vector2f cell_center = tsdf->limits().GetCellCenter(cell_index);
      float distance_cell_to_origin = (cell_center - origin).norm();
      float update_tsd = range - distance_cell_to_origin;
      if (options_.project_sdf_distance_to_scan_normal()) {
        float normal_orientation = normals[hit_index];
        update_tsd = (cell_center - hit)
                         .dot(Eigen::Vector2f{std::cos(normal_orientation),
                                              std::sin(normal_orientation)});
      }
      float update_weight =
          weight_factor_range * weight_factor_angle_ray_normal;
      if (options_.update_weight_distance_cell_to_hit_kernel_bandwith() !=
          0.f) {
        update_weight *= GaussianKernel(
            update_tsd,
            options_.update_weight_distance_cell_to_hit_kernel_bandwith());
      }
      UpdateCell(tsdf, cell_index, update_tsd, update_weight);
    }
  }
  tsdf->FinishUpdate();
}

void TSDFRangeDataInserter2D::UpdateCell(TSDF2D* const tsdf,
                                         const Eigen::Array2i& cell,
                                         float update_sdf,
                                         float update_weight) const {
  if (update_weight == 0.f) return;
  const std::pair<float, float> tsd_and_weight = tsdf->GetTSDAndWeight(cell);
  float updated_weight = tsd_and_weight.second + update_weight;
  float updated_sdf = updated_weight > 0.f
                          ? (tsd_and_weight.first * tsd_and_weight.second +
                             update_sdf * update_weight) /
                                updated_weight
                          : tsd_and_weight.first;
  tsdf->SetCell(cell, updated_sdf, updated_weight);
}

float TSDFRangeDataInserter2D::ComputeWeight(float ray_length,
                                             int exponent) const {
  float weight = 0.f;
  if (std::abs(ray_length) > kMinRange) {
    weight = 1.f / (std::pow(ray_length, exponent));
  }
  return weight;
}

}  // namespace mapping
}  // namespace cartographer
