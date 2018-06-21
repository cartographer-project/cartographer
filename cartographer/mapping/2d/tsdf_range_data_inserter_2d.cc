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

#include <cstdlib>
#include <fstream>
#include <random>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_casting.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

float Angle(const Eigen::Vector3f& v) {
  const Eigen::Vector3f v_normalized = v.normalized();
  return std::atan2(v_normalized[1], v_normalized[0]);
}

float Angle2D(const Eigen::Vector2f& v) {
  const Eigen::Vector2f v_normalized = v.normalized();
  return std::atan2(v_normalized[1], v_normalized[0]);
}

float ToTwoPi(float angle) {
  if (angle > M_PI)
    return ToTwoPi(angle - 2.0 * M_PI);
  else if (angle < -M_PI)
    return ToTwoPi(angle + 2.0 * M_PI);
  return angle;
}

float CircularDistance(const float angle_0, const float angle_1) {
  return (ToTwoPi(angle_0 - angle_1));
}

float CircularMean(const std::vector<float>& angles) {
  float summed_y = 0;
  float summed_x = 0;
  for (const float angle : angles) {
    summed_y += std::sin(angle);
    summed_x += std::cos(angle);
  }
  return std::atan2(summed_y, summed_x);
}

float GaussianKernel(const float x, const float mu = 0, const float sigma = 1) {
  return 1.0 / (std::sqrt(2.0 * M_PI) * std::pow(sigma, 2)) *
         std::exp((-0.5 * std::pow((x - mu) / std::pow(sigma, 2), 2)));
}
}  // namespace

RangeDataInserter2DTSDF::RangeDataInserter2DTSDF(
    const proto::TSDFRangeDataInserterOptions2D& options)
    : options_(options) {}

// Utility function for obtaining helper variables for the raytracing step.
inline void GetRaytracingHelperVariables(
    const Eigen::Vector2f& observation_origin,
    const Eigen::Vector2f& observation_ray, float t_start, float t_end,
    const MapLimits& map_limits, double grid_size_inv,
    Eigen::Vector2f* grid_index, Eigen::Vector2f* grid_step,
    Eigen::Vector2f* t_max, Eigen::Vector2f* t_delta) {
  CHECK(grid_index != nullptr);
  CHECK(grid_step != nullptr);
  CHECK(t_max != nullptr);
  CHECK(t_delta != nullptr);

  // Start and end of voxel traversal region.
  const Eigen::Vector2f traversal_start =
      observation_origin + t_start * observation_ray;
  const Eigen::Vector2f traversal_end =
      observation_origin + t_end * observation_ray;
  const Eigen::Vector2f traversal_start_scaled =
      traversal_start * grid_size_inv;
  const Eigen::Vector2f traversal_end_scaled = traversal_end * grid_size_inv;
  const Eigen::Vector2f traversal_ray_scaled =
      traversal_end_scaled - traversal_start_scaled;

  const Eigen::Vector2f traversal_ray_scaled_inv(
      1.f / traversal_ray_scaled.x(), 1.f / traversal_ray_scaled.y());

  // Calculate starting voxel index.
  *grid_index << std::floor(traversal_start_scaled.x()),
      std::floor(traversal_start_scaled.y());

  // Calculate the direction of the traversal step.
  *grid_step << (traversal_ray_scaled.x() < 0.0f ? -1 : 1),
      (traversal_ray_scaled.y() < 0.0f ? -1 : 1);

  const Eigen::Vector2f adjustment(grid_step->x() > 0 ? 1 : 0,
                                   grid_step->y() > 0 ? 1 : 0);

  *t_max = ((*grid_index + adjustment) - traversal_start_scaled)
               .cwiseProduct(traversal_ray_scaled_inv);
  // Determine how far we must travel along the ray before we have crossed a
  // grid cell.
  *t_delta = grid_step->cast<float>().cwiseProduct(traversal_ray_scaled_inv);
}

void RangeDataInserter2DTSDF::Insert(const sensor::RangeData& range_data_ref,
                                     GridInterface* grid) const {
  TSDF2D* tsdf = static_cast<TSDF2D*>(grid);
  sensor::RangeData range_data = range_data_ref;

  std::vector<float> normals;
  if (options_.normal_estimation_options().enable()) {
    ComputeNormals(range_data, &normals);
  }
  const float truncation_distance = options_.truncation_distance();
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Vector3f direction = (hit - range_data.origin).normalized();
    const Eigen::Vector3f end_position = hit + truncation_distance * direction;
    bounding_box.extend(end_position.head<2>());
  }
  constexpr float kPadding = 1e-6f;
  tsdf->GrowLimits(bounding_box.min() - kPadding * Eigen::Vector2f::Ones());
  tsdf->GrowLimits(bounding_box.max() + kPadding * Eigen::Vector2f::Ones());

  bool project_distance_to_normal =
      options_.project_sdf_distance_to_scan_normal();
  bool scale_weight_angle_ray_normal =
      options_.scale_update_weight_to_angle_scan_normal_to_ray();
  bool scale_weight_distance_to_hit =
      options_.scale_update_weight_to_distance_cell_to_hit();

  const Eigen::Vector2f origin = range_data.origin.head<2>();
  size_t i_hit = 0;
  for (const Eigen::Vector3f& hit_3d : range_data.returns) {
    const Eigen::Vector2f hit = hit_3d.head<2>();

    const double range = (hit - origin).norm();
    // Start and end of truncation parameter, normalized by range.
    const double range_inv = 1.0 / range;
    const double t_truncation_distance = truncation_distance * range_inv;
    const float t_start = 1.0f - t_truncation_distance;
    const float t_end = 1.0f + t_truncation_distance;

    // Calculate helper variables for raytracing loop.
    const Eigen::Vector2f ray = hit - origin;
    float resolution = tsdf->limits().resolution();
    float voxel_size_inv = 1. / resolution;
    Eigen::Vector2f voxel_index, voxel_step;
    Eigen::Vector2f t_max, t_delta;
    GetRaytracingHelperVariables(origin, ray, t_start, t_end, tsdf->limits(),
                                 voxel_size_inv, &voxel_index, &voxel_step,
                                 &t_max, &t_delta);

    float t = 0;
    const Eigen::Vector2f p_start = origin + t_start * ray;
    const Eigen::Vector2f p_delta = (t_end - t_start) * ray;

    float scale_weight_angle_ray_normal_scale = 1.f;
    if (scale_weight_angle_ray_normal) {
      float angle_ray_normal = CircularDistance(normals[i_hit], Angle2D(-ray));
      scale_weight_angle_ray_normal_scale = GaussianKernel(
          angle_ray_normal, 0.0,
          options_.update_weight_angle_scan_normal_to_ray_kernel_bandwith());
    }

    while (t < 1.0) {
      int min_coeff_idx;
      const float t_next = t_max.minCoeff(&min_coeff_idx);
      Eigen::Vector2f sampling_point = p_start + t * p_delta;
      Eigen::Array2i cell_index = tsdf->limits().GetCellIndex(sampling_point);
      Eigen::Vector2f cell_center = tsdf->limits().GetCellCenter(cell_index);

      // Distance in meters from sensor origin to the traversal point.
      float distance = (cell_center - origin).norm();
      float update_weight = 1.f;
      float update_sdf = range - distance;

      if (project_distance_to_normal) {
        float normal_orientation = normals[i_hit];
        update_sdf = (update_sdf * (origin - hit).normalized())
                         .dot(Eigen::Vector2f{std::cos(normal_orientation),
                                              std::sin(normal_orientation)});
      }
      if (scale_weight_angle_ray_normal) {
        update_weight *= scale_weight_angle_ray_normal_scale;
      }
      if (scale_weight_distance_to_hit) {
        update_weight *= GaussianKernel(
            range - distance, 0.0,
            options_.update_weight_distance_cell_to_hit_kernel_bandwith());
      }

      UpdateCell(tsdf, cell_index, update_sdf, range, update_weight);

      // Advance to next voxel.
      t = t_next;
      t_max(min_coeff_idx) += t_delta(min_coeff_idx);
    }
    i_hit++;
  }
  tsdf->FinishUpdate();
}

struct RangeDataSorter {
  RangeDataSorter(Eigen::Vector3f origin) { origin_ = origin; }
  bool operator()(Eigen::Vector3f p0, Eigen::Vector3f p1) {
    float angle_p0 = Angle(p0 - origin_);
    float angle_p1 = Angle(p1 - origin_);
    return (angle_p0 < angle_p1);
  }

 private:
  Eigen::Vector3f origin_;
};

void RangeDataInserter2DTSDF::UpdateCell(TSDF2D* const tsdf,
                                         const Eigen::Array2i& cell,
                                         float update_sdf, float ray_length,
                                         float update_weight_scale) const {
  float update_weight = 0.f;
  if (proto::CONSTANT_WEIGHT == options_.range_data_inserter_weight_type()) {
    update_weight = ComputeWeightConstant(update_sdf);
  } else if (proto::LINEAR_WEIGHT ==
             options_.range_data_inserter_weight_type()) {
    update_weight = ComputeWeightLinear(update_sdf, ray_length);
  } else if (proto::QUADRATIC_WEIGHT ==
             options_.range_data_inserter_weight_type()) {
    update_weight = ComputeWeightQuadratic(update_sdf, ray_length);
  }
  update_weight *= update_weight_scale;

  // todo(kdaun) use pair getter
  float updated_weight = tsdf->GetWeight(cell) + update_weight;
  float updated_sdf = updated_weight > 0.f
                          ? (tsdf->GetTSD(cell) * tsdf->GetWeight(cell) +
                             update_sdf * update_weight) /
                                updated_weight
                          : tsdf->GetTSD(cell);
  tsdf->SetCell(cell, updated_sdf, updated_weight);
}

float RangeDataInserter2DTSDF::ComputeWeightConstant(float sdf) const {
  float behind_surface_factor = 1.0;
  if (-options_.behind_surface_distance() > sdf) {
    behind_surface_factor =
        (sdf + options_.truncation_distance()) /
        (options_.truncation_distance() - options_.behind_surface_distance());
  }
  return behind_surface_factor * options_.update_weight();
}

float RangeDataInserter2DTSDF::ComputeWeightLinear(float sdf,
                                                   float ray_length) const {
  float weight = 0.f;
  if (-options_.behind_surface_distance() < sdf) {
    weight = options_.update_weight() / ray_length;
  } else if (-options_.truncation_distance() < sdf) {
    float behind_surface_factor =
        (sdf + options_.truncation_distance()) /
        (options_.truncation_distance() - options_.behind_surface_distance());
    weight = behind_surface_factor * options_.update_weight() / ray_length;
  }
  return weight;
}

float RangeDataInserter2DTSDF::ComputeWeightQuadratic(float sdf,
                                                      float ray_length) const {
  float weight = 0.f;
  if (-options_.behind_surface_distance() < sdf) {
    weight = options_.update_weight() / (std::pow(ray_length, 2));
  } else if (-options_.truncation_distance() < sdf) {
    float behind_surface_factor =
        (sdf + options_.truncation_distance()) /
        (options_.truncation_distance() - options_.behind_surface_distance());
    weight = behind_surface_factor * options_.update_weight() /
             (std::pow(ray_length, 2));
  }
  return weight;
}

}  // namespace mapping
}  // namespace cartographer