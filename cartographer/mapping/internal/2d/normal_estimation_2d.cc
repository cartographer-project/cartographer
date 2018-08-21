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

#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"

namespace cartographer {
namespace mapping {
namespace {

float NormalTo2DAngle(const Eigen::Vector3f& v) {
  return std::atan2(v[1], v[0]);
}

// Estimate the normal of an estimation_point as the arithmetic mean of the the
// normals of the vectors from estimation_point to each point in the
// sample_window.
float EstimateNormal(const sensor::PointCloud& returns,
                     const size_t estimation_point_index,
                     const size_t sample_window_begin,
                     const size_t sample_window_end,
                     const Eigen::Vector3f& sensor_origin) {
  const Eigen::Vector3f& estimation_point =
      returns[estimation_point_index].position;
  if (sample_window_end - sample_window_begin < 2) {
    return NormalTo2DAngle(sensor_origin - estimation_point);
  }
  Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
  const Eigen::Vector3f& estimation_point_to_observation =
      sensor_origin - estimation_point;
  for (size_t sample_point_index = sample_window_begin;
       sample_point_index < sample_window_end; ++sample_point_index) {
    if (sample_point_index == estimation_point_index) continue;
    const Eigen::Vector3f& sample_point = returns[sample_point_index].position;
    const Eigen::Vector3f& tangent = estimation_point - sample_point;
    Eigen::Vector3f sample_normal = {-tangent[1], tangent[0], 0.f};
    constexpr float kMinNormalLength = 1e-6f;
    if (sample_normal.norm() < kMinNormalLength) {
      continue;
    }
    // Ensure sample_normal points towards 'sensor_origin'.
    if (sample_normal.dot(estimation_point_to_observation) < 0) {
      sample_normal = -sample_normal;
    }
    sample_normal.normalize();
    mean_normal += sample_normal;
  }
  return NormalTo2DAngle(mean_normal);
}
}  // namespace

proto::NormalEstimationOptions2D CreateNormalEstimationOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::NormalEstimationOptions2D options;
  options.set_num_normal_samples(
      parameter_dictionary->GetInt("num_normal_samples"));
  options.set_sample_radius(parameter_dictionary->GetDouble("sample_radius"));
  CHECK_GT(options.num_normal_samples(), 0);
  CHECK_GT(options.sample_radius(), 0.0);
  return options;
}

// Estimates the normal for each 'return' in 'range_data'.
// Assumes the angles in the range data returns are sorted with respect to
// the orientation of the vector from 'origin' to 'return'.
std::vector<float> EstimateNormals(
    const sensor::RangeData& range_data,
    const proto::NormalEstimationOptions2D& normal_estimation_options) {
  std::vector<float> normals;
  normals.reserve(range_data.returns.size());
  const size_t max_num_samples = normal_estimation_options.num_normal_samples();
  const float sample_radius = normal_estimation_options.sample_radius();
  for (size_t current_point = 0; current_point < range_data.returns.size();
       ++current_point) {
    const Eigen::Vector3f& hit = range_data.returns[current_point].position;
    size_t sample_window_begin = current_point;
    for (; sample_window_begin > 0 &&
           current_point - sample_window_begin < max_num_samples / 2 &&
           (hit - range_data.returns[sample_window_begin - 1].position).norm() <
               sample_radius;
         --sample_window_begin) {
    }
    size_t sample_window_end = current_point;
    for (;
         sample_window_end < range_data.returns.size() &&
         sample_window_end - current_point < ceil(max_num_samples / 2.0) + 1 &&
         (hit - range_data.returns[sample_window_end].position).norm() <
             sample_radius;
         ++sample_window_end) {
    }
    const float normal_estimate =
        EstimateNormal(range_data.returns, current_point, sample_window_begin,
                       sample_window_end, range_data.origin);
    normals.push_back(normal_estimate);
  }
  return normals;
}

}  // namespace mapping
}  // namespace cartographer
