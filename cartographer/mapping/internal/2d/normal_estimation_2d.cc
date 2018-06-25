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

float Angle(const Eigen::Vector3f& v) { return std::atan2(v[1], v[0]); }

// Estimate the normal of an 'observation' as the arithmetic mean of the the
// normals of the vectors from 'observation' to each 'neighbor'.
float EstimateNormal(const Eigen::Vector3f& observation,
                     const std::vector<Eigen::Vector3f>& neighbors,
                     const Eigen::Vector3f& observation_origin) {
  if (neighbors.empty()) {
    return Angle(observation_origin - observation);
  }
  std::vector<float> normals;
  const Eigen::Vector3f origin_to_observation =
      observation_origin - observation;
  for (const auto& neighbor : neighbors) {
    const Eigen::Vector3f tangent = observation - neighbor;
    Eigen::Vector3f normal = {-tangent[1], tangent[0], tangent[2]};
    // Ensure normal points towards 'observation_origin'.
    if (normal.dot(origin_to_observation) < 0) {
      normal = -normal;
    }
    float normal_angle = Angle(normal);
    normals.push_back(normal_angle);
  }
  return common::CircularMean(normals);
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
void EstimateNormals(
    const sensor::RangeData& range_data, std::vector<float>* normals,
    const proto::NormalEstimationOptions2D& normal_estimation_options) {
  CHECK(normals != nullptr);
  const size_t max_num_samples = normal_estimation_options.num_normal_samples();
  const float sample_radius = normal_estimation_options.sample_radius();
  for (size_t i_hit = 0; i_hit < range_data.returns.size(); ++i_hit) {
    const Eigen::Vector3f& hit = range_data.returns[i_hit];
    size_t neighbors_lower_bound = i_hit;
    while (neighbors_lower_bound > 0 &&
           i_hit - neighbors_lower_bound < max_num_samples / 2 &&
           (hit - range_data.returns[neighbors_lower_bound - 1]).norm() <
               sample_radius) {
      neighbors_lower_bound--;
    }
    size_t neighbors_upper_bound = i_hit;
    while (neighbors_upper_bound < range_data.returns.size() - 1 &&
           neighbors_upper_bound - i_hit <
               max_num_samples / 2 + max_num_samples % 2 &&
           (hit - range_data.returns[neighbors_upper_bound + 1]).norm() <
               sample_radius) {
      neighbors_upper_bound++;
    }
    std::vector<Eigen::Vector3f> neighbors;
    const size_t n_to_upper_bound = neighbors_upper_bound - i_hit;
    const size_t n_to_lower_bound = i_hit - neighbors_lower_bound;
    for (size_t upper_offset = 1;
         i_hit + upper_offset < range_data.returns.size() &&
         upper_offset <= n_to_upper_bound;
         ++upper_offset) {
      neighbors.push_back(range_data.returns[i_hit + upper_offset]);
    }
    for (size_t lower_offset = 1;
         i_hit >= lower_offset && lower_offset <= n_to_lower_bound;
         ++lower_offset) {
      neighbors.push_back(range_data.returns[i_hit - lower_offset]);
    }
    const float normal_estimate =
        EstimateNormal(range_data.returns[i_hit], neighbors, range_data.origin);
    normals->push_back(normal_estimate);
  }
}

}  // namespace mapping
}  // namespace cartographer
