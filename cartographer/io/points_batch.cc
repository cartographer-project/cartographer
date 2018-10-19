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

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

void RemovePoints(absl::flat_hash_set<int> to_remove, PointsBatch* batch) {
  const int new_num_points = batch->points.size() - to_remove.size();
  sensor::PointCloud points;
  points.reserve(new_num_points);
  std::vector<float> intensities;
  if (!batch->intensities.empty()) {
    intensities.reserve(new_num_points);
  }
  std::vector<FloatColor> colors;
  if (!batch->colors.empty()) {
    colors.reserve(new_num_points);
  }

  for (size_t i = 0; i < batch->points.size(); ++i) {
    if (to_remove.count(i) == 1) {
      continue;
    }
    points.push_back(batch->points[i]);
    if (!batch->colors.empty()) {
      colors.push_back(batch->colors[i]);
    }
    if (!batch->intensities.empty()) {
      intensities.push_back(batch->intensities[i]);
    }
  }
  batch->points = std::move(points);
  batch->intensities = std::move(intensities);
  batch->colors = std::move(colors);
}

}  // namespace io
}  // namespace cartographer
