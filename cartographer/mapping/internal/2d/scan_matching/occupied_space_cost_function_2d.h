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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
