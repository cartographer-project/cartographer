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

#ifndef CARTOGRAPHER_IO_PROBABILITY_GRID_UTILS_H_
#define CARTOGRAPHER_IO_PROBABILITY_GRID_UTILS_H_

#include <memory>

#include "cartographer/io/image.h"
#include "cartographer/mapping/2d/probability_grid.h"

namespace cartographer {
namespace io {

// Draws 'probability_grid' into an image and fills in 'offset' with the cropped
// map limits. Returns 'nullptr' if probability_grid was empty.
std::unique_ptr<Image> DrawProbabilityGrid(
    const mapping::ProbabilityGrid& probability_grid, Eigen::Array2i* offset);

// Create an initially empty probability grid with 'resolution' and a small
// size, suitable for a PointsBatchProcessor.
mapping::ProbabilityGrid CreateProbabilityGrid(const double resolution);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROBABILITY_GRID_UTILS_H_
