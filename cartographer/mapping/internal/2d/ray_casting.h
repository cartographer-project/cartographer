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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_

#include <vector>

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// Compute all pixels in which some part of the line segment connecting 'begin'
// and 'end' lies with 'subpixel_scale' precision.
std::vector<Eigen::Array2i> CastRay(const Eigen::Array2i& begin,
                                    const Eigen::Array2i& end,
                                    int subpixel_scale);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_CASTING_H_
