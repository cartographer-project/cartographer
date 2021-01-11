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

#ifndef CARTOGRAPHER_MAPPING_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_
#define CARTOGRAPHER_MAPPING_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_

#include "Eigen/Geometry"

namespace cartographer {
namespace mapping {

// Calls Eigen::Quaterniond::FromTwoVectors(). This is in its own compilation
// unit since it can take more than 10 s to build while using more than 1 GB of
// memory causing slow build times and high peak memory usage.
Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_
