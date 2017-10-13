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

#include "cartographer/mapping_3d/imu_integration.h"

namespace cartographer {
namespace mapping_3d {

IntegrateImuResult<double> IntegrateImu(
    const std::deque<sensor::ImuData>& imu_data, const common::Time start_time,
    const common::Time end_time,
    std::deque<sensor::ImuData>::const_iterator* it) {
  return IntegrateImu<double>(imu_data, Eigen::Affine3d::Identity(),
                              Eigen::Affine3d::Identity(), start_time, end_time,
                              it);
}

}  // namespace mapping_3d
}  // namespace cartographer
