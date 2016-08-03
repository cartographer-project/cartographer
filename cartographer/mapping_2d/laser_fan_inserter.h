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

#ifndef CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_
#define CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/laser_fan_inserter_options.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {

proto::LaserFanInserterOptions CreateLaserFanInserterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

class LaserFanInserter {
 public:
  explicit LaserFanInserter(const proto::LaserFanInserterOptions& options);

  LaserFanInserter(const LaserFanInserter&) = delete;
  LaserFanInserter& operator=(const LaserFanInserter&) = delete;

  // Inserts 'laser_fan' into 'probability_grid'.
  void Insert(const sensor::LaserFan& laser_fan,
              ProbabilityGrid* probability_grid) const;

  const std::vector<uint16>& hit_table() const { return hit_table_; }
  const std::vector<uint16>& miss_table() const { return miss_table_; }

 private:
  const proto::LaserFanInserterOptions options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_
