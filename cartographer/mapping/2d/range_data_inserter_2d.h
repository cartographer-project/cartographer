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

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/proto/range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

proto::RangeDataInserterOptions2D CreateRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class RangeDataInserter2D {
 public:
  explicit RangeDataInserter2D(
      const proto::RangeDataInserterOptions2D& options);

  RangeDataInserter2D(const RangeDataInserter2D&) = delete;
  RangeDataInserter2D& operator=(const RangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  void Insert(const sensor::RangeData& range_data,
              ProbabilityGrid* probability_grid) const;

 private:
  const proto::RangeDataInserterOptions2D options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_H_
