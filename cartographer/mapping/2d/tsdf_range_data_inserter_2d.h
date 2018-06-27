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

#ifndef CARTOGRAPHER_MAPPING_2D_TSDF_RANGE_DATA_INSERTER_2D_H_
#define CARTOGRAPHER_MAPPING_2D_TSDF_RANGE_DATA_INSERTER_2D_H_

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/tsdf_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

proto::TSDFRangeDataInserterOptions2D CreateTSDFRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class TSDFRangeDataInserter2D : public RangeDataInserterInterface {
 public:
  explicit TSDFRangeDataInserter2D(
      const proto::TSDFRangeDataInserterOptions2D& options);

  TSDFRangeDataInserter2D(const TSDFRangeDataInserter2D&) = delete;
  TSDFRangeDataInserter2D& operator=(const TSDFRangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'grid'.
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const override;

 private:
  void UpdateCell(TSDF2D* const tsdf, const Eigen::Array2i& cell,
                  float update_sdf, float update_weight) const;
  float ComputeWeight(float ray_length, int exponent) const;
  const proto::TSDFRangeDataInserterOptions2D options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_TSDF_RANGE_DATA_INSERTER_2D_H_