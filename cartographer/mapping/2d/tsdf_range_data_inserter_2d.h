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

class RangeDataInserter2DTSDF : public RangeDataInserterInterface {
 public:
  explicit RangeDataInserter2DTSDF(
      const proto::TSDFRangeDataInserterOptions2D& options);

  RangeDataInserter2DTSDF(const RangeDataInserter2DTSDF&) = delete;
  RangeDataInserter2DTSDF& operator=(const RangeDataInserter2DTSDF&) = delete;

  // Inserts 'range_data' into 'grid'.
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const override;

  void ComputeNormals(sensor::RangeData& range_data,
                      std::vector<float>* normals) const;

 private:
  void UpdateCell(TSDF2D* const tsdf, const Eigen::Array2i& cell,
                  float update_sdf, float ray_length,
                  float update_weight_scale) const;
  float ComputeWeightConstant(float sdf) const;
  float ComputeWeightLinear(float sdf, float ray_length) const;
  float ComputeWeightQuadratic(float sdf, float ray_length) const;
  const proto::TSDFRangeDataInserterOptions2D options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_TSDF_RANGE_DATA_INSERTER_2D_H_