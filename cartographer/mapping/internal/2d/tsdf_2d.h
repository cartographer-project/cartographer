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

#ifndef CARTOGRAPHER_MAPPING_2D_TSDF_2D_H_
#define CARTOGRAPHER_MAPPING_2D_TSDF_2D_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/tsd_value_converter.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of truncated signed distances and weights.
class TSDF2D : public Grid2D {
 public:
  TSDF2D(const MapLimits& limits, float truncation_distance, float max_weight,
         ValueConversionTables* conversion_tables);
  explicit TSDF2D(const proto::Grid2D& proto,
                  ValueConversionTables* conversion_tables);

  void SetCell(const Eigen::Array2i& cell_index, const float tsd,
               const float weight);
  GridType GetGridType() const override;
  float GetTSD(const Eigen::Array2i& cell_index) const;
  float GetWeight(const Eigen::Array2i& cell_index) const;
  std::pair<float, float> GetTSDAndWeight(
      const Eigen::Array2i& cell_index) const;

  void GrowLimits(const Eigen::Vector2f& point) override;
  proto::Grid2D ToProto() const override;
  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const override;
  bool CellIsUpdated(const Eigen::Array2i& cell_index) const;

 private:
  ValueConversionTables* conversion_tables_;
  std::unique_ptr<TSDValueConverter> value_converter_;
  std::vector<uint16> weight_cells_;  // Highest bit is update marker.
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_TSDF_2D_H_
