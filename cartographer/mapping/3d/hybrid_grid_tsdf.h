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

#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_TSDF_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_TSDF_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/tsd_value_converter.h"
#include "cartographer/mapping/3d/hybrid_grid_base.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/3d/hybrid_grid_tsdf.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

struct TSDFVoxel {
  uint16 discrete_tsd;
  uint16 discrete_weight;

 public:
  bool operator==(const TSDFVoxel& rhs) const
  {
    return (discrete_tsd == rhs.discrete_tsd) && (discrete_weight == rhs.discrete_weight);
  }
};

// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
// Points are expected to be close to the origin. Points far from the origin
// require the grid to grow dynamically. For centimeter resolution, points
// can only be tens of meters from the origin.
// The hard limit of cell indexes is +/- 8192 around the origin.
class HybridGridTSDF : public GridInterface, public HybridGridBase<TSDFVoxel> {
 public:
  explicit HybridGridTSDF(const float resolution,
                          float relative_truncation_distance, float max_weight,
                          ValueConversionTables* conversion_tables)
      : HybridGridBase<TSDFVoxel>(resolution),
        conversion_tables_(conversion_tables),
        value_converter_(absl::make_unique<TSDValueConverter>(
            relative_truncation_distance * resolution, max_weight,
            conversion_tables_)) {}

  explicit HybridGridTSDF(const proto::HybridGridTSDF& proto,
                          ValueConversionTables* conversion_tables)
      : HybridGridTSDF(proto.resolution(), proto.relative_truncation_distance(),
                       proto.max_weight(), conversion_tables) {
    CHECK_EQ(proto.values_tsd_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_tsd_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_tsd_size(), proto.z_indices_size());
    CHECK_EQ(proto.values_tsd_size(), proto.values_weight_size());
    for (int i = 0; i < proto.values_tsd_size(); ++i) {
      SetCell(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                              proto.z_indices(i)),
              ValueConverter().ValueToTSD(proto.values_tsd(i)),
              ValueConverter().ValueToWeight(proto.values_weight(i)));
    }
  }
  virtual GridType GetGridType() const override { return GridType::TSDF; };

  // Sets the probability of the cell at 'index' to the given 'probability'.
  void SetCell(const Eigen::Array3i& index, const float tsd,
               const float weight) {
    *mutable_value(index) = {
        static_cast<uint16>(value_converter_->TSDToValue(tsd) +
                            value_converter_->getUpdateMarker()),
        value_converter_->WeightToValue(weight)};
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(update_indices_.back()->discrete_tsd, kUpdateMarker);
      update_indices_.back()->discrete_tsd -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Returns the truncated signed distance of the cell with 'index'.
  float GetTSD(const Eigen::Array3i& index) const {
    return value_converter_->ValueToTSD(value(index).discrete_tsd);
  }

  // Returns the weight of the cell with 'index'.
  float GetWeight(const Eigen::Array3i& index) const {
    return value_converter_->ValueToWeight(value(index).discrete_weight);
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const {
    return value(index).discrete_weight != 0;
  }

  proto::HybridGridTSDF ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGridTSDF result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values_tsd(it.second.discrete_tsd);
      result.add_values_weight(it.second.discrete_weight);
    }
    result.set_max_weight(ValueConverter().getMaxWeight());
    result.set_relative_truncation_distance(ValueConverter().getMaxTSD());
    return result;
  }

  const TSDValueConverter& ValueConverter() const { return *value_converter_; }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;
  ValueConversionTables* conversion_tables_;
  std::unique_ptr<TSDValueConverter> value_converter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_TSDF_H_
