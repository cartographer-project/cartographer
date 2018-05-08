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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/proto/2d/grid_2d.pb.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

class Grid2D : public GridInterface {
 public:
  explicit Grid2D(const MapLimits& limits, float min_correspondence_cost,
                  float max_correspondence_cost);
  explicit Grid2D(const proto::Grid2D& proto);

  // Returns the limits of this Grid2D.
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate();
  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& cell_index) const;

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f& point);

  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

  virtual proto::Grid2D ToProto() const;

  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 protected:
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  const std::vector<int>& update_indices() const { return update_indices_; }
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }
  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  int ToFlatIndex(const Eigen::Array2i& cell_index) const;

 private:
  MapLimits limits_;
  std::vector<uint16> correspondence_cost_cells_;
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
