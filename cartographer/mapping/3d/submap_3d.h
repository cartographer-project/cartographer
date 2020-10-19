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

#ifndef CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_
#define CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/submaps_options_3d.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions3D CreateSubmapsOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap3D : public Submap {
 public:
  Submap3D(float high_resolution, float low_resolution,
           const transform::Rigid3d& local_submap_pose,
           const Eigen::VectorXf& rotational_scan_matcher_histogram);

  explicit Submap3D(const proto::Submap3D& proto);

  proto::Submap ToProto(bool include_probability_grid_data) const override;
  void UpdateFromProto(const proto::Submap& proto) override;

  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;

  const HybridGrid& high_resolution_hybrid_grid() const {
    return *high_resolution_hybrid_grid_;
  }
  const HybridGrid& low_resolution_hybrid_grid() const {
    return *low_resolution_hybrid_grid_;
  }
  const IntensityHybridGrid& high_resolution_intensity_hybrid_grid() const {
    CHECK(high_resolution_intensity_hybrid_grid_ != nullptr);
    return *high_resolution_intensity_hybrid_grid_;
  }
  void ForgetIntensityHybridGrid() {
    high_resolution_intensity_hybrid_grid_.reset();
  }

  const Eigen::VectorXf& rotational_scan_matcher_histogram() const {
    return rotational_scan_matcher_histogram_;
  }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertData(const sensor::RangeData& range_data,
                  const RangeDataInserter3D& range_data_inserter,
                  float high_resolution_max_range,
                  const Eigen::Quaterniond& local_from_gravity_aligned,
                  const Eigen::VectorXf& scan_histogram_in_gravity);

  void Finish();

 private:
  void UpdateFromProto(const proto::Submap3D& submap_3d);

  std::unique_ptr<HybridGrid> high_resolution_hybrid_grid_;
  std::unique_ptr<HybridGrid> low_resolution_hybrid_grid_;
  std::unique_ptr<IntensityHybridGrid> high_resolution_intensity_hybrid_grid_;
  Eigen::VectorXf rotational_scan_matcher_histogram_;
};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps3D {
 public:
  explicit ActiveSubmaps3D(const proto::SubmapsOptions3D& options);

  ActiveSubmaps3D(const ActiveSubmaps3D&) = delete;
  ActiveSubmaps3D& operator=(const ActiveSubmaps3D&) = delete;

  // Inserts 'range_data_in_local' into the Submap collection.
  // 'local_from_gravity_aligned' is used for the orientation of new submaps so
  // that the z axis approximately aligns with gravity.
  // 'rotational_scan_matcher_histogram_in_gravity' will be accumulated in all
  // submaps of the Submap collection.
  std::vector<std::shared_ptr<const Submap3D>> InsertData(
      const sensor::RangeData& range_data_in_local,
      const Eigen::Quaterniond& local_from_gravity_aligned,
      const Eigen::VectorXf& rotational_scan_matcher_histogram_in_gravity);

  std::vector<std::shared_ptr<const Submap3D>> submaps() const;

 private:
  void AddSubmap(const transform::Rigid3d& local_submap_pose,
                 int rotational_scan_matcher_histogram_size);

  const proto::SubmapsOptions3D options_;
  std::vector<std::shared_ptr<Submap3D>> submaps_;
  RangeDataInserter3D range_data_inserter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SUBMAP_3D_H_
