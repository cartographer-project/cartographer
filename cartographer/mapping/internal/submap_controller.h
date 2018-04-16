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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_SUBMAP_CONTROLLER_H
#define CARTOGRAPHER_MAPPING_INTERNAL_SUBMAP_CONTROLLER_H

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"

namespace cartographer {
namespace mapping {

template <class SubmapType>
class SubmapController {
 public:
  std::shared_ptr<SubmapType> UpdateSubmap(
      const mapping::proto::Submap& proto) {
    mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                                proto.submap_id().submap_index()};
    std::shared_ptr<SubmapType> submap_ptr;
    auto submap_it = unfinished_submaps_.find(submap_id);
    if (submap_it == unfinished_submaps_.end()) {
      submap_ptr = CreateSubmap(proto);
      unfinished_submaps_.Insert(submap_id, submap_ptr);
      return submap_ptr;
    }
    submap_ptr = submap_it->data;
    CHECK(submap_ptr);
    submap_ptr->UpdateFromProto(proto);

    // If the submap was just finished by the recent update, remove it from
    // the list of unfinished submaps.
    if (submap_ptr->finished()) {
      unfinished_submaps_.Trim(submap_id);
    } else {
      // If the submap is unfinished set the 'num_range_data' to 0 since we
      // haven't changed the HybridGrid.
      submap_ptr->set_num_range_data(0);
    }
    return submap_ptr;
  }

 private:
  std::shared_ptr<SubmapType> CreateSubmap(const mapping::proto::Submap& proto);

  mapping::MapById<mapping::SubmapId, std::shared_ptr<SubmapType>>
      unfinished_submaps_;
};

template <>
std::shared_ptr<mapping::Submap2D>
SubmapController<mapping::Submap2D>::CreateSubmap(
    const mapping::proto::Submap& proto);
template <>
std::shared_ptr<mapping::Submap3D>
SubmapController<mapping::Submap3D>::CreateSubmap(
    const mapping::proto::Submap& proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_SUBMAP_CONTROLLER_H
