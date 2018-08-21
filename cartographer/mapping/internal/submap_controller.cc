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

#include "cartographer/mapping/internal/submap_controller.h"

namespace cartographer {
namespace mapping {

template <>
std::shared_ptr<mapping::Submap2D>
SubmapController<mapping::Submap2D>::CreateSubmap(
    const mapping::proto::Submap& proto) {
  if (proto.submap_2d().num_range_data() != 1) {
    LOG(WARNING) << "Refusing to create partially filled submap: "
                 << proto.submap_2d().num_range_data();
    return nullptr;
  }
  return std::make_shared<mapping::Submap2D>(proto.submap_2d(),
                                             &conversion_tables_);
}

template <>
std::shared_ptr<mapping::Submap3D>
SubmapController<mapping::Submap3D>::CreateSubmap(
    const mapping::proto::Submap& proto) {
  if (proto.submap_3d().num_range_data() != 1) {
    LOG(INFO) << "Refusing to create partially filled submap: "
              << proto.submap_3d().num_range_data();
    return nullptr;
  }
  return std::make_shared<mapping::Submap3D>(proto.submap_3d());
}

}  // namespace mapping
}  // namespace cartographer
