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

#include "cartographer/mapping/submaps.h"

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

Submaps::Submaps() {}

Submaps::~Submaps() {}

int Submaps::matching_index() const {
  if (size() > 1) {
    return size() - 2;
  }
  return size() - 1;
}

std::vector<int> Submaps::insertion_indices() const {
  if (size() > 1) {
    return {size() - 2, size() - 1};
  }
  return {size() - 1};
}

}  // namespace mapping
}  // namespace cartographer
