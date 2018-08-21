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

#ifndef CARTOGRAPHER_COMMON_UTILS_H_
#define CARTOGRAPHER_COMMON_UTILS_H_

namespace cartographer {
namespace common {

template <typename MapType, typename KeyType = typename MapType::key_type,
          typename ValueType = typename MapType::mapped_type>
ValueType* FindOrNull(MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it == map.end()) return nullptr;
  return &(it->second);
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_UTILS_H_
