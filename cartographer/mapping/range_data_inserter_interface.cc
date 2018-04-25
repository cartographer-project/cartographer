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

#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

namespace cartographer {
namespace mapping {

proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::RangeDataInserterOptions options;
  const std::string range_data_inserter_type_string =
      parameter_dictionary->GetString("range_data_inserter_type");
  proto::RangeDataInserterOptions_RangeDataInserterType
      range_data_inserter_type;
  CHECK(proto::RangeDataInserterOptions_RangeDataInserterType_Parse(
      range_data_inserter_type_string, &range_data_inserter_type))
      << "Unknown RangeDataInserterOptions_RangeDataInserterType kind: "
      << range_data_inserter_type_string;
  options.set_range_data_inserter_type(range_data_inserter_type);
  *options.mutable_probability_grid_range_data_inserter_options_2d() =
      CreateProbabilityGridRangeDataInserterOptions2D(
          parameter_dictionary
              ->GetDictionary("probability_grid_range_data_inserter")
              .get());
  return options;
}
}  // namespace mapping
}  // namespace cartographer