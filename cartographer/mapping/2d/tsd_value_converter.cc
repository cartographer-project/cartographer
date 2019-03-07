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

#include "cartographer/mapping/2d/tsd_value_converter.h"

namespace cartographer {
namespace mapping {

TSDValueConverter::TSDValueConverter(float max_tsd, float max_weight,
                                     ValueConversionTables* conversion_tables)
    : max_tsd_(max_tsd),
      min_tsd_(-max_tsd),
      max_weight_(max_weight),
      tsd_resolution_(32766.f / (max_tsd_ - min_tsd_)),
      weight_resolution_(32766.f / (max_weight_ - min_weight_)),
      value_to_tsd_(
          conversion_tables->GetConversionTable(min_tsd_, min_tsd_, max_tsd_)),
      value_to_weight_(conversion_tables->GetConversionTable(
          min_weight_, min_weight_, max_weight)) {}

}  // namespace mapping
}  // namespace cartographer
