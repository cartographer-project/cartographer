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

#include "cartographer/mapping/probability_values.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(ProbabilityValuesTest, OddsConversions) {
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMinProbability)), kMinProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMaxProbability)), kMaxProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(0.5)), 0.5, 1e-6);
}

TEST(ProbabilityValuesTest, OddsConversionsCorrespondenceCost) {
  EXPECT_NEAR(ProbabilityToCorrespondenceCost(ProbabilityFromOdds(Odds(
                  CorrespondenceCostToProbability(kMaxCorrespondenceCost)))),
              kMaxCorrespondenceCost, 1e-6);
  EXPECT_NEAR(ProbabilityToCorrespondenceCost(ProbabilityFromOdds(Odds(
                  CorrespondenceCostToProbability(kMinCorrespondenceCost)))),
              kMinCorrespondenceCost, 1e-6);
  EXPECT_NEAR(ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                  Odds(CorrespondenceCostToProbability(0.5)))),
              0.5, 1e-6);
}

TEST(ProbabilityValuesTest,
     ProbabilityValueToCorrespondenceCostValueConversions) {
  for (uint16 i = 0; i < 32768; ++i) {
    EXPECT_EQ(ProbabilityValueToCorrespondenceCostValue(
                  CorrespondenceCostValueToProbabilityValue(i)),
              i);
    EXPECT_EQ(CorrespondenceCostValueToProbabilityValue(
                  ProbabilityValueToCorrespondenceCostValue(i)),
              i);
  }
}

TEST(ProbabilityValuesTest,
     ProbabilityValueToCorrespondenceCostValueConversionsWithUpdateMarker) {
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_EQ(ProbabilityValueToCorrespondenceCostValue(
                  CorrespondenceCostValueToProbabilityValue(i + kUpdateMarker)),
              i + kUpdateMarker);
    EXPECT_EQ(CorrespondenceCostValueToProbabilityValue(
                  ProbabilityValueToCorrespondenceCostValue(i + kUpdateMarker)),
              i + kUpdateMarker);
  }
}

TEST(ProbabilityValuesTest, ConversionLookUpTable) {
  EXPECT_NEAR(ValueToProbability(0), 1.f - ValueToCorrespondenceCost(0), 1e-6);
  for (uint16 i = 1; i < 32768; ++i) {
    EXPECT_NEAR(ValueToProbability(i), ValueToCorrespondenceCost(i), 1e-6)
        << " i " << i;
  }
}

TEST(ProbabilityValuesTest, CellUpdate) {
  std::vector<uint16> probability_table =
      ComputeLookupTableToApplyOdds(Odds(0.9f));
  std::vector<uint16> correspondence_table =
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.9f));
  uint16 cell_pg_pre_update = 0;
  uint16 cell_cg_pre_update = 0;
  uint16 cell_pg_post_update = probability_table[cell_pg_pre_update];
  uint16 cell_cg_post_update = correspondence_table[cell_cg_pre_update];
  float p_post = ValueToProbability(cell_pg_post_update);
  float c_post = ValueToCorrespondenceCost(cell_cg_post_update);
  EXPECT_NEAR(p_post, 1.f - c_post, 1e-6);
  int num_evaluations = 5000;
  for (int i_probability = 0; i_probability < num_evaluations;
       ++i_probability) {
    float p = (static_cast<float>(i_probability) /
               static_cast<float>(num_evaluations)) *
                  (kMaxProbability - kMinProbability) +
              kMinProbability;
    cell_pg_pre_update = ProbabilityToValue(p);
    cell_cg_pre_update =
        CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(p));
    float p_value =
        (common::Clamp(p, kMinProbability, kMaxProbability) - kMinProbability) *
        (32766.f / (kMaxProbability - kMinProbability));
    float cvalue = (common::Clamp(ProbabilityToCorrespondenceCost(p),
                                  kMinProbability, kMaxProbability) -
                    kMinProbability) *
                   (32766.f / (kMaxProbability - kMinProbability));

    EXPECT_NEAR(cell_pg_pre_update, 32768 - cell_cg_pre_update, 1)
        << "p " << p << " p_value " << p_value << " cvalue " << cvalue
        << " p_valuei " << common::RoundToInt(p_value) << " cvaluei "
        << common::RoundToInt(cvalue);
    cell_pg_post_update = probability_table[cell_pg_pre_update];
    cell_cg_post_update = correspondence_table[cell_cg_pre_update];
    p_post = ValueToProbability(cell_pg_post_update);
    c_post = ValueToCorrespondenceCost(cell_cg_post_update);
    EXPECT_NEAR(p_post, 1.f - c_post, 5e-5)
        << "p " << p << " " << p_post - 1.f + c_post;
  }
}

TEST(ProbabilityValuesTest, MultipleCellUpdate) {
  std::vector<uint16> probability_table =
      ComputeLookupTableToApplyOdds(Odds(0.55f));
  std::vector<uint16> correspondence_table =
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.55f));
  uint16 cell_pg_post_update = probability_table[0];
  uint16 cell_cg_post_update = correspondence_table[0];
  float p_post = ValueToProbability(cell_pg_post_update);
  float c_post = ValueToCorrespondenceCost(cell_cg_post_update);
  EXPECT_NEAR(p_post, 1.f - c_post, 1e-6);
  int num_evaluations = 5000;
  for (int i_probability = 0; i_probability < num_evaluations;
       ++i_probability) {
    float p = (static_cast<float>(i_probability) /
               static_cast<float>(num_evaluations)) *
                  (kMaxProbability - kMinProbability) +
              kMinProbability;
    cell_pg_post_update = ProbabilityToValue(p) + kUpdateMarker;
    cell_cg_post_update =
        CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(p)) +
        kUpdateMarker;
    for (int i_update = 0; i_update < 20; ++i_update) {
      cell_pg_post_update =
          probability_table[cell_pg_post_update - kUpdateMarker];
      cell_cg_post_update =
          correspondence_table[cell_cg_post_update - kUpdateMarker];
    }
    p_post = ValueToProbability(cell_pg_post_update);
    c_post = ValueToCorrespondenceCost(cell_cg_post_update);
    EXPECT_NEAR(p_post, 1.f - c_post, 5e-5)
        << "p " << p << " p_post " << p_post << " " << p_post - 1.f + c_post;
  }
}

TEST(ProbabilityValuesTest, EqualityLookupTableToApplyOdds) {
  std::vector<uint16> probability_table = ComputeLookupTableToApplyOdds(0.3);
  std::vector<uint16> correspondence_table =
      ComputeLookupTableToApplyCorrespondenceCostOdds(0.3);
  for (int i = 0; i < 32768; ++i) {
    EXPECT_NEAR(
        probability_table[i],
        CorrespondenceCostValueToProbabilityValue(
            correspondence_table[ProbabilityValueToCorrespondenceCostValue(i)]),
        1);
    EXPECT_NEAR(
        ProbabilityValueToCorrespondenceCostValue(
            probability_table[CorrespondenceCostValueToProbabilityValue(i)]),
        correspondence_table[i], 1);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer