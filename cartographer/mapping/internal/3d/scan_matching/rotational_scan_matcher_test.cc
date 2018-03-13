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

#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

TEST(RotationalScanMatcher3DTest, OnlySameHistogramIsScoreOne) {
  Eigen::VectorXf histogram(7);
  histogram << 1.f, 43.f, 0.5f, 0.3123f, 23.f, 42.f, 0.f;
  RotationalScanMatcher matcher({{histogram, 0.f}});
  const auto scores = matcher.Match(histogram, 0.f, {0.f, 1.f});
  ASSERT_EQ(2, scores.size());
  EXPECT_NEAR(1.f, scores[0], 1e-6);
  EXPECT_GT(1.f, scores[1]);
}

TEST(RotationalScanMatcher3DTest, InterpolatesAsExpected) {
  constexpr int kNumBuckets = 10;
  constexpr float kAnglePerBucket = M_PI / kNumBuckets;
  constexpr float kNoInitialRotation = 0.f;
  RotationalScanMatcher matcher(
      {{Eigen::VectorXf::Unit(kNumBuckets, 3), kNoInitialRotation}});
  for (float t = 0.f; t < 1.f; t += 0.1f) {
    // 't' is the fraction of overlap and we have to divide by the norm of the
    // histogram to get the expected score.
    const float expected_score = t / std::hypot(t, 1 - t);
    // We rotate the 't'-th fraction of a bucket into the matcher's histogram.
    auto scores = matcher.Match(Eigen::VectorXf::Unit(kNumBuckets, 2),
                                kNoInitialRotation, {t * kAnglePerBucket});
    ASSERT_EQ(1, scores.size());
    EXPECT_NEAR(expected_score, scores[0], 1e-6);
    // Also verify rotating out of a bucket.
    scores = matcher.Match(Eigen::VectorXf::Unit(kNumBuckets, 2),
                           kNoInitialRotation, {(2 - t) * kAnglePerBucket});
    ASSERT_EQ(1, scores.size());
    EXPECT_NEAR(expected_score, scores[0], 1e-6);
    // And into and out of a bucket with negative angle.
    scores =
        matcher.Match(Eigen::VectorXf::Unit(kNumBuckets, 4), kNoInitialRotation,
                      {-t * kAnglePerBucket, (t - 2) * kAnglePerBucket});
    ASSERT_EQ(2, scores.size());
    EXPECT_NEAR(expected_score, scores[0], 1e-6);
    EXPECT_NEAR(expected_score, scores[1], 1e-6);
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
