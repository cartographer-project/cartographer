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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(
    trajectory_filename, "",
    "Proto containing the trajectory of which to assess the quality.");
DEFINE_string(relations_filename, "",
              "Relations file containing the ground truth.");

namespace cartographer {
namespace ground_truth {
namespace {

transform::Rigid3d LookupPose(const transform::TransformInterpolationBuffer&
                                  transform_interpolation_buffer,
                              const double time) {
  constexpr int64 kUtsTicksPerSecond = 10000000;
  common::Time common_time =
      common::FromUniversal(common::kUtsEpochOffsetFromUnixEpochInSeconds *
                            kUtsTicksPerSecond) +
      common::FromSeconds(time);
  return transform_interpolation_buffer.Lookup(common_time);
}

struct Error {
  double translational_squared;
  double rotational_squared;
};

Error ComputeError(const transform::Rigid3d& pose1,
                   const transform::Rigid3d& pose2,
                   const transform::Rigid3d& expected) {
  const transform::Rigid3d error =
      (pose1.inverse() * pose2) * expected.inverse();
  return Error{error.translation().squaredNorm(),
               common::Pow2(transform::GetAngle(error))};
}

string MeanAndStdDevString(const std::vector<double>& values) {
  CHECK_GE(values.size(), 2);
  const double mean =
      std::accumulate(values.begin(), values.end(), 0.) / values.size();
  double sum_of_squared_differences = 0.;
  for (const double value : values) {
    sum_of_squared_differences += common::Pow2(value - mean);
  }
  const double standard_deviation =
      std::sqrt(sum_of_squared_differences / (values.size() - 1));
  std::ostringstream out;
  out << std::fixed << std::setprecision(5) << mean << " +/- "
      << standard_deviation;
  return string(out.str());
}

string StatisticsString(const std::vector<Error>& errors) {
  std::vector<double> translational_errors;
  std::vector<double> squared_translational_errors;
  std::vector<double> rotational_errors_degrees;
  std::vector<double> squared_rotational_errors_degrees;
  for (const Error& error : errors) {
    translational_errors.push_back(std::sqrt(error.translational_squared));
    squared_translational_errors.push_back(error.translational_squared);
    rotational_errors_degrees.push_back(
        common::RadToDeg(std::sqrt(error.rotational_squared)));
    squared_rotational_errors_degrees.push_back(
        common::Pow2(rotational_errors_degrees.back()));
  }
  return "Abs translational error " +
         MeanAndStdDevString(translational_errors) +
         " m\n"
         "Sqr translational error " +
         MeanAndStdDevString(squared_translational_errors) +
         " m^2\n"
         "Abs rotational error " +
         MeanAndStdDevString(rotational_errors_degrees) +
         " deg\n"
         "Sqr rotational error " +
         MeanAndStdDevString(squared_rotational_errors_degrees) + " deg^2\n";
}

void Run(const string& trajectory_filename, const string& relations_filename) {
  LOG(INFO) << "Reading trajectory from '" << trajectory_filename << "'...";
  mapping::proto::Trajectory trajectory_proto;
  {
    std::ifstream trajectory_stream(trajectory_filename.c_str(),
                                    std::ios::binary);
    CHECK(trajectory_proto.ParseFromIstream(&trajectory_stream));
  }

  const auto transform_interpolation_buffer =
      transform::TransformInterpolationBuffer::FromTrajectory(trajectory_proto);

  LOG(INFO) << "Reading relations from '" << relations_filename << "'...";
  std::vector<Error> errors;
  {
    std::ifstream relations_stream(relations_filename.c_str());
    double time1, time2, x, y, z, roll, pitch, yaw;
    while (relations_stream >> time1 >> time2 >> x >> y >> z >> roll >> pitch >>
           yaw) {
      const auto pose1 = LookupPose(*transform_interpolation_buffer, time1);
      const auto pose2 = LookupPose(*transform_interpolation_buffer, time2);
      const transform::Rigid3d expected =
          transform::Rigid3d(transform::Rigid3d::Vector(x, y, z),
                             transform::RollPitchYaw(roll, pitch, yaw));
      errors.push_back(ComputeError(pose1, pose2, expected));
    }
    CHECK(relations_stream.eof());
  }

  LOG(INFO) << "Result:\n" << StatisticsString(errors);
}

}  // namespace
}  // namespace ground_truth
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program computes the relation based metric described in:\n"
      "R. Kuemmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti,\n"
      "C. Stachniss, and A. Kleiner, \"On measuring the accuracy of SLAM\n"
      "algorithms,\" Autonomous Robots, vol. 27, no. 4, pp. 387–407, 2009.\n"
      "\n"
      "Note: Timestamps in the relations_file are interpreted relative to\n"
      "      the Unix epoch.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_trajectory_filename.empty() || FLAGS_relations_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "compute_relations_metrics");
    return EXIT_FAILURE;
  }
  ::cartographer::ground_truth::Run(FLAGS_trajectory_filename,
                                    FLAGS_relations_filename);
}
