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
#include "cartographer/ground_truth/proto/relations.pb.h"
#include "cartographer/ground_truth/relations_text_file.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(
    pose_graph_filename, "",
    "Proto stream file containing the pose graph used to assess quality.");
DEFINE_string(relations_filename, "",
              "Relations file containing the ground truth.");
DEFINE_bool(read_text_file_with_unix_timestamps, false,
            "Enable support for the relations text files as in the paper. "
            "Default is to read from a GroundTruth proto file.");
DEFINE_bool(write_relation_metrics, false,
            "Enable exporting relation metrics as comma-separated values to "
            "[pose_graph_filename].relation_metrics.csv");

namespace cartographer {
namespace ground_truth {
namespace {

struct Error {
  double translational_squared;
  double rotational_squared;
};

// TODO(whess): This gives different results for the translational error if
// 'pose1' and 'pose2' are swapped and 'expected' is inverted. Consider a
// different way to compute translational error. Maybe just look at the
// absolute difference in translation norms of each relative transform as a
// lower bound of the translational error.
Error ComputeError(const transform::Rigid3d& pose1,
                   const transform::Rigid3d& pose2,
                   const transform::Rigid3d& expected) {
  const transform::Rigid3d error =
      (pose1.inverse() * pose2) * expected.inverse();
  return Error{error.translation().squaredNorm(),
               common::Pow2(transform::GetAngle(error))};
}

std::string MeanAndStdDevString(const std::vector<double>& values) {
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
  return std::string(out.str());
}

std::string StatisticsString(const std::vector<Error>& errors) {
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

void WriteRelationMetricsToFile(const std::vector<Error>& errors,
                                const proto::GroundTruth& ground_truth,
                                const std::string& relation_metrics_filename) {
  std::ofstream relation_errors_file;
  std::string log_file_path;
  LOG(INFO) << "Writing relation metrics to '" + relation_metrics_filename +
                   "'...";
  relation_errors_file.open(relation_metrics_filename);
  relation_errors_file
      << "translational_error,squared_translational_error,rotational_"
         "errors_degree,squared_rotational_errors_degree,"
         "expected_translation_x,expected_translation_y,expected_"
         "translation_z,expected_rotation_w,expected_rotation_x,"
         "expected_rotation_y,expected_rotation_z,covered_distance\n";
  for (int relation_index = 0; relation_index < ground_truth.relation_size();
       ++relation_index) {
    const Error& error = errors[relation_index];
    const proto::Relation& relation = ground_truth.relation(relation_index);
    double translational_error = std::sqrt(error.translational_squared);
    double squared_translational_error = error.translational_squared;
    double rotational_errors_degree =
        common::RadToDeg(std::sqrt(error.rotational_squared));
    double squared_rotational_errors_degree =
        common::Pow2(rotational_errors_degree);
    relation_errors_file << translational_error << ","
                         << squared_translational_error << ","
                         << rotational_errors_degree << ","
                         << squared_rotational_errors_degree << ","
                         << relation.expected().translation().x() << ","
                         << relation.expected().translation().y() << ","
                         << relation.expected().translation().z() << ","
                         << relation.expected().rotation().w() << ","
                         << relation.expected().rotation().x() << ","
                         << relation.expected().rotation().y() << ","
                         << relation.expected().rotation().z() << ","
                         << relation.covered_distance() << "\n";
  }
  relation_errors_file.close();
}

transform::Rigid3d LookupTransform(
    const transform::TransformInterpolationBuffer&
        transform_interpolation_buffer,
    const common::Time time) {
  const common::Time earliest_time =
      transform_interpolation_buffer.earliest_time();
  if (transform_interpolation_buffer.Has(time)) {
    return transform_interpolation_buffer.Lookup(time);
  } else if (time < earliest_time) {
    return transform_interpolation_buffer.Lookup(earliest_time);
  }
  return transform_interpolation_buffer.Lookup(
      transform_interpolation_buffer.latest_time());
}

void Run(const std::string& pose_graph_filename,
         const std::string& relations_filename,
         const bool read_text_file_with_unix_timestamps,
         const bool write_relation_metrics) {
  LOG(INFO) << "Reading pose graph from '" << pose_graph_filename << "'...";
  mapping::proto::PoseGraph pose_graph =
      io::DeserializePoseGraphFromFile(pose_graph_filename);

  const transform::TransformInterpolationBuffer transform_interpolation_buffer(
      pose_graph.trajectory(0));

  proto::GroundTruth ground_truth;
  if (read_text_file_with_unix_timestamps) {
    LOG(INFO) << "Reading relations from '" << relations_filename << "'...";
    ground_truth = ReadRelationsTextFile(relations_filename);
  } else {
    LOG(INFO) << "Reading ground truth from '" << relations_filename << "'...";
    std::ifstream ground_truth_stream(relations_filename.c_str(),
                                      std::ios::binary);
    CHECK(ground_truth.ParseFromIstream(&ground_truth_stream));
  }

  std::vector<Error> errors;
  for (const auto& relation : ground_truth.relation()) {
    const auto pose1 =
        LookupTransform(transform_interpolation_buffer,
                        common::FromUniversal(relation.timestamp1()));
    const auto pose2 =
        LookupTransform(transform_interpolation_buffer,
                        common::FromUniversal(relation.timestamp2()));
    const transform::Rigid3d expected =
        transform::ToRigid3(relation.expected());
    errors.push_back(ComputeError(pose1, pose2, expected));
  }

  const std::string relation_metrics_filename =
      pose_graph_filename + ".relation_metrics.csv";
  if (write_relation_metrics) {
    WriteRelationMetricsToFile(errors, ground_truth, relation_metrics_filename);
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
      "algorithms,\" Autonomous Robots, vol. 27, no. 4, pp. 387–407, 2009.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pose_graph_filename.empty() || FLAGS_relations_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "compute_relations_metrics");
    return EXIT_FAILURE;
  }

  ::cartographer::ground_truth::Run(
      FLAGS_pose_graph_filename, FLAGS_relations_filename,
      FLAGS_read_text_file_with_unix_timestamps, FLAGS_write_relation_metrics);
}
