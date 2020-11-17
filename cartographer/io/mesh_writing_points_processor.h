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

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#ifdef WITH_OPEN3D
#include "Open3D/Open3D.h"
#endif

namespace cartographer {
  namespace io {

    // Streams a PLY file to disk. The header is written in 'Flush'.
    class MeshWritingPointsProcessor : public PointsProcessor {
    public:
      constexpr static const char* kConfigurationFileActionName = "write_mesh";
      MeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                const size_t &aggregate,
                                const int64 &poisson_depth,
                                const double &trim_surface,
                                const int64 &statistical_outlier_neighbours,
                                const double &statistical_outlier_radius,
                                const std::vector<std::string>& comments,
                                PointsProcessor* next);

      static std::unique_ptr<MeshWritingPointsProcessor> FromDictionary(
              const FileWriterFactory& file_writer_factory,
              common::LuaParameterDictionary* dictionary, PointsProcessor* next);

      ~MeshWritingPointsProcessor() override {}

      MeshWritingPointsProcessor(const MeshWritingPointsProcessor&) = delete;
      MeshWritingPointsProcessor& operator=(const MeshWritingPointsProcessor&) =
      delete;

      void Process(std::unique_ptr<PointsBatch> batch) override;
      FlushResult Flush() override;

    private:
      PointsProcessor* const next_;
      size_t aggregate_;
      int64 poisson_depth_;
      double trim_surface_;
      int64 statistical_outlier_neighbours_;
      double statistical_outlier_radius_;
      int64 aggregation_counter_ = 0;
      std::vector<std::string> comments_;
      int64 num_points_;
      std::string name_;
      common::Time currentTime_;
      bool has_colors_;
      bool has_intensities_;
      std::unique_ptr<FileWriter> file_;

#ifdef WITH_OPEN3D
      std::shared_ptr<open3d::geometry::PointCloud> pc_;
      std::shared_ptr<open3d::geometry::PointCloud> resultpc_;
#endif
    };

  }  // namespace io
}  // namespace cartographer
