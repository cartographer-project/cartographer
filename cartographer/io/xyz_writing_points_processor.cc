#include "cartographer/io/xyz_writing_points_processor.h"

#include <iomanip>

#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

void WriteXyzPoint(const Eigen::Vector3f& point, std::ofstream* output) {
  (*output) << point.x() << " " << point.y() << " " << point.z() << "\n";
}

}  // namespace

XyzWriterPointsProcessor::XyzWriterPointsProcessor(const string& filename,
                                                   PointsProcessor* next)
    : next_(next), file_(filename, std::ios_base::out | std::ios_base::binary) {
  file_ << std::setprecision(6);
}

PointsProcessor::FlushResult XyzWriterPointsProcessor::Flush() {
  file_.close();
  CHECK(file_) << "Writing XYZ file failed.";
  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "XYZ generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void XyzWriterPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  for (const Eigen::Vector3f& point : batch->points) {
    WriteXyzPoint(point, &file_);
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
