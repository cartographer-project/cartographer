#include "cartographer/io/xyz_writing_points_processor.h"

#include <iomanip>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

void WriteXyzPoint(const Eigen::Vector3f& point, File* file) {
  std::ostringstream stream;
  stream << std::setprecision(6);
  stream << point.x() << " " << point.y() << " " << point.z() << "\n";
  const string out = stream.str();
  CHECK(file->Write(out.data(), out.size()));
}

}  // namespace

XyzWriterPointsProcessor::XyzWriterPointsProcessor(std::unique_ptr<File> file,
                                                   PointsProcessor* next)
    : next_(next), file_(std::move(file)) {}

std::unique_ptr<XyzWriterPointsProcessor>
XyzWriterPointsProcessor::FromDictionary(
    const FileFactory& file_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<XyzWriterPointsProcessor>(
      file_factory(dictionary->GetString("filename")), next);
}

PointsProcessor::FlushResult XyzWriterPointsProcessor::Flush() {
  CHECK(file_->Close()) << "Closing XYZ file failed.";
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
    WriteXyzPoint(point, file_.get());
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
