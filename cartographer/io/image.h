#ifndef CARTOGRAPHER_IO_IMAGE_H_
#define CARTOGRAPHER_IO_IMAGE_H_

#include <cstdint>
#include <vector>

#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

class Image {
 public:
  Image(int width, int height);

  const Color GetPixel(int x, int y) const;
  void SetPixel(int x, int y, const Color& color);
  void WritePng(FileWriter* const file_writer);

 private:
  int width_;
  int height_;
  int stride_;
  std::vector<uint32_t> pixels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_IMAGE_H_
