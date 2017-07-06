#ifndef CARTOGRAPHER_IO_IMAGE_UTILS_H_
#define CARTOGRAPHER_IO_IMAGE_UTILS_H_

#include <cstdint>
#include <vector>

namespace cartographer {
namespace io {

class FileWriter;

struct Image
{
  int width;
  int height;
  int stride;
  std::vector<uint32_t> pixels;
};

Image CreateImage(int width, int height);

void WritePng(Image * const image, FileWriter * const file_writer);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_IMAGE_UTILS_H_
