#ifndef CARTOGRAPHER_IO_IMAGE_H_
#define CARTOGRAPHER_IO_IMAGE_H_

#include <cstdint>
#include <vector>

#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/color.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueCairoSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

// Takes ownership.
UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t* surface);

// std::unique_ptr for Cairo contexts.
using UniqueCairoPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// Takes ownership.
UniqueCairoPtr MakeUniqueCairoPtr(cairo_t* surface);

class Image {
 public:
  Image(int width, int height);

  const Uint8Color GetPixel(int x, int y) const;
  void SetPixel(int x, int y, const Uint8Color& color);
  void WritePng(FileWriter* const file_writer);

  // Returns a pointer to a cairo surface that contains the current pixel data.
  // The 'Image' object must therefore outlive the returned surface object. It
  // is undefined behavior to call any of the mutating functions while a pointer
  // to this surface is alive.
  UniqueCairoSurfacePtr GetCairoSurface();

 private:
  int width_;
  int height_;
  int stride_;
  std::vector<uint32> pixels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_IMAGE_H_
