/*
 * Copyright 2017 The Cartographer Authors
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

// The only cairo image format we use for Cartographer.
constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

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
  explicit Image(UniqueCairoSurfacePtr surface);
  Image(int width, int height);

  const Uint8Color GetPixel(int x, int y) const;
  void SetPixel(int x, int y, const Uint8Color& color);
  void WritePng(FileWriter* const file_writer);

  // Rotates the image in place.
  void Rotate90DegreesClockwise();

  // Returns a pointer to a cairo surface that contains the current pixel data.
  // The 'Image' object must therefore outlive the returned surface object. It
  // is undefined behavior to call any of the mutating functions while a pointer
  // to this surface is alive.
  UniqueCairoSurfacePtr GetCairoSurface();

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  int width_;
  int height_;
  std::vector<uint32> pixels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_IMAGE_H_
