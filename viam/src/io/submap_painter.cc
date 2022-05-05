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

#include "viam/src/io/submap_painter.h"

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"

namespace viam {
namespace io {
namespace {

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

void CairoPaintSubmapSlices(
    const double scale,
    const std::map<::cartographer::mapping::SubmapId, 
          cartographer::io::SubmapSlice>& submaps,
    cairo_t* cr, std::function<void(const 
          cartographer::io::SubmapSlice&)> draw_callback) {
  cairo_scale(cr, scale, scale);

  for (auto& pair : submaps) {
    const auto& submap_slice = pair.second;
    if (submap_slice.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_slice.pose * submap_slice.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_slice.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);

    // Invokes caller's callback to utilize slice data in global cooridnate
    // frame. e.g. finds bounding box, paints slices.
    draw_callback(submap_slice);
    cairo_restore(cr);
  }
}

bool Has2DGrid(const cartographer::mapping::proto::Submap& submap) {
  return submap.has_submap_2d() && submap.submap_2d().has_grid();
}

bool Has3DGrids(const cartographer::mapping::proto::Submap& submap) {
  return submap.has_submap_3d() &&
         submap.submap_3d().has_low_resolution_hybrid_grid() &&
         submap.submap_3d().has_high_resolution_hybrid_grid();
}

}  // namespace

cartographer::io::PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId,
          cartographer::io::SubmapSlice>& submaps,
    const double resolution) {
  Eigen::AlignedBox2f bounding_box;
  {
    auto surface = cartographer::io::MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(cartographer::io::kCairoFormat, 1, 1));
    auto cr = cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoPaintSubmapSlices(
        1. / resolution, submaps, cr.get(),
        [&update_bounding_box](const cartographer::io::SubmapSlice& submap_slice) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_slice.width, 0);
          update_bounding_box(0, submap_slice.height);
          update_bounding_box(submap_slice.width, submap_slice.height);
        });
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  auto surface = cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(cartographer::io::kCairoFormat, size.x(), size.y()));
  {
    auto cr = cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.5, 0.5, 0.8);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), origin.x(), origin.y());
    CairoPaintSubmapSlices(1. / resolution, submaps, cr.get(),
                           [&cr](const cartographer::io::SubmapSlice& submap_slice) {
                             cairo_set_source_surface(
                                 cr.get(), submap_slice.surface.get(), 0., 0.);
                             cairo_paint(cr.get());
                           });
    cairo_surface_flush(surface.get());
  }
  return cartographer::io::PaintSubmapSlicesResult(std::move(surface), origin);
}

}  // namespace io
}  // namespace viam
