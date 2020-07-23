// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "axes.h"

#include "shapes.h"

MeshData makeAxes(double length, double radius, size_t segments) {
  MeshData mesh;
  mesh += makeCylinder(segments)
              .colorize(1, 0, 0)
              .scale(radius, radius, length * 0.5)
              .translate(0, 0, length * 0.5)
              .rotate(M_PI * 0.5, Eigen::Vector3f::UnitY());
  mesh += makeCylinder(segments)
              .colorize(0, 1, 0)
              .scale(radius, radius, length * 0.5)
              .translate(0, 0, length * 0.5)
              .rotate(M_PI * -0.5, Eigen::Vector3f::UnitX());
  mesh += makeCylinder(segments)
              .colorize(0, 0, 1)
              .scale(radius, radius, length * 0.5)
              .translate(0, 0, length * 0.5);
  return mesh;
}

void AxesDisplay::renderSync(const RenderSyncContext &context) {
  if (_watcher.changed(length(), radius())) {
    _mesh_renderer = node()->create<MeshRenderer>(
        std::make_shared<Mesh>(makeAxes(length(), radius(), 32)), material());
    _mesh_renderer->pose(Eigen::Isometry3d::Identity());
  }
  FrameDisplayBase::renderSync(context);
}
