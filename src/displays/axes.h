// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "marker.h"

MeshData makeAxes(double radius, double length, size_t segments);

struct AxesDisplay : FrameDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  PROPERTY(double, radius, 0.05, min = 0.0);
  PROPERTY(double, length, 1.0, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  Watcher _watcher;
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(AxesDisplay, FrameDisplayBase);
