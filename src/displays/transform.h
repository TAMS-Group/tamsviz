// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "marker.h"

struct TransformationsDisplay : MeshDisplayBase {
  std::vector<Frame> _frames;
  std::shared_ptr<InstancedMeshRenderer> _mesh_renderer;
  PROPERTY(double, radius, 0.01, min = 0.0);
  PROPERTY(double, length, 0.2, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  Watcher _watcher;
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(TransformationsDisplay, MeshDisplayBase);
