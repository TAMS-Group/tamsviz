// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "marker.h"
#include "text.h"

struct TransformationsDisplay : MeshDisplayBase {
  std::vector<Frame> _frames;

  Watcher _axis_watcher;
  std::shared_ptr<InstancedMeshRenderer> _axis_renderer;
  PROPERTY(bool, showAxes, true);
  PROPERTY(bool, crossAxes, false);
  PROPERTY(double, radius, 0.01, min = 0.0);
  PROPERTY(double, length, 0.2, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));

  std::vector<std::shared_ptr<TextRenderer>> _text_renderers;
  std::shared_ptr<Material> _text_material = std::make_shared<Material>();
  PROPERTY(bool, showNames, true);
  PROPERTY(Color3, labelColor, Color3(1, 1, 1));
  PROPERTY(double, labelOpacity, 1.0, min = 0.0, max = 1.0);
  PROPERTY(double, labelSize, 0.1, min = 0);
  PROPERTY(Eigen::Vector2d, labelOffset, Eigen::Vector2d::Zero());

  Watcher _sphere_watcher;
  std::shared_ptr<InstancedMeshRenderer> _sphere_renderer;
  PROPERTY(bool, showSpheres, true);
  PROPERTY(double, sphereRadius, 0.02, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, sphereMaterial,
           std::make_shared<Material>(1, 1, 0));

  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE_C(TransformationsDisplay, MeshDisplayBase, Transform);
