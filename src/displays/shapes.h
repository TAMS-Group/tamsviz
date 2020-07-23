// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/watcher.h"
#include "../render/renderer.h"
#include "../scene/material.h"
#include "../scene/mesh.h"

#include "frame.h"

namespace shapes {
class Shape;
}

class MeshData;

MeshData makePlane();
MeshData makeBox();
MeshData makeSphere(size_t segments = 32, size_t rings = 16);
MeshData makeDisk(size_t segments = 32);
MeshData makeCylinder(size_t segments = 32);
MeshData makeCone(size_t segments = 32);
MeshData makeRing(double inner, double outer = 1.0, size_t segments = 32);

struct ShapeDisplay : FrameDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  Watcher _watcher;
  PROPERTY(std::shared_ptr<Material>, material, std::make_shared<Material>());

protected:
  ShapeDisplay(const std::shared_ptr<Mesh> &mesh = nullptr);
};
DECLARE_TYPE(ShapeDisplay, FrameDisplayBase);
