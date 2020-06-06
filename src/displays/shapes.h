// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../render/renderer.h"

#include "frame.h"
#include "material.h"
#include "mesh.h"

namespace shapes {
class Shape;
}

class MeshData;

MeshData makePlane();
MeshData makeBox();
MeshData makeSphere(size_t segments, size_t rings);
MeshData makeDisk(size_t segments);
MeshData makeCylinder(size_t segments);
MeshData makeCone(size_t segments);

struct ShapeDisplay : FrameDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  Watcher _watcher;
  PROPERTY(std::shared_ptr<Material>, material, std::make_shared<Material>());
  ShapeDisplay(const std::shared_ptr<Mesh> &mesh = nullptr);
};
DECLARE_TYPE(ShapeDisplay, FrameDisplayBase);
