// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "transform.h"

#include "axes.h"
#include "shapes.h"

#include "../core/transformer.h"
#include "../core/workspace.h"

void TransformationsDisplay::renderSync(const RenderSyncContext &c) {
  RenderSyncContext context = c;
  context.pose.setIdentity();
  if (_watcher.changed(length(), radius())) {
    _mesh_renderer = std::make_shared<MeshRenderer>(
        this, std::make_shared<Mesh>(makeAxes(length(), radius(), 24)),
        material());
  }
  auto &transformer = LockScope()->document()->display()->transformer;
  _mesh_renderer->clearInstances();
  for (auto &name : transformer->list()) {
    if (auto frame = transformer->lookup(name)) {
      _mesh_renderer->addInstance(*frame);
    }
  }
  MeshDisplayBase::renderSync(context);
}
