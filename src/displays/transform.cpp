// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "transform.h"

#include "../core/transformer.h"
#include "../core/workspace.h"
#include "axes.h"
#include "shapes.h"

void TransformationsDisplay::renderSync(const RenderSyncContext &context) {
  if (_watcher.changed(length(), radius())) {
    _mesh_renderer = node()->create<InstancedMeshRenderer>(
        std::make_shared<Mesh>(makeAxes(length(), radius(), 24)), material());
  }
  auto &transformer = LockScope()->document()->display()->transformer;
  _mesh_renderer->clearInstances();
  auto frames = transformer->list();
  _frames.resize(frames.size());
  for (size_t i = 0; i < frames.size(); i++) {
    _frames[i].name(frames[i]);
    if (auto frame = _frames[i].pose(transformer)) {
      _mesh_renderer->addInstance(*frame);
    }
  }
  MeshDisplayBase::renderSync(context);
}
