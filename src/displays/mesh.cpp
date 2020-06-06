// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "mesh.h"

#include "material.h"

#include "../core/transformer.h"
#include "../core/workspace.h"

void MeshDisplayBase::renderSync(const RenderSyncContext &context) {
  {
    _current_mesh_renderers.clear();
    std::lock_guard<std::mutex> lock(_render_set->_mutex);
    for (auto &x : _render_set->_mesh_renderers) {
      _current_mesh_renderers.push_back(x);
    }
  }
  for (auto &x : _current_mesh_renderers) {
    x->renderSync(context);
  }
}

void MeshDisplayBase::renderAsync(const RenderAsyncContext &context) {
  for (auto &x : _current_mesh_renderers) {
    x->renderAsync(context);
  }
}
