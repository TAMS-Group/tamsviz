// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "mesh.h"

void MeshDisplayBase::renderSync(const RenderSyncContext &context) {
  _context.nodes.clear();
  if (visible()) {
    _node->renderSyncRecursive(context, _context);
  }
  Display::renderSync(context);
}

void MeshDisplayBase::renderAsync(const RenderAsyncContext &context) {
  for (auto &node : _context.nodes) {
    node->renderAsync(context);
  }
  Display::renderAsync(context);
}

bool MeshDisplayBase::pick(uint32_t id) const {
  // if (!visible() || !interactive()) {
  //   return false;
  // }
  return _node->pick(id);
}

bool MeshDisplayBase::interact(const Interaction &interaction) {
  // if (!visible() || !interactive()) {
  //   return false;
  // }
  return _node->interact(interaction);
}
