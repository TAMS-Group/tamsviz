// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "mesh.h"

void MeshDisplayBase::renderSync(const RenderSyncContext &context) {
  _context.nodes.clear();
  _node->renderSyncRecursive(context, _context);
}

void MeshDisplayBase::renderAsync(const RenderAsyncContext &context) {
  for (auto &node : _context.nodes) {
    node->renderAsync(context);
  }
}

bool MeshDisplayBase::pick(uint32_t id) const { return _node->pick(id); }

bool MeshDisplayBase::interact(const Interaction &interaction) {
  return _node->interact(interaction);
}
