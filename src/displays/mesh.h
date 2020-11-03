// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"

#include "../scene/material.h"
#include "../scene/mesh.h"
#include "../scene/node.h"

class MeshDisplayBase : public Display {
  std::shared_ptr<SceneNode> _node = std::make_shared<SceneNode>();
  SceneContext _context;

protected:
  MeshDisplayBase() {}

public:
  PROPERTY(bool, visible, true);
  // PROPERTY(bool, interactive, true);
  const std::shared_ptr<SceneNode> &node() { return _node; }
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  virtual bool pick(uint32_t id) const override;
  virtual bool interact(const Interaction &interaction) override;
};
DECLARE_TYPE(MeshDisplayBase, Display);
