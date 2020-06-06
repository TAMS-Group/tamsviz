// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"

#include "materialrenderer.h"
#include "meshrenderer.h"

#include <unordered_set>

class RenderSet {
private:
  std::mutex _mutex;
  std::unordered_set<std::shared_ptr<MeshRenderer::Data>> _mesh_renderers;

public:
  void insert(const std::shared_ptr<MeshRenderer::Data> &renderer) {
    std::lock_guard<std::mutex> lock(_mutex);
    _mesh_renderers.insert(renderer);
  }
  void erase(const std::shared_ptr<MeshRenderer::Data> &renderer) {
    std::lock_guard<std::mutex> lock(_mutex);
    _mesh_renderers.erase(renderer);
  }
  friend class MeshDisplayBase;
};

class MeshDisplayBase : public Display {
  std::shared_ptr<RenderSet> _render_set = std::make_shared<RenderSet>();
  std::vector<std::shared_ptr<MeshRenderer::Data>> _current_mesh_renderers;

protected:
  MeshDisplayBase() {}

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  friend class MaterialRenderer;
  friend class MeshRenderer;
};
DECLARE_TYPE(MeshDisplayBase, Display);
