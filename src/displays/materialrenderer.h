// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../render/renderer.h"
#include "../render/texture.h"

class MeshDisplayBase;
class RenderSet;
class Material;
class MaterialOverride;

class MaterialRenderer {
private:
  std::shared_ptr<const Material> _material;
  std::shared_ptr<const MaterialOverride> _material_override;
  MaterialBlock _block;
  std::string _color_texture_url, _normal_texture_url;
  std::shared_ptr<Texture> _color_texture, _normal_texture;
  void updateSync(const Material &material);
  void updateSync(const MaterialOverride &material_override);
  void renderSync(const RenderSyncContext &context);
  void renderAsync(const RenderAsyncContext &context);

public:
  MaterialRenderer(const MaterialRenderer &) = delete;
  MaterialRenderer &operator=(const MaterialRenderer &) = delete;
  MaterialRenderer(
      std::shared_ptr<const Material> material,
      std::shared_ptr<const MaterialOverride> material_override = nullptr);
  ~MaterialRenderer();
  friend class MeshDisplayBase;
  friend class MeshRenderer;
  friend class RenderSet;
};
