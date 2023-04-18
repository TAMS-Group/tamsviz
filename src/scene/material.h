// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"

#include "../render/renderlist.h"
#include "../render/texture.h"

struct Material : Object {
  bool _alive = false;
  PROPERTY(Color3, color, Color3());
  PROPERTY(double, opacity, 1.0, min = 0, max = 1);
  PROPERTY(double, roughness, 0.4, min = 0, max = 1);
  PROPERTY(double, metallic, 0.0, min = 0, max = 1);
  PROPERTY(bool, unlit, false);
  PROPERTY(std::string, texture, "");
  PROPERTY(std::string, normals, "");
  Material();
  Material(float r, float g, float b, float a = 1);
  ~Material();
};
DECLARE_TYPE(Material, Object);

struct MaterialOverride : Object {
  bool _alive = false;
  PROPERTY(bool, color, false);
  PROPERTY(bool, parameters, false);
  PROPERTY(bool, texture, false);
  PROPERTY(bool, normals, false);
  PROPERTY(std::shared_ptr<Material>, material, std::make_shared<Material>());
  MaterialOverride();
  ~MaterialOverride();
};
DECLARE_TYPE(MaterialOverride, Object);

class MaterialRenderer {
private:
  std::shared_ptr<const Material> _material;
  std::shared_ptr<const MaterialOverride> _material_override;
  MaterialBlock _block;
  std::string _color_texture_url, _normal_texture_url;
  std::shared_ptr<Texture> _color_texture, _normal_texture;
  void updateSync(const Material &material);
  void updateSync(const MaterialOverride &material_override);

public:
  MaterialRenderer(const MaterialRenderer &) = delete;
  MaterialRenderer &operator=(const MaterialRenderer &) = delete;
  MaterialRenderer(
      std::shared_ptr<const Material> material,
      std::shared_ptr<const MaterialOverride> material_override = nullptr);
  ~MaterialRenderer();
  void renderSync(const RenderSyncContext &context);
  void renderAsync(const RenderAsyncContext &context);
  const MaterialBlock &block() const { return _block; }
  MaterialBlock &block() { return _block; }
  bool pick(uint32_t id) const { return _block.id == id; }
  uint32_t id() const { return _block.id; }
};
