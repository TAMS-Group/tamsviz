// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "material.h"

#include "../core/log.h"
#include "mesh.h"

Material::Material() { _alive = true; }

Material::Material(float r, float g, float b, float a) {
  _alive = true;
  color().r() = r;
  color().g() = g;
  color().b() = b;
  opacity() = a;
}

Material::~Material() { _alive = false; }

MaterialOverride::MaterialOverride() { _alive = true; }

MaterialOverride::~MaterialOverride() { _alive = false; }

static void syncTexture(std::shared_ptr<Texture> &texture,
                        const std::string &url, TextureType type) {
  if (!url.empty()) {
    if (!texture || texture->url() != url) {
      texture = Texture::manager().load(url, type);
    }
  } else {
    texture = nullptr;
  }
}

struct MaterialIDFactory {
  std::mutex _mutex;
  size_t _counter = 1;
  std::vector<uint32_t> _free;

public:
  uint32_t allocate() {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_free.empty()) {
      return _counter++;
    } else {
      auto id = _free.back();
      _free.pop_back();
      return id;
    }
  }
  void release(uint32_t id) {
    std::lock_guard<std::mutex> lock(_mutex);
    _free.push_back(id);
  }
  static std::shared_ptr<MaterialIDFactory> instance() {
    static auto instance = std::make_shared<MaterialIDFactory>();
    return instance;
  }
};

MaterialRenderer::MaterialRenderer(
    std::shared_ptr<const Material> material,
    std::shared_ptr<const MaterialOverride> material_override) {
  if (!material) {
    throw std::invalid_argument("material");
  }
  _material = material;
  _material_override = material_override;
  _block.id = MaterialIDFactory::instance()->allocate();
}

MaterialRenderer::~MaterialRenderer() {
  MaterialIDFactory::instance()->release(_block.id);
}

void MaterialRenderer::updateSync(const Material &material) {
  if (!material._alive) {
    throw std::runtime_error("material already destroyed");
  }
  _block.color = material.color().toLinearVector4f();
  _block.color.w() = (float)material.opacity();
  _block.roughness = (float)material.roughness();
  _block.metallic = (float)material.metallic();
  _color_texture_url = material.texture();
  _normal_texture_url = material.normals();
  if (!material._alive) {
    throw std::runtime_error("material already destroyed");
  }
}

void MaterialRenderer::updateSync(const MaterialOverride &material_override) {
  if (!material_override._alive) {
    throw std::runtime_error("material override already destroyed");
  }
  if (material_override.color()) {
    _block.color = material_override.material()->color().toLinearVector4f();
  }
  if (material_override.parameters()) {
    _block.roughness = (float)material_override.material()->roughness();
    _block.metallic = (float)material_override.material()->metallic();
  }
  _block.color.w() = (float)material_override.material()->opacity();
  if (material_override.texture()) {
    _color_texture_url = material_override.material()->texture();
  }
  if (material_override.normals()) {
    _normal_texture_url = material_override.material()->normals();
  }
  if (!material_override._alive) {
    throw std::runtime_error("material override already destroyed");
  }
}

void MaterialRenderer::renderAsync(const RenderAsyncContext &context) {
  syncTexture(_color_texture, _color_texture_url, TextureType::Color);
  syncTexture(_normal_texture, _normal_texture_url, TextureType::Normal);
  _block.color_texture = _color_texture ? _color_texture->update() : 0;
  _block.normal_texture = _normal_texture ? _normal_texture->update() : 0;
  _block.transparent = ((_block.color.w() < 1.0) ||
                        (_color_texture && _color_texture->transparent()));
}

void MaterialRenderer::renderSync(const RenderSyncContext &context) {
  if (_material) {
    updateSync(*_material);
    if (_material_override) {
      updateSync(*_material_override);
    }
  }
}
