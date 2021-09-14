// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "frame.h"
#include "mesh.h"

class Material;
class MaterialRenderer;
class Texture;
class RenderOptions;
class Mesh;

class TextRenderer : public SceneNode {
  std::shared_ptr<Material> _material;
  std::shared_ptr<MaterialRenderer> _material_renderer;
  std::shared_ptr<Texture> _texture;
  std::shared_ptr<Mesh> _mesh;
  std::string _text;
  Watcher _watcher;
  Eigen::Affine3d _parent_pose = Eigen::Affine3d::Identity();
  bool _view_facing = true;
  double _size = 1.0;
  Eigen::Vector2d _offset = Eigen::Vector2d::Zero();

public:
  void viewFacing(bool v) { _view_facing = v; }
  bool viewFacing() const { return _view_facing; }
  const std::string &text() const { return _text; }
  void text(const std::string &text) { _text = text; }
  TextRenderer(const std::shared_ptr<Material> &material = nullptr);
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  virtual bool pick(uint32_t id) const override;
  void size(double s) { _size = s; }
  double size() const { return _size; }
  void offset(const Eigen::Vector2d &offset) { _offset = offset; }
  const Eigen::Vector2d &offset() const { return _offset; }
};

class TextDisplay : public FrameDisplayBase {
  std::shared_ptr<TextRenderer> _renderer;
  std::shared_ptr<Material> _material = std::make_shared<Material>();

public:
  PROPERTY(std::string, text, "Text");
  PROPERTY(Color3, color, Color3(1, 1, 1));
  PROPERTY(double, brightness, 1.0, min = 0.0);
  PROPERTY(double, opacity, 1.0, min = 0.0, max = 1.0);
  PROPERTY(double, size, 0.1, min = 0);
  PROPERTY(bool, viewFacing, true);
  PROPERTY(Eigen::Vector2d, offset, Eigen::Vector2d::Zero());
  TextDisplay();
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE(TextDisplay, MeshDisplayBase);
