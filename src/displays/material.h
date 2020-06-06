// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../render/renderer.h"
#include "../render/texture.h"

struct Material : Object {
  bool _alive = false;
  PROPERTY(Color3, color, Color3());
  PROPERTY(double, opacity, 1.0, min = 0, max = 1);
  PROPERTY(double, roughness, 0.4, min = 0, max = 1);
  PROPERTY(double, metallic, 0.0, min = 0, max = 1);
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
