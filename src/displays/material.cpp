// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "material.h"

#include "../core/log.h"

Material::Material() {
  // LOG_DEBUG("Material::Material");
  _alive = true;
}

Material::Material(float r, float g, float b, float a) {
  _alive = true;
  color().r() = r;
  color().g() = g;
  color().b() = b;
  opacity() = a;
}

Material::~Material() { // LOG_DEBUG("Material::~Material");
  _alive = false;
}

MaterialOverride::MaterialOverride() {
  // LOG_DEBUG("Material::MaterialOverride");
  _alive = true;
}

MaterialOverride::~MaterialOverride() {
  // LOG_DEBUG("Material::~MaterialOverride");
  _alive = false;
}
