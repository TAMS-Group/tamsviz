// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "renderlist.h"

#include "../core/log.h"
#include "mesh.h"

void RenderList::push(const MaterialBlock &material) {
  _materials.push_back(material);
}

void RenderList::push(const std::shared_ptr<Mesh> &mesh,
                      const RenderOptions &options) {
  RenderCommand command;
  command.options = options;
  if (mesh->transparent()) {
    command.options.transparent = true;
  }
  command.vertex_array_object = mesh->vertexArrayObject();
  if (!mesh->data().indices.empty()) {
    command.element_count = mesh->data().indices.size();
    command.indexed = true;
  } else {
    command.element_count = mesh->data().positions.size();
    command.indexed = false;
  }
  command.material_index = _materials.size() - 1;
  _commands.push_back(command);
}

void RenderList::push(const InstanceBlock &instance) {
  auto &command = _commands.back();
  if (command.instance_count == 0) {
    command.first_instance = _instances.size();
    command.instance_count = 1;
  } else {
    command.instance_count++;
  }
  _instances.push_back(instance);
}

void RenderList::push(const LightBlock &light) {
  auto l = light;
  switch (l.type) {
  case uint32_t(LightType::PointShadow):
    l.shadow_index = (_shadow_cube_count++);
    break;
  case uint32_t(LightType::SpotShadow):
  case uint32_t(LightType::DirectionalShadow):
    l.shadow_index = (_shadow_map_count++);
    break;
  }
  _lights.push_back(l);
}

void RenderList::clear() {
  _shadow_map_count = 0;
  _shadow_cube_count = 0;
  _materials.clear();
  _instances.clear();
  _commands.clear();
  _lights.clear();
}
