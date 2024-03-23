// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "mesh.h"

#include "../core/interaction.h"
#include "material.h"
#include "mesh.h"

bool MeshRenderer::pick(uint32_t id) const {
  return _material && _material->id() == id;
}

bool MeshRenderer::interact(const Interaction &interaction) {
  if (_interact_callback && pick(interaction.id)) {
    _interact_callback(interaction);
    return true;
  }
  return false;
}

MeshRenderer::MeshRenderer(const std::shared_ptr<Mesh> &mesh,
                           const std::shared_ptr<MaterialRenderer> &material) {
  _mesh = mesh;
  _material = material;
}

MeshRenderer::MeshRenderer(
    const std::shared_ptr<Mesh> &mesh, std::shared_ptr<const Material> material,
    std::shared_ptr<const MaterialOverride> material_override)
    : MeshRenderer(mesh, std::make_shared<MaterialRenderer>(
                             material, material_override)) {}

MeshRenderer::~MeshRenderer() {}

void MeshRenderer::renderSync(const RenderSyncContext &context) {
  _parent_pose = context.pose;
  _material->renderSync(context);
  _render_options_2 = _render_options;
}

void MeshRenderer::renderAsync(const RenderAsyncContext &context) {
  _material->renderAsync(context);
  if (visible()) {
    context.render_list->push(_material->block());
    context.render_list->push(_mesh, _render_options_2);
    InstanceBlock instance;
    instance.setPose(_parent_pose.matrix().cast<float>());
    context.render_list->push(instance);
  }
}

void MeshRenderer::options(const RenderOptions &options) {
  _render_options = options;
}

MeshRenderer::MeshRenderer(const std::shared_ptr<Mesh> &mesh,
                           std::shared_ptr<const Material> material,
                           const Eigen::Isometry3d &pose)
    : MeshRenderer(mesh, material) {
  this->pose(pose);
}

MeshRenderer::MeshRenderer(
    const std::shared_ptr<Mesh> &mesh, std::shared_ptr<const Material> material,
    const std::function<void(const Interaction &)> &interact_callback)
    : MeshRenderer(mesh, material) {
  _interact_callback = interact_callback;
}

const std::shared_ptr<Mesh> &MeshRenderer::mesh() const { return _mesh; }

const std::shared_ptr<MaterialRenderer> &MeshRenderer::materialRenderer() {
  return _material;
}

const RenderOptions &MeshRenderer::options() const { return _render_options; }

RenderOptions &MeshRenderer::options() { return _render_options; }

void InstancedMeshRenderer::renderAsync(const RenderAsyncContext &context) {
  _material->renderAsync(context);
  if (visible()) {
    context.render_list->push(_material->block());
    context.render_list->push(_mesh, _render_options_2);
    for (auto &pose : _poses) {
      InstanceBlock instance;
      instance.setPose(pose.matrix().cast<float>());
      context.render_list->push(instance);
    }
  }
}

InstancedMeshRenderer::InstancedMeshRenderer(
    const std::shared_ptr<Mesh> &mesh, std::shared_ptr<const Material> material)
    : MeshRenderer(mesh, material) {}

void InstancedMeshRenderer::clearInstances() { _poses.clear(); }

void InstancedMeshRenderer::addInstance(const Eigen::Isometry3d &pose) {
  _poses.push_back(pose);
}
