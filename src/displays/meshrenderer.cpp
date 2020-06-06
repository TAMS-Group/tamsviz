// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "meshrenderer.h"

#include "mesh.h"

MeshRenderer::MeshRenderer(MeshDisplayBase *object,
                           const std::shared_ptr<Mesh> &mesh,
                           const std::shared_ptr<MaterialRenderer> &material)
    : _render_set(object->_render_set) {
  if (!_render_set) {
    throw std::invalid_argument("object");
  }
  _data->_mesh = mesh;
  _data->_material = material;
  _data->_render_options = std::make_shared<RenderOptions>();
  _render_set->insert(_data);
}

MeshRenderer::MeshRenderer(
    MeshDisplayBase *object, const std::shared_ptr<Mesh> &mesh,
    std::shared_ptr<const Material> material,
    std::shared_ptr<const MaterialOverride> material_override)
    : MeshRenderer(
          object, mesh,
          std::make_shared<MaterialRenderer>(material, material_override)) {}

MeshRenderer::~MeshRenderer() {
  _render_set->erase(_data);
  _render_set = nullptr;
  _data = nullptr;
}

void MeshRenderer::Data::renderSync(const RenderSyncContext &context) {
  _parent_pose = context.pose;
  _material->renderSync(context);
}

void MeshRenderer::Data::renderAsync(const RenderAsyncContext &context) {
  _material->renderAsync(context);
  if (_visible && !_poses.empty()) {
    context.render_list->push(_material->_block);
    context.render_list->push(_mesh, *_render_options);
    for (auto &pose : _poses) {
      InstanceBlock instance;
      instance.setPose((_parent_pose * pose).matrix().cast<float>());
      context.render_list->push(instance);
    }
  }
}

void MeshRenderer::options(const RenderOptions &options) {
  *(_data->_render_options) = options;
}
