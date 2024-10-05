// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "transform.h"

#include "../core/transformer.h"
#include "../core/workspace.h"
#include "axes.h"
#include "shapes.h"

void TransformationsDisplay::renderSync(const RenderSyncContext &context) {
  if (_axis_watcher.changed(length(), radius(), crossAxes())) {
    _axis_renderer = node()->create<InstancedMeshRenderer>(
        std::make_shared<Mesh>(crossAxes()
                                   ? makeCrossedAxes(length(), radius(), 24)
                                   : makeAxes(length(), radius(), 24)),
        material());
  }
  if (_sphere_watcher.changed(sphereRadius())) {
    _sphere_renderer = node()->create<InstancedMeshRenderer>(
        std::make_shared<Mesh>(makeSphere().scale(sphereRadius())),
        sphereMaterial());
  }
  auto &transformer = LockScope()->document()->display()->transformer;
  auto frames = transformer->list();
  size_t n_text = 0;
  _axis_renderer->clearInstances();
  _sphere_renderer->clearInstances();
  _frames.resize(frames.size());
  for (size_t i = 0; i < frames.size(); i++) {
    _frames[i].name(frames[i]);
    if (auto frame = _frames[i].pose(transformer)) {
      if (showAxes()) _axis_renderer->addInstance(*frame);
      if (showSpheres()) _sphere_renderer->addInstance(*frame);
      if (showNames()) {
        if (_text_renderers.size() <= n_text) {
          _text_renderers.push_back(
              node()->create<TextRenderer>(_text_material));
        }
        auto &text = *_text_renderers.at(n_text);
        text.text(frames[i]);
        text.pose(*frame);
        text.offset(labelOffset());
        text.viewFacing(true);
        text.size(labelSize());
        n_text++;
      }
    }
  }
  _text_renderers.resize(n_text);
  _text_material->color().r() = labelColor().r();
  _text_material->color().g() = labelColor().g();
  _text_material->color().b() = labelColor().b();
  _text_material->opacity() = labelOpacity();
  MeshDisplayBase::renderSync(context);
}
