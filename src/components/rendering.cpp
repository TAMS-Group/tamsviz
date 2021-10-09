// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "rendering.h"

#include "../render/renderer.h"

void RenderingComponent::renderSync(const RenderSyncContext &context) {
  Component::renderSync(context);

  RenderParameters &params = context.render_list->parameters();
  params.shadow_map_resolution = shadowMapResolution();
  params.shadow_cube_resolution = shadowCubeResolution();
  params.exposure = exposure();
  params.tone_mapping = toneMapping();
  params.black_level = blackLevel();
  params.white_level = whiteLevel();
}
