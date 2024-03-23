// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "framebuffer.h"
#include "renderlist.h"
#include "texture.h"
#include "uniformbuffer.h"

class RenderTarget;
class Shader;

class Renderer {
  std::shared_ptr<Shader> default_shader, blend_shader;
  std::vector<uint8_t> aligned_material_buffer;
  UniformBuffer<CameraBlock> camera_uniform_buffer;
  UniformBuffer<MaterialBlock> material_buffer;
  UniformBuffer<LightArrayBlock> light_buffer;
  std::vector<RenderCommand> _transparent, _opaque;
  int buffer_alignment = -1;
  Renderer(const Renderer &) = delete;
  Renderer &operator=(const Renderer &) = delete;
  void render(const RenderList &render_list,
              const std::vector<RenderCommand> &commands, bool picking = false);
  std::shared_ptr<Mesh> screen_quad;
  void prepare(const CameraBlock &camera_block, const RenderList &render_list);
  Texture _shadow_map_array;
  Watcher _shadow_map_watcher;
  Texture _shadow_cube_array;
  Watcher _shadow_cube_watcher;
  Framebuffer _shadow_framebuffer;
  void _splitTransparentOpaque(const RenderList &render_list);

public:
  Renderer();
  void renderShadows(const RenderList &render_list);
  void render(RenderTarget &render_target, const CameraBlock &camera_block,
              const RenderList &render_list);
  struct PickResult {
    uint32_t id = 0;
    double depth = 0;
  };
  PickResult pick(RenderTarget &render_target, const CameraBlock &camera_block,
                  const RenderList &render_list, int x, int y);
};
