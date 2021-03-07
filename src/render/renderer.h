// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderlist.h"
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

public:
  Renderer();
  void render(RenderTarget &render_target, const CameraBlock &camera_block,
              const RenderList &render_list);
  struct PickResult {
    uint32_t id = 0;
    double depth = 0;
  };
  PickResult pick(RenderTarget &render_target, const CameraBlock &camera_block,
                  const RenderList &render_list, int x, int y);
};
