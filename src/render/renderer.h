// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderlist.h"

#include "shader.h"
#include "uniformbuffer.h"

class Renderer {
  std::shared_ptr<Shader> default_shader;
  std::vector<uint8_t> aligned_material_buffer;
  UniformBuffer<CameraBlock> camera_uniform_buffer{
      (size_t)UniformBindingPoint::camera};
  UniformBuffer<MaterialBlock> material_buffer{
      (size_t)UniformBindingPoint::material};
  UniformBuffer<LightArrayBlock> light_buffer{
      (size_t)UniformBindingPoint::lights};
  int buffer_alignment = -1;
  Renderer(const Renderer &) = delete;
  Renderer &operator=(const Renderer &) = delete;

public:
  Renderer();
  void render(const CameraBlock &camera_block, const RenderList &render_list);
};
