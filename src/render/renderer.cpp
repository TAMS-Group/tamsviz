// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderer.h"

#include "../core/log.h"
#include "mesh.h"
#include "renderlist.h"
#include "rendertarget.h"
#include "shader.h"
#include "texture.h"
#include "uniformbuffer.h"

Renderer::Renderer()
    : camera_uniform_buffer((size_t)UniformBindingPoint::camera),
      material_buffer((size_t)UniformBindingPoint::material),
      light_buffer((size_t)UniformBindingPoint::lights) {

  {
    MeshData mesh_data;

    mesh_data.indices.emplace_back(0);
    mesh_data.indices.emplace_back(2);
    mesh_data.indices.emplace_back(1);
    mesh_data.indices.emplace_back(0);
    mesh_data.indices.emplace_back(3);
    mesh_data.indices.emplace_back(2);

    mesh_data.positions.emplace_back(-1, -1, 0);
    mesh_data.positions.emplace_back(-1, 1, 0);
    mesh_data.positions.emplace_back(1, 1, 0);
    mesh_data.positions.emplace_back(1, -1, 0);

    mesh_data.texcoords.emplace_back(0, 0);
    mesh_data.texcoords.emplace_back(0, 1);
    mesh_data.texcoords.emplace_back(1, 1);
    mesh_data.texcoords.emplace_back(1, 0);

    screen_quad = std::make_shared<Mesh>(mesh_data);
  }

  default_shader = std::make_shared<Shader>(
      "package://" ROS_PACKAGE_NAME "/shaders/default.vert",
      "package://" ROS_PACKAGE_NAME "/shaders/default.frag");

  blend_shader = std::make_shared<Shader>(
      "package://" ROS_PACKAGE_NAME "/shaders/quad.vert",
      "package://" ROS_PACKAGE_NAME "/shaders/blend.frag");
}

void Renderer::render(const RenderList &render_list,
                      const std::vector<RenderCommand> &commands) {

  int previous_double_sided = -1;

  for (auto &command : commands) {

    auto &material = render_list._materials[command.material_index];

    int double_sided = (command.options.double_sided ? 1 : 0);
    if (double_sided != previous_double_sided) {
      if (double_sided) {
        V_GL(glDisable(GL_CULL_FACE));
      } else {
        V_GL(glEnable(GL_CULL_FACE));
      }
      previous_double_sided = double_sided;
    }

    if (command.material_index < 0 ||
        command.material_index >= render_list._materials.size()) {
      throw std::runtime_error("material index out of range");
    }

    material_buffer.update(material);
    material_buffer.bind();

    V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::color));
    V_GL(glBindTexture(GL_TEXTURE_2D, material.color_texture));

    V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::normal));
    V_GL(glBindTexture(GL_TEXTURE_2D, material.normal_texture));

    V_GL(glBindVertexArray(command.vertex_array_object));

    for (size_t instance_index = command.first_instance;
         instance_index < command.first_instance + command.instance_count;
         instance_index++) {
      auto &instance = render_list._instances[instance_index];

      V_GL(glVertexAttrib4fv((GLuint)VertexAttributes::pose_x,
                             instance.pose_x.data()));
      V_GL(glVertexAttrib4fv((GLuint)VertexAttributes::pose_y,
                             instance.pose_y.data()));
      V_GL(glVertexAttrib4fv((GLuint)VertexAttributes::pose_z,
                             instance.pose_z.data()));

      if (command.indexed) {
        V_GL(glDrawElements(command.options.primitive_type,
                            command.element_count, GL_UNSIGNED_INT, nullptr));
      } else {
        V_GL(glDrawArrays(command.options.primitive_type, 0,
                          command.element_count));
      }
    }
  }
}

void Renderer::prepare(const CameraBlock &camera_block,
                       const RenderList &render_list) {

  V_GL(glEnable(GL_FRAMEBUFFER_SRGB));
  V_GL(glEnable(GL_DEPTH_TEST));
  V_GL(glEnable(GL_CULL_FACE));
  V_GL(glCullFace(GL_BACK));
  V_GL(glDisable(GL_BLEND));
  V_GL(glDepthMask(GL_TRUE));

  glEnable(GL_MULTISAMPLE);

  camera_uniform_buffer.update(camera_block);
  camera_uniform_buffer.bind();

  Eigen::Matrix4f view_to_world = camera_block.view_matrix.inverse();

  LightArrayBlock light_array;
  light_array.light_count = std::min(sizeof(light_array.light_array) /
                                         sizeof(light_array.light_array[0]),
                                     render_list._lights.size());
  for (size_t light_index = 0; light_index < light_array.light_count;
       light_index++) {

    auto &light = light_array.light_array[light_index];

    light_array.light_array[light_index] = render_list._lights[light_index];

    if (light.type & uint32_t(LightType::ViewSpace)) {
      Eigen::Vector4f p;
      p.head(3) = light_array.light_array[light_index].position;
      p.w() = 1.0f;
      p = view_to_world * p;
      p /= p.w();
      light_array.light_array[light_index].position = p.head(3);

      light_array.light_array[light_index].pose =
          light_array.light_array[light_index].pose * camera_block.view_matrix;
    }

    light.type = (light.type & 0xff);
  }
  light_buffer.update(light_array);
  light_buffer.bind();
}

Renderer::PickResult Renderer::pick(RenderTarget &render_target,
                                    const CameraBlock &camera_block,
                                    const RenderList &render_list, int x,
                                    int y) {

  if (x < 0 || y < 0 || x >= render_target._width ||
      y >= render_target._height) {
    return PickResult();
  }

  V_GL(glDisable(GL_FRAMEBUFFER_SRGB));
  V_GL(glEnable(GL_DEPTH_TEST));
  V_GL(glEnable(GL_CULL_FACE));
  V_GL(glCullFace(GL_BACK));
  V_GL(glDisable(GL_BLEND));

  prepare(camera_block, render_list);

  default_shader->use();
  render_target._pick_framebuffer.bind();

  V_GL(glClearColor(0, 0, 0, 0));
  V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

  render(render_list, render_list._commands);

  PickResult ret;

  {
    std::array<uint32_t, 4> id;
    id[0] = 0;
    V_GL(glReadPixels(x, y, 1, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, id.data()));
    ret.id = id[0];
  }

  {
    float depth = 0.0f;
    V_GL(glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth));
    ret.depth = depth;
  }

  LOG_DEBUG("pick " << x << " " << y << " " << ret.id << " " << ret.depth);

  V_GL(glUseProgram(0));
  V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

  return ret;
}

void Renderer::render(RenderTarget &render_target,
                      const CameraBlock &camera_block,
                      const RenderList &render_list) {

  prepare(camera_block, render_list);

  default_shader->use();

  _transparent.clear();
  _opaque.clear();
  for (auto &command : render_list._commands) {
    if (command.element_count > 0) {
      auto &material = render_list._materials[command.material_index];
      if (command.options.transparent || material.transparent ||
          material.color.w() < 1.0) {
        _transparent.push_back(command);
      } else {
        _opaque.push_back(command);
      }
    }
  }

  render_target._transparent_framebuffer_head.bind();
  V_GL(glClearColor(0, 0, 0, 0));
  V_GL(glClear(GL_COLOR_BUFFER_BIT));

  if (camera_block.flags & CameraBlock::SampleShadingFlag) {
    glEnable(GL_SAMPLE_SHADING);
    glMinSampleShading(1.0);
  } else {
    glDisable(GL_SAMPLE_SHADING);
  }

  render_target._opaque_framebuffer.bind();
  V_GL(glDepthMask(GL_TRUE));
  V_GL(glDisable(GL_BLEND));
  render(render_list, _opaque);

  if (_transparent.empty()) {

    std::lock_guard<std::mutex> lock(render_target._mutex);

    render_target._front_framebuffer.bind();
    render_target._front_framebuffer.attach(render_target._front_colorbuffer,
                                            GL_COLOR_ATTACHMENT0);

    V_GL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                           render_target._front_framebuffer.id()));
    V_GL(glBindFramebuffer(GL_READ_FRAMEBUFFER,
                           render_target._opaque_framebuffer.id()));
    V_GL(glBlitFramebuffer(0, 0, render_target._width, render_target._height, 0,
                           0, render_target._width, render_target._height,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST));

    render_target._front_framebuffer.bind();
    V_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_RENDERBUFFER, 0));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    V_GL(glFlush());
    V_GL(glFinish());

  } else {

    if (camera_block.flags & CameraBlock::TransparentSampleShadingFlag) {
      glEnable(GL_SAMPLE_SHADING);
      glMinSampleShading(1.0);
    } else {
      glDisable(GL_SAMPLE_SHADING);
    }

    render_target._transparent_framebuffer_tail.bind();
    V_GL(glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE, GL_ZERO,
                             GL_ONE_MINUS_SRC_ALPHA));
    V_GL(glDisable(GL_CULL_FACE));
    V_GL(glDepthMask(GL_FALSE));
    V_GL(glEnable(GL_BLEND));
    V_GL(glClearColor(0, 0, 0, 1));
    V_GL(glClear(GL_COLOR_BUFFER_BIT));
    render(render_list, _transparent);

    render_target._transparent_framebuffer_head.bind();
    V_GL(glDepthMask(GL_TRUE));
    V_GL(glDisable(GL_BLEND));
    render(render_list, _transparent);

    V_GL(glEnable(GL_CULL_FACE));

    std::lock_guard<std::mutex> lock(render_target._mutex);

    render_target._front_framebuffer.attach(render_target._front_colorbuffer,
                                            GL_COLOR_ATTACHMENT0);
    render_target._front_framebuffer.bind();

    V_GL(glDepthMask(GL_FALSE));

    glDisable(GL_SAMPLE_SHADING);

    screen_quad->bind();

    blend_shader->use();

    V_GL(glUniform1i(glGetUniformLocation(blend_shader->program(), "opaque"),
                     1));
    V_GL(glActiveTexture(GL_TEXTURE1));
    V_GL(glBindTexture(GL_TEXTURE_2D_MULTISAMPLE,
                       render_target._opaque_texture.id()));

    V_GL(glUniform1i(
        glGetUniformLocation(blend_shader->program(), "transparent_head"), 2));
    V_GL(glActiveTexture(GL_TEXTURE2));
    V_GL(glBindTexture(GL_TEXTURE_2D_MULTISAMPLE,
                       render_target._transparent_texture_head.id()));

    V_GL(glUniform1i(
        glGetUniformLocation(blend_shader->program(), "transparent_tail_color"),
        3));
    V_GL(glActiveTexture(GL_TEXTURE3));
    V_GL(glBindTexture(GL_TEXTURE_2D_MULTISAMPLE,
                       render_target._transparent_texture_tail_color.id()));

    V_GL(glUniform1i(
        glGetUniformLocation(blend_shader->program(), "transparent_tail_alpha"),
        4));
    V_GL(glActiveTexture(GL_TEXTURE4));
    V_GL(glBindTexture(GL_TEXTURE_2D_MULTISAMPLE,
                       render_target._transparent_texture_tail_alpha.id()));

    V_GL(glUniform1i(glGetUniformLocation(blend_shader->program(), "samples"),
                     render_target._samples));

    V_GL(glDrawElements(GL_TRIANGLES, screen_quad->data().indices.size(),
                        GL_UNSIGNED_INT, nullptr));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    V_GL(glActiveTexture(GL_TEXTURE1));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glActiveTexture(GL_TEXTURE2));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glActiveTexture(GL_TEXTURE3));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glActiveTexture(GL_TEXTURE4));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glFlush());
    V_GL(glFinish());
  }

  glDisable(GL_SAMPLE_SHADING);

  V_GL(glDepthMask(GL_TRUE));
  V_GL(glDisable(GL_BLEND));

  V_GL(glBindVertexArray(0));
  V_GL(glUseProgram(0));
}
