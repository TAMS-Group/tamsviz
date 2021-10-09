// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "renderer.h"

#include "../core/log.h"
#include "mesh.h"
#include "renderlist.h"
#include "rendertarget.h"
#include "shader.h"
#include "texture.h"
#include "transformations.h"
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
                      const std::vector<RenderCommand> &commands,
                      bool picking) {

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

    if (command.options.primitive_type == GL_POINTS &&
        command.options.point_size != 1) {
      V_GL(glPointSize(std::max(1.0f, command.options.point_size)));
    }

    // if (!command.options.colors_linear && !picking) {
    //  V_GL(glDisable(GL_FRAMEBUFFER_SRGB));
    //}

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

    // if (!command.options.colors_linear && !picking) {
    //  V_GL(glEnable(GL_FRAMEBUFFER_SRGB));
    //}

    if (command.options.primitive_type == GL_POINTS &&
        command.options.point_size != 1) {
      V_GL(glPointSize(1));
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

      light_array.light_array[light_index].view_matrix =
          light_array.light_array[light_index].view_matrix *
          camera_block.view_matrix;
    }

    light.type = (light.type & 0xff);

    if (light.type == uint32_t(LightType::PointShadow)) {
      light.shadow_bias *= 1.0 / render_list._parameters.shadow_cube_resolution;
    } else {
      light.shadow_bias *= 1.0 / render_list._parameters.shadow_map_resolution;
    }
  }
  light_buffer.update(light_array);
  light_buffer.bind();

  V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::environment));
  V_GL(glBindTexture(GL_TEXTURE_CUBE_MAP,
                     render_list._parameters.environment_cube_map));
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

  render(render_list, render_list._commands, true);

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

void Renderer::_splitTransparentOpaque(const RenderList &render_list) {
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
}

void Renderer::renderShadows(const RenderList &render_list) {

  uint32_t shadow_map_size = render_list._parameters.shadow_map_resolution;
  uint32_t shadow_cube_size = render_list._parameters.shadow_cube_resolution;

  _splitTransparentOpaque(render_list);

  _shadow_framebuffer.create();
  _shadow_map_array.create();
  _shadow_cube_array.create();

  default_shader->use();

  V_GL(glActiveTexture(GL_TEXTURE0 + uint32_t(Samplers::shadowmap)));
  V_GL(glBindTexture(GL_TEXTURE_2D_ARRAY, _shadow_map_array.id()));
  if (_shadow_map_watcher.changed(render_list._shadow_map_count,
                                  shadow_map_size)) {
    LOG_DEBUG("shadow map count " << render_list._shadow_map_count);
    V_GL(glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T,
                         GL_CLAMP_TO_EDGE));
    V_GL(
        glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    V_GL(
        glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    V_GL(glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BASE_LEVEL, 0));
    V_GL(glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE,
                         GL_COMPARE_REF_TO_TEXTURE));
    V_GL(glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT,
                      shadow_map_size, shadow_map_size,
                      render_list._shadow_map_count, 0, GL_DEPTH_COMPONENT,
                      GL_UNSIGNED_BYTE, nullptr));
  }
  V_GL(glActiveTexture(GL_TEXTURE0));

  V_GL(glActiveTexture(GL_TEXTURE0 + uint32_t(Samplers::shadowcube)));
  V_GL(glBindTexture(GL_TEXTURE_CUBE_MAP_ARRAY, _shadow_cube_array.id()));
  if (_shadow_cube_watcher.changed(render_list._shadow_cube_count,
                                   shadow_cube_size)) {
    LOG_DEBUG("shadow cube count " << render_list._shadow_cube_count);
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_WRAP_S,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_WRAP_T,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameterf(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_WRAP_R,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_MIN_FILTER,
                         GL_LINEAR));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_MAG_FILTER,
                         GL_LINEAR));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_BASE_LEVEL, 0));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_COMPARE_MODE,
                         GL_COMPARE_REF_TO_TEXTURE));
    V_GL(glTexImage3D(GL_TEXTURE_CUBE_MAP_ARRAY, 0, GL_DEPTH_COMPONENT,
                      shadow_cube_size, shadow_cube_size,
                      render_list._shadow_cube_count * 6, 0, GL_DEPTH_COMPONENT,
                      GL_UNSIGNED_BYTE, nullptr));
  }
  V_GL(glActiveTexture(GL_TEXTURE0));

  for (auto &light : render_list._lights) {
    switch (light.type & 0xff) {

    case uint32_t(LightType::SpotShadow):
    case uint32_t(LightType::DirectionalShadow): {
      CameraBlock shadow_camera_block;
      shadow_camera_block.view_matrix = light.view_matrix;
      shadow_camera_block.projection_matrix = light.projection_matrix;
      shadow_camera_block.flags = CameraBlock::ShadowCameraFlag;
      prepare(shadow_camera_block, render_list);

      _shadow_framebuffer.bind();
      V_GL(glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                     _shadow_map_array.id(), 0,
                                     light.shadow_index));

      V_GL(glViewport(0, 0, shadow_map_size, shadow_map_size));

      V_GL(glClearColor(0, 0, 0, 0));
      V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

      glEnable(GL_SCISSOR_TEST);
      V_GL(glScissor(1, 1, shadow_map_size - 2, shadow_map_size - 2));

      render(render_list, _opaque);

      glDisable(GL_SCISSOR_TEST);

      break;
    }

    case uint32_t(LightType::PointShadow): {

      for (size_t face = 0; face < 6; face++) {

        CameraBlock shadow_camera_block;

        {
          auto &v = shadow_camera_block.view_matrix;
          v = cubeMapViewMatrix(light.position.cast<double>(), face)
                  .cast<float>();
        }

        double far = 20.0;
        double near = 0.1;
        shadow_camera_block.projection_matrix =
            cubeMapProjectionMatrix(near, far).cast<float>();

        shadow_camera_block.flags = CameraBlock::ShadowCameraFlag;
        prepare(shadow_camera_block, render_list);

        _shadow_framebuffer.bind();
        V_GL(glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                       _shadow_cube_array.id(), 0,
                                       light.shadow_index * 6 + face));

        V_GL(glViewport(0, 0, shadow_cube_size, shadow_cube_size));

        V_GL(glClearColor(0, 0, 0, 0));
        V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

        render(render_list, _opaque);
      }
      break;
    }
    }
  }
}

void Renderer::render(RenderTarget &render_target,
                      const CameraBlock &camera_block,
                      const RenderList &render_list) {

  V_GL(glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS));

  _splitTransparentOpaque(render_list);

  default_shader->use();

  prepare(camera_block, render_list);

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

  {
    blend_shader->use();
    std::lock_guard<std::mutex> lock(render_target._mutex);
    V_GL(glUniform1i(
        glGetUniformLocation(blend_shader->program(), "transparency"),
        !_transparent.empty()));
    V_GL(glUniform1i(
        glGetUniformLocation(blend_shader->program(), "tone_mapping"),
        render_list._parameters.tone_mapping));
    // V_GL(glUniform1f(
    //     glGetUniformLocation(blend_shader->program(), "exposure"),
    //     float(render_list._parameters.exposure / render_target._samples)));
    V_GL(glUniform1f(glGetUniformLocation(blend_shader->program(), "exposure"),
                     float(render_list._parameters.exposure)));
    V_GL(glUniform1i(glGetUniformLocation(blend_shader->program(), "samples"),
                     render_target._samples));
    V_GL(glUniform1f(
        glGetUniformLocation(blend_shader->program(), "black_level"),
        float(render_list._parameters.black_level)));
    V_GL(glUniform1f(
        glGetUniformLocation(blend_shader->program(), "white_level"),
        float(render_list._parameters.white_level)));
  }

  if (_transparent.empty()) {

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

    V_GL(glDrawElements(GL_TRIANGLES, screen_quad->data().indices.size(),
                        GL_UNSIGNED_INT, nullptr));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    V_GL(glActiveTexture(GL_TEXTURE1));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glFlush());
    V_GL(glFinish());

  } else {

    default_shader->use();

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
