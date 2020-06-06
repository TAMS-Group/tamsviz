// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderer.h"

#include "shader.h"
#include "uniformbuffer.h"

#include "../core/log.h"

Renderer::Renderer() {
  default_shader = std::make_shared<Shader>(
      "package://" ROS_PACKAGE_NAME "/shaders/default.vert",
      "package://" ROS_PACKAGE_NAME "/shaders/default.frag");
}

void Renderer::render(const CameraBlock &camera_block,
                      const RenderList &render_list) {

  // V_GL(glEnable(GL_DITHER));
  V_GL(glEnable(GL_FRAMEBUFFER_SRGB));
  V_GL(glEnable(GL_DEPTH_TEST));
  V_GL(glEnable(GL_CULL_FACE));
  V_GL(glCullFace(GL_BACK));

  /*V_GL(glDisable(GL_SAMPLE_SHADING));
  V_GL(glDisable(GL_SAMPLE_SHADING_ARB));
  V_GL(glDisable(GL_SAMPLE_COVERAGE));
  V_GL(glDisable(GL_MULTISAMPLE));
  V_GL(glMinSampleShading(0.0));*/

  V_GL(glEnable(GL_SAMPLE_SHADING));
  V_GL(glEnable(GL_SAMPLE_COVERAGE));
  V_GL(glEnable(GL_MULTISAMPLE));
  V_GL(glMinSampleShading(1.0));

  // V_GL(glDisable(GL_MULTISAMPLE));

  camera_uniform_buffer.update(camera_block);
  camera_uniform_buffer.bind();

  /*if (rand() % 2)
    glEnable(GL_FRAMEBUFFER_SRGB);
  else
    glDisable(GL_FRAMEBUFFER_SRGB);*/

  // V_GL(glEnable(GL_MULTISAMPLE));
  // V_GL(glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE));
  // V_GL(glEnable(GL_SAMPLE_ALPHA_TO_ONE));
  // V_GL(glSampleCoverage(0.5, GL_FALSE));

  material_buffer.update(render_list._materials);

  LightArrayBlock light_array;
  light_array.light_count = std::min(sizeof(light_array.light_array) /
                                         sizeof(light_array.light_array[0]),
                                     render_list._lights.size());
  for (size_t light_index = 0; light_index < light_array.light_count;
       light_index++) {
    light_array.light_array[light_index] = render_list._lights[light_index];
  }
  light_buffer.update(light_array);
  light_buffer.bind();

  default_shader->use();

  for (auto &command : render_list._commands) {

    // LOG_DEBUG("material " << command.material_index << " / "
    //                      << render_list._materials.size());

    if (command.element_count <= 0) {
      continue;
    }

    material_buffer.bind(command.material_index);

    auto &material = render_list._materials[command.material_index];

    V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::color));
    V_GL(glBindTexture(GL_TEXTURE_2D, material.color_texture));

    V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::normal));
    V_GL(glBindTexture(GL_TEXTURE_2D, material.normal_texture));

    V_GL(glBindVertexArray(command.vertex_array_object));

    // LOG_DEBUG("instance count " << command.instance_count);
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

  V_GL(glBindVertexArray(0));
  V_GL(glUseProgram(0));
}
