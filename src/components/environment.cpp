// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "environment.h"

#include "../render/renderer.h"
#include "../render/shader.h"

void EnvironmentComponent::renderSync(const RenderSyncContext &context) {
  Component::renderSync(context);

  // setup environment light
  _light_block.color = backgroundColor().toLinearVector4f().head(3) *
                       (float)(ambientLighting() * backgroundBrightness());
  _light_block.hemispheric = float(hemisphericLighting());
  _light_block.type = uint32_t(useEnvironmentMap() ? LightType::Environment
                                                   : LightType::Ambient);
  _light_block.color2 = groundColor().toLinearVector4f().head(3) *
                        (float)(ambientLighting() * backgroundBrightness());
  context.render_list->push(_light_block);

  // create environment texture
  if (_environment_map_watcher.changed(environmentMap(), useEnvironmentMap()) ||
      _invalidated.poll()) {
    _environment_loader = std::make_shared<Loader<ImageLoader>>(
        useEnvironmentMap() ? environmentMap() : "");
  }
  _use_environment_map = useEnvironmentMap();

  // setup environment cube material
  _environment_material_block.color = backgroundColor().toLinearVector4f();
  _environment_material_block.brightness =
      (float)(ambientLighting() * backgroundBrightness());
  _environment_material_block.flags = (2 | 8);
}

void EnvironmentComponent::renderAsync(const RenderAsyncContext &context) {
  Component::renderAsync(context);

  //_environment_mesh->bind();

  bool environment_changed = false;

  // load environment map
  if (_environment_loader) {
    if (auto data = _environment_loader->load()) {
      LOG_DEBUG("environment map loaded");
      _environment_loader = nullptr;
      environment_changed = true;

      _environment_map_texture = nullptr;
      if (!data->url.empty()) {

        _environment_map_texture = std::make_shared<Texture>();
        _environment_map_texture->create();
        _environment_map_texture->update(data->image);

        V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::color));
        V_GL(glBindTexture(GL_TEXTURE_2D, _environment_map_texture->id()));

        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                             GL_CLAMP_TO_EDGE));

        V_GL(glBindTexture(GL_TEXTURE_2D, 0));
      }
    }
  }

  // create environment cube
  // size_t cube_map_size =
  //      (_use_environment_map && _environment_map_texture ?
  //      _environment_cube_size : 32);
  size_t cube_map_size = _environment_cube_size;
  if (_environment_cube_watcher.changed(cube_map_size)) {
    environment_changed = true;
    LOG_DEBUG("creating environment cube map");
    _environment_cube_map.create();
    V_GL(glActiveTexture(GL_TEXTURE0 + uint32_t(Samplers::environment)));
    V_GL(glBindTexture(GL_TEXTURE_CUBE_MAP, _environment_cube_map.id()));

    V_GL(glTexStorage2D(GL_TEXTURE_CUBE_MAP, _environment_mip_levels,
                        GL_RGBA16F, cube_map_size, cube_map_size));

    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R,
                         GL_CLAMP_TO_EDGE));
    V_GL(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,
                         GL_LINEAR_MIPMAP_LINEAR));
    V_GL(
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    V_GL(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
    V_GL(glActiveTexture(GL_TEXTURE0));
  }

  // render environment cube map
  if (environment_changed) {
    LOG_INFO("updating environment map");

    _env_shader->use();

    {
      Framebuffer environment_framebuffer;
      environment_framebuffer.bind();

      size_t s = cube_map_size;
      for (size_t level = 0; level < _environment_mip_levels; level++) {
        for (size_t face = 0; face < 6; face++) {

          _env_shader->use();

          V_GL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                      GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                                      _environment_cube_map.id(), level));
          V_GL(glViewport(0, 0, s, s));
          V_GL(glClearColor(0, 0, 1, 1));
          V_GL(glClear(GL_COLOR_BUFFER_BIT));

          if (_environment_map_texture) {

            V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::color));
            V_GL(glBindTexture(GL_TEXTURE_2D, _environment_map_texture->id()));

            CameraBlock camera_block;
            camera_block.view_matrix =
                cubeMapViewMatrix(Eigen::Vector3d::Zero(), face).cast<float>();
            double far = 100.0;
            double near = 0.1;
            camera_block.projection_matrix =
                cubeMapProjectionMatrix(near, far).cast<float>();
            _camera_uniform_buffer.update(camera_block);
            _camera_uniform_buffer.bind();

            V_GL(glUniform1i(
                glGetUniformLocation(_env_shader->program(), "levels"),
                _environment_mip_levels));
            V_GL(glUniform1i(
                glGetUniformLocation(_env_shader->program(), "level"), level));

            _environment_mesh->bind();

            if (_environment_mesh->data().indices.size()) {
              V_GL(glDrawElements(GL_TRIANGLES,
                                  _environment_mesh->data().indices.size(),
                                  GL_UNSIGNED_INT, nullptr));
            } else {
              V_GL(glDrawArrays(GL_TRIANGLES, 0,
                                _environment_mesh->data().positions.size()));
            }
          }
        }

        s = std::max(size_t(1), (s / 2));
      }
    }

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    V_GL(glActiveTexture(GL_TEXTURE0 + (int)Samplers::color));
    V_GL(glBindTexture(GL_TEXTURE_2D, 0));

    V_GL(glUseProgram(0));
  }

  // send environment cube map
  context.render_list->parameters().environment_cube_map =
      _environment_cube_map.id();

  // render environment
  _environment_material_block.color_texture = 0;
  if (_use_environment_map && _environment_map_texture) {
    _environment_material_block.color_texture = _environment_map_texture->id();
    InstanceBlock instance_block;
    instance_block.setPose(Eigen::Affine3d(Eigen::Scaling(100.0)));
    RenderOptions render_options;
    render_options.double_sided = true;
    context.render_list->push(_environment_material_block, _environment_mesh,
                              instance_block, render_options);
  }
}
