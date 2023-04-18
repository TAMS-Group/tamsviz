// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"

#include "../core/loader.h"
#include "../displays/shapes.h"
#include "../render/imageloader.h"
#include "../render/mesh.h"
#include "../render/renderlist.h"
#include "../render/resource.h"
#include "../render/shader.h"
#include "../render/transformations.h"

struct EnvironmentComponent : Component {

  std::shared_ptr<Shader> _env_shader = std::make_shared<Shader>(
      "package://" ROS_PACKAGE_NAME "/shaders/env.vert",
      "package://" ROS_PACKAGE_NAME "/shaders/env.frag");

  UniformBuffer<CameraBlock> _camera_uniform_buffer{
      (size_t)UniformBindingPoint::camera};

  PROPERTY(Color4, backgroundColor, Color4(0.3, 0.3, 0.3, 1.0));
  PROPERTY(double, backgroundBrightness, 1, min = 0);
  PROPERTY(double, ambientLighting, 1.0, min = 0.0, max = 1.0);
  PROPERTY(Color3, groundColor, Color3(0.0, 0.0, 0.0));
  PROPERTY(double, hemisphericLighting, 0.5, min = 0.0, max = 1.0);

  EventFlag _invalidated{ResourceEvents::instance().reload};

  LightBlock _light_block;

  size_t _environment_mip_levels = 10;
  size_t _environment_cube_size = 1024;

  bool _use_environment_map = false;

  PROPERTY(bool, useEnvironmentMap, false);
  PROPERTY(std::string, environmentMap
           //, "package://" ROS_PACKAGE_NAME "/environments/.jpg"
  );
  Watcher _environment_map_watcher;
  std::shared_ptr<Loader<ImageLoader>> _environment_loader;
  std::shared_ptr<Texture> _environment_map_texture =
      std::make_shared<Texture>();
  Texture _environment_cube_map;
  Watcher _environment_cube_watcher;

  std::shared_ptr<Mesh> _environment_mesh =
      std::make_shared<Mesh>(makeSphere(64, 32));

  MaterialBlock _environment_material_block;

  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
DECLARE_TYPE(EnvironmentComponent, Component);
