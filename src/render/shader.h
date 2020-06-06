// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "opengl.h"
#include "resource.h"

#include <string>

enum class Samplers : GLuint {
  color = 1,
  normal = 2,
};

enum class VertexAttributes : GLuint {
  position = 0,
  normal = 1,
  texcoord = 2,
  tangent = 3,
  bitangent = 4,
  color = 5,
  extra = 6,
  pose_x = 7,
  pose_y = 8,
  pose_z = 9,
};

enum class UniformBindingPoint : GLuint {
  camera = 1,
  material = 2,
  lights = 3,
};

class Shader;

typedef ResourceManager<Shader, std::string, std::string> ShaderManager;

class Shader : public ResourceBase {
  GLuint _program = 0;
  std::string _vertex_shader_url;
  std::string _fragment_shader_url;
  void create();
  void destroy();
  void addShader(GLenum type, const std::string &url);

public:
  Shader(const std::string &vertex_shader_url,
         const std::string &fragment_shader_url)
      : _vertex_shader_url(vertex_shader_url),
        _fragment_shader_url(fragment_shader_url) {}
  ~Shader() { destroy(); }
  int program();
  void use();
  static ShaderManager &manager();
};
