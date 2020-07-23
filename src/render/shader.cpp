// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "shader.h"

#include "../core/destructor.h"
#include "../core/log.h"
#include "opengl.h"

#include <vector>

static bool logEmpty(const std::string &s) {
  for (auto c : s) {
    if (c && !std::isspace(c)) {
      return false;
    }
  }
  return true;
}

ShaderManager &Shader::manager() {
  static ShaderManager instance;
  return instance;
}

void Shader::addShader(GLenum type, const std::string &url) {
  std::string source;
  loadResource(url, source);
  if (source.empty()) {
    throw std::runtime_error("shader file " + url + " is empty");
  }
  GLuint shader = 0;
  V_GL(shader = glCreateShader(type));
  Destructor dtor([&]() {
    if (shader) {
      V_GL(glDeleteShader(shader));
      shader = 0;
    }
  });
  auto *src = source.c_str();
  V_GL(glShaderSource(shader, 1, &src, NULL));
  V_GL(glCompileShader(shader));
  {
    std::vector<char> error_log(1024 * 32, 0);
    V_GL(glGetShaderInfoLog(shader, error_log.size(), NULL, error_log.data()));
    std::string error_string(error_log.data(), error_log.size());
    int success = 0;
    V_GL(glGetShaderiv(shader, GL_COMPILE_STATUS, &success));
    if (success != GL_TRUE) {
      LOG_ERROR("failed to compile shader " << url << "\n" << error_string);
      throw std::runtime_error(error_string);
    } else {
      if (!logEmpty(error_string)) {
        LOG_WARN("shader log " << url << "\n" << error_string);
      }
    }
  }
  V_GL(glAttachShader(_program, shader));
}

void Shader::create() {
  if (!_program) {
    destroy();

    V_GL(_program = glCreateProgram());
    addShader(GL_VERTEX_SHADER, _vertex_shader_url);
    addShader(GL_FRAGMENT_SHADER, _fragment_shader_url);

    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::position,
                              "position"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::normal,
                              "normal"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::texcoord,
                              "texcoord"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::tangent,
                              "tangent"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::bitangent,
                              "bitangent"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::color,
                              "color"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::extra,
                              "extra"));

    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::pose_x,
                              "pose_x"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::pose_y,
                              "pose_y"));
    V_GL(glBindAttribLocation(_program, (GLuint)VertexAttributes::pose_z,
                              "pose_z"));

    V_GL(glBindFragDataLocation(_program, 0, "out_color"));
    V_GL(glBindFragDataLocation(_program, 1, "out_blend"));
    V_GL(glBindFragDataLocation(_program, 2, "out_id"));

    V_GL(glLinkProgram(_program));

    {
      GLuint index = GL_INVALID_INDEX;
      V_GL(index = glGetUniformBlockIndex(_program, "camera_block"));
      if (index != GL_INVALID_INDEX) {
        V_GL(glUniformBlockBinding(_program, index,
                                   (uint32_t)UniformBindingPoint::camera));
      }
    }
    {
      GLuint index = GL_INVALID_INDEX;
      V_GL(index = glGetUniformBlockIndex(_program, "material_block"));
      if (index != GL_INVALID_INDEX) {
        V_GL(glUniformBlockBinding(_program, index,
                                   (uint32_t)UniformBindingPoint::material));
      }
    }
    {
      GLuint index = GL_INVALID_INDEX;
      V_GL(index = glGetUniformBlockIndex(_program, "light_block"));
      if (index != GL_INVALID_INDEX) {
        V_GL(glUniformBlockBinding(_program, index,
                                   (uint32_t)UniformBindingPoint::lights));
      }
    }

    {
      std::vector<char> error_log(1024 * 32);
      V_GL(glGetProgramInfoLog(_program, error_log.size(), NULL,
                               error_log.data()));
      std::string error_string(error_log.data(), error_log.size());
      int success = 0;
      V_GL(glGetProgramiv(_program, GL_LINK_STATUS, &success));
      if (success != GL_TRUE) {
        LOG_ERROR("failed to link shader "
                  << " " << _vertex_shader_url << " " << _fragment_shader_url
                  << "\n"
                  << error_string);
        throw std::runtime_error(error_string);
      } else {
        if (!logEmpty(error_string)) {
          LOG_WARN("shader program log "
                   << " " << _vertex_shader_url << " " << _fragment_shader_url
                   << "\n"
                   << error_string);
        }
      }
    }

    V_GL(glUseProgram(_program));
    {
      GLint index = -1;
      V_GL(index = glGetUniformLocation(_program, "color_sampler"));
      if (index != -1) {
        V_GL(glUniform1i(index, (int)Samplers::color));
      }
    }
    {
      GLint index = -1;
      V_GL(index = glGetUniformLocation(_program, "normal_sampler"));
      if (index != -1) {
        V_GL(glUniform1i(index, (int)Samplers::normal));
      }
    }
    V_GL(glUseProgram(0));
  }
}

void Shader::destroy() {
  if (_program) {
    int program = _program;
    cleanup([program]() { V_GL(glDeleteProgram(program)); });
    _program = 0;
  }
}

int Shader::program() {
  if (invalidated()) {
    GLuint previous_program = _program;
    _program = 0;
    try {
      create();
    } catch (const std::exception &e) {
      LOG_ERROR("failed to recompile shader " << _vertex_shader_url << " "
                                              << _fragment_shader_url);
      LOG_ERROR(e.what());
      if (_program) {
        V_GL(glDeleteProgram(_program));
        _program = 0;
      }
    }
    if (_program) {
      V_GL(glDeleteProgram(previous_program));
    } else {
      _program = previous_program;
    }
  }
  if (!_program) {
    create();
  }
  return _program;
}

void Shader::use() { V_GL(glUseProgram(program())); }
