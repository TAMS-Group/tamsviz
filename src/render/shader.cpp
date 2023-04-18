// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "shader.h"

#include "../core/destructor.h"
#include "../core/log.h"
#include "opengl.h"

#include <regex>
#include <sstream>
#include <vector>

static const char *g_tamsviz_shader_preamble = "#version 400\n";

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

static std::string loadShader(const std::string &url) {

  static const std::regex local_include_regex(
      "\\s*\\#include\\s+\\\"([^\\>]*)\\\"(.*)?");

  static const std::regex package_include_regex(
      "\\s*\\#include\\s+\\<([^\\>]*)\\>(.*)?");

  std::smatch regex_match;

  std::string source;
  loadResource(url, source);
  std::istringstream istream{source};

  std::ostringstream ostream;

  ostream << "// " << url << "\n";

  std::string line;
  size_t index = 1;

  while (getline(istream, line)) {

    if (std::regex_match(line, regex_match, local_include_regex)) {

      std::string include_url = url;

      {
        auto p = include_url.find_last_of("/");
        if (p != std::string::npos) {
          include_url.resize(p + 1);
        }
      }

      include_url = include_url + regex_match[1].str();

      LOG_INFO("local shader include " << include_url);
      ostream << "// " << line << "\n";
      ostream << loadShader(include_url) << "\n";
      ostream << "// " << url << "\n";
      ostream << regex_match[2].str() << "\n";

    } else if (std::regex_match(line, regex_match, package_include_regex)) {

      std::string include_url = regex_match[1].str();

      LOG_INFO("package shader include " << include_url);
      ostream << "// " << line << "\n";
      ostream << loadShader(include_url) << "\n";
      ostream << "// " << url << "\n";
      ostream << regex_match[2].str() << "\n";

    } else {

      ostream << "#line " << index << "\n";
      ostream << line << "\n";
    }

    index++;
  }

  return ostream.str();
};

void Shader::addShader(GLenum type, const std::string &url) {

  LOG_INFO("loading shader " << url);

  std::string source = g_tamsviz_shader_preamble + loadShader(url);
  if (source.empty()) {
    throw std::runtime_error("shader file " + url + " is empty");
  }

  // LOG_DEBUG("SHADER\n" << source);

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
    auto bindSampler = [&](const Samplers &sampler, const char *name) {
      GLint index = -1;
      V_GL(index = glGetUniformLocation(_program, name));
      if (index != -1) {
        V_GL(glUniform1i(index, (int)sampler));
      }
    };
    bindSampler(Samplers::color, "color_sampler");
    bindSampler(Samplers::normal, "normal_sampler");
    bindSampler(Samplers::shadowmap, "shadow_map_sampler");
    bindSampler(Samplers::shadowcube, "shadow_cube_sampler");
    bindSampler(Samplers::environment, "environment_sampler");
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
