// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "uniformbuffer.h"

void UniformBufferBase::destroy() {
  if (_buffer) {
    GLuint buffer = _buffer;
    cleanup([buffer]() { V_GL(glDeleteBuffers(1, &buffer)); });
    _buffer = 0;
  }
}

void UniformBufferBase::update(const void *data, size_t size) {
  if (!_buffer) {
    V_GL(glGenBuffers(1, &_buffer));
  }
  V_GL(glBindBuffer(GL_UNIFORM_BUFFER, _buffer));
  V_GL(glBufferData(GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW));
}

void UniformBufferBase::bind(size_t binding_point) {
  V_GL(glBindBufferBase(GL_UNIFORM_BUFFER, binding_point, _buffer));
}

void UniformBufferBase::bind(size_t binding_point, size_t offset, size_t size) {
  V_GL(glBindBufferRange(GL_UNIFORM_BUFFER, binding_point, _buffer, offset,
                         size));
}
