// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "opengl.h"
#include "resource.h"

#include <cstring>
#include <vector>

#include "../core/log.h"

class UniformBufferBase : public ResourceBase {
  GLuint _buffer = 0;
  void destroy();

protected:
  void bind(size_t binding_point);
  void bind(size_t binding_point, size_t offset, size_t size);

public:
  ~UniformBufferBase() { destroy(); }
  void update(const void *data, size_t size);
};

template <class T> class UniformBuffer : public UniformBufferBase {
  std::vector<uint8_t> _aligned_data;
  ssize_t _aligned_stride = -1;
  GLuint _buffer = 0;
  size_t _binding_point = 0;
  void destroy();

public:
  UniformBuffer(size_t binding_point) : _binding_point(binding_point) {}
  void update(const T &value) { UniformBufferBase::update(&value, sizeof(T)); }
  template <class CONTAINER> void update(const CONTAINER &data) {
    if (_aligned_stride < 0) {
      GLint buffer_alignment = -1;
      V_GL(
          glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &buffer_alignment));
      if (buffer_alignment < 1) {
        LOG_ERROR("failed to get buffer alignment, using 256");
        buffer_alignment = 256;
      }

      _aligned_stride =
          ((sizeof(T) + buffer_alignment - 1) / buffer_alignment) *
          buffer_alignment;
    }
    _aligned_data.resize(_aligned_stride * data.size());
    for (size_t index = 0; index < data.size(); index++) {
      std::memcpy(_aligned_data.data() + index * _aligned_stride,
                  (const T *)&(data[index]), sizeof(T));
    }
    UniformBufferBase::update(_aligned_data.data(), _aligned_data.size());
  }
  void bind() { UniformBufferBase::bind(_binding_point); }
  void bind(size_t index) {
    UniformBufferBase::bind(_binding_point, _aligned_stride * index,
                            _aligned_stride);
  }
};
