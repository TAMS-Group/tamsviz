// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "rendertarget.h"

#include "../core/log.h"

void RenderTarget::update(int width, int height) {
  if (_watcher.changed(width, height)) {
    std::lock_guard<std::mutex> lock(_mutex);

    int samples = 4;
    {
      int gl_max_samples = 1;
      int gl_max_integer_samples = 1;
      V_GL(glGetIntegerv(GL_MAX_SAMPLES, &gl_max_samples));
      V_GL(glGetIntegerv(GL_MAX_INTEGER_SAMPLES, &gl_max_integer_samples));
      samples = std::min(samples, gl_max_samples);
      samples = std::min(samples, gl_max_integer_samples);
    }

    _samples = samples;

    _depth_buffer.update(width, height, GL_DEPTH_COMPONENT, samples);

    _pick_id.update(width, height, GL_R32UI);
    _pick_depth.update(width, height, GL_DEPTH_COMPONENT);
    _pick_framebuffer.attach(_pick_id, GL_COLOR_ATTACHMENT0);
    _pick_framebuffer.attach(_pick_depth, GL_DEPTH_ATTACHMENT);
    {
      _pick_framebuffer.bind();
      std::vector<GLenum> buffers = {GL_NONE, GL_NONE, GL_COLOR_ATTACHMENT0};
      V_GL(glDrawBuffers(3, buffers.data()));
    }

    _opaque_texture.update(width, height, GL_RGBA16F, samples);
    _opaque_framebuffer.attach(_depth_buffer, GL_DEPTH_ATTACHMENT);
    _opaque_framebuffer.attach(_opaque_texture, GL_COLOR_ATTACHMENT0);

    _transparent_texture_head.update(width, height, GL_RGBA16F, samples);
    _transparent_framebuffer_head.attach(_depth_buffer, GL_DEPTH_ATTACHMENT);
    _transparent_framebuffer_head.attach(_transparent_texture_head,
                                         GL_COLOR_ATTACHMENT0);

    _transparent_texture_tail_color.update(width, height, GL_RGBA16F, samples);
    _transparent_texture_tail_alpha.update(width, height, GL_R16F, samples);
    _transparent_framebuffer_tail.attach(_transparent_texture_tail_color,
                                         GL_COLOR_ATTACHMENT0);
    _transparent_framebuffer_tail.attach(_transparent_texture_tail_alpha,
                                         GL_COLOR_ATTACHMENT1);
    _transparent_framebuffer_tail.attach(_depth_buffer, GL_DEPTH_ATTACHMENT);
    {
      _transparent_framebuffer_tail.bind();
      std::vector<GLenum> buffers = {GL_COLOR_ATTACHMENT0,
                                     GL_COLOR_ATTACHMENT1};
      V_GL(glDrawBuffers(2, buffers.data()));
    }

    _front_colorbuffer.update(width, height, GL_SRGB8);

    _width = width;
    _height = height;

    _can_present = true;
  }
}

void RenderTarget::bind() { _opaque_framebuffer.bind(); }

void RenderTarget::present(int target) {
  std::lock_guard<std::mutex> lock(_mutex);
  if (_can_present) {
    if (!_present_framebuffer.id()) {
      _present_framebuffer.create();
    }

    _present_framebuffer.bind();
    _present_framebuffer.attach(_front_colorbuffer, GL_COLOR_ATTACHMENT0);

    V_GL(glBindFramebuffer(GL_READ_FRAMEBUFFER, _present_framebuffer.id()));
    V_GL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target));
    V_GL(glBlitFramebuffer(0, 0, _width, _height, 0, 0, _width, _height,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST));

    _present_framebuffer.bind();
    V_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_RENDERBUFFER, 0));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
  }
}
