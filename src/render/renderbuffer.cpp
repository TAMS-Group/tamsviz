// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderbuffer.h"

void Renderbuffer::destroy() {
  if (_id) {
    GLuint fbo = _id;
    cleanup([fbo]() { V_GL(glDeleteRenderbuffers(1, &fbo)); });
    _id = 0;
  }
}

void Renderbuffer::create() {
  if (!_id) {
    V_GL(glGenRenderbuffers(1, &_id));
  }
}

void Renderbuffer::bind() {
  create();
  V_GL(glBindRenderbuffer(GL_RENDERBUFFER, _id));
}

void Renderbuffer::update(int width, int height, int format, int samples) {
  create();
  if (_watcher.changed(width, height, format, samples)) {
    bind();
    V_GL(glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, format,
                                          width, height));
  }
}
