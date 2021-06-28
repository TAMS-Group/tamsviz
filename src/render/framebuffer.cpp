// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "framebuffer.h"

#include "renderbuffer.h"
#include "texture.h"

void Framebuffer::destroy() {
  if (_id) {
    GLuint fbo = _id;
    cleanup([fbo]() { V_GL(glDeleteFramebuffers(1, &fbo)); });
    _id = 0;
  }
}

void Framebuffer::create() {
  if (!_id) {
    V_GL(glGenFramebuffers(1, &_id));
  }
}

void Framebuffer::bind() {
  create();
  V_GL(glBindFramebuffer(GL_FRAMEBUFFER, _id));
}

void Framebuffer::attach(Renderbuffer &renderbuffer, int attachment) {
  bind();
  renderbuffer.create();
  V_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER,
                                 renderbuffer.id()));
}

void Framebuffer::attach(Texture &texture, int attachment) {
  bind();
  V_GL(glFramebufferTexture(GL_FRAMEBUFFER, attachment, texture.id(), 0));
}
