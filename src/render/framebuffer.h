// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "opengl.h"
#include "resource.h"

class Renderbuffer;
class Texture;

class Framebuffer : public ResourceBase {
protected:
  GLuint _id = 0;
  void destroy();

public:
  ~Framebuffer() { destroy(); }
  inline GLuint id() const { return _id; }
  void create();
  void bind();
  void attach(Renderbuffer &renderbuffer, int attachment);
  void attach(Texture &texture, int attachment);
};
