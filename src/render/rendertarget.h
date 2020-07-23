// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "framebuffer.h"
#include "renderbuffer.h"
#include "texture.h"

class RenderTarget : public ResourceBase {
  Watcher _watcher;
  bool _can_present = false;

public:
  void update(int width, int height);
  void bind();
  void present(int target = 0);

public:
  int _width = -1, _height = -1;
  int _samples = 0;

  std::mutex _mutex;

  Renderbuffer _depth_buffer;

  Renderbuffer _pick_id;
  Renderbuffer _pick_depth;
  Framebuffer _pick_framebuffer;

  Texture _opaque_texture;
  Framebuffer _opaque_framebuffer;

  Texture _transparent_texture_head;
  Framebuffer _transparent_framebuffer_head;

  Texture _transparent_texture_tail_color;
  Texture _transparent_texture_tail_alpha;
  Framebuffer _transparent_framebuffer_tail;

  Framebuffer _front_framebuffer, _present_framebuffer;
  Texture _front_colorbuffer;
};
