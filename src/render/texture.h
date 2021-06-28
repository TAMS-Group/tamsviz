// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "../core/loader.h"
#include "../core/watcher.h"

#include "opengl.h"
#include "resource.h"

namespace cv {
class Mat;
}

enum class TextureType {
  Color = 0,
  Normal = 1,
  Linear = 2,
};

class Texture;

typedef ResourceManager<Texture, std::string, TextureType> TextureManager;

class TextureData;

class TextureBase : public ResourceBase {
protected:
  GLuint _id = 0;
  void destroy();

public:
  ~TextureBase() { destroy(); }
  inline GLuint id() const { return _id; }
  void create();
};

class Texture : public TextureBase {
  bool _loaded = false;
  TextureType _type = TextureType::Color;
  std::string _url;
  Loader<TextureData> _loader;
  Watcher _watcher;
  bool _transparent = false;
  bool _mipmap = true;

public:
  Texture(TextureType type = TextureType::Color) : _type(type) {}
  Texture(const std::string &url, TextureType type);
  void mipmap(bool mipmap) { _mipmap = mipmap; }
  GLuint update(const cv::Mat &image);
  GLuint update();
  GLuint update(int width, int height, int format, int samples = 0);
  bool transparent() const { return _transparent; }
  const std::string &url() const { return _url; }
  static TextureManager &manager();
};
