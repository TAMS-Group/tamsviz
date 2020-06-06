// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "../core/loader.h"

#include "opengl.h"
#include "resource.h"

namespace cv {
class Mat;
}

enum class TextureType {
  Color = 0,
  Normal = 1,
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
  // Loader<std::shared_ptr<cv::Mat>, std::string> _loader;
  Loader<TextureData> _loader;

public:
  Texture(TextureType type) : _type(type) {}
  Texture(const std::string &url, TextureType type);
  GLuint update(const cv::Mat &image);
  GLuint update();
  const std::string &url() const { return _url; }
  static TextureManager &manager();
};
