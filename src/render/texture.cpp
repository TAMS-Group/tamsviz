// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "texture.h"

#include "../core/log.h"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

struct TextureData {
  cv::Mat image;
  TextureData(const std::string &url) {
    if (url.empty()) {
      return;
    }
    std::vector<uint8_t> data;
    try {
      loadResource(url, data);
    } catch (std::exception &ex) {
      LOG_ERROR("failed to read texture file " << url);
      return;
    }
    if (data.empty()) {
      LOG_ERROR("texture file is empty " << url);
      return;
    }
    auto img = cv::imdecode(data, cv::IMREAD_COLOR);
    if (img.data == nullptr) {
      LOG_ERROR("failed to decode texture " << url);
      return;
    }
    image = img;
  }
};

Texture::Texture(const std::string &url, TextureType type)
    : _url(url), _type(type), _loader(url) {}

void TextureBase::destroy() {
  if (_id) {
    GLuint texture = _id;
    cleanup([texture]() { V_GL(glDeleteTextures(1, &texture)); });
    _id = 0;
  }
}

void TextureBase::create() {
  if (!_id) {
    V_GL(glGenTextures(1, &_id));
  }
}

GLuint Texture::update(const cv::Mat &img) {
  cv::Mat image = img;
  cv::flip(image, image, 0);
  create();
  V_GL(glActiveTexture(GL_TEXTURE0));
  V_GL(glBindTexture(GL_TEXTURE_2D, _id));
  switch (image.elemSize()) {
  case 1:
    V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
    break;
  case 2:
    V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 2));
    break;
  case 3:
    V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
    break;
  default:
    V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 4));
    break;
  }
  V_GL(glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize()));
  if (_mipmap) {
    V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                         GL_LINEAR_MIPMAP_LINEAR));
  } else {
    V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
  }
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
  if (_type == TextureType::Color) {
    switch (image.channels()) {
    case 3:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8, image.cols, image.rows, 0,
                        GL_BGR, GL_UNSIGNED_BYTE, image.data));
      break;
    case 4:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, image.cols,
                        image.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, image.data));
      break;
    default:
      LOG_ERROR("invalid channel count " << image.channels()
                                         << " for color image " << _url);
    }
  }
  if (_type == TextureType::Normal) {
    switch (image.channels()) {
    case 3:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, image.cols, image.rows, 0,
                        GL_BGR, GL_UNSIGNED_BYTE, image.data));
      break;
    case 4:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.cols, image.rows, 0,
                        GL_BGRA, GL_UNSIGNED_BYTE, image.data));
      break;
    default:
      LOG_ERROR("invalid channel count " << image.channels()
                                         << " for normal map " << _url);
    }
  }
  if (_type == TextureType::Linear) {
    switch (image.channels()) {
    case 1:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, image.cols, image.rows, 0,
                        GL_RED, GL_UNSIGNED_BYTE, image.data));
      break;
    case 3:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, image.cols, image.rows, 0,
                        GL_BGR, GL_UNSIGNED_BYTE, image.data));
      break;
    case 4:
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.cols, image.rows, 0,
                        GL_BGRA, GL_UNSIGNED_BYTE, image.data));
      break;
    default:
      LOG_ERROR("invalid channel count " << image.channels()
                                         << " for color image " << _url);
    }
  }
  if (_mipmap) {
    V_GL(glGenerateMipmap(GL_TEXTURE_2D));
  }
  V_GL(glBindTexture(GL_TEXTURE_2D, 0));
  return _id;
}

GLuint Texture::update(int width, int height, int format, int samples) {
  create();
  if (_watcher.changed(width, height, format, samples)) {
    V_GL(glActiveTexture(GL_TEXTURE0));
    if (samples <= 0) {
      V_GL(glBindTexture(GL_TEXTURE_2D, _id));
      V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
      V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
      V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
      V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
      V_GL(glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, GL_BGRA,
                        GL_UNSIGNED_BYTE, nullptr));
    } else {
      V_GL(glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _id));
      V_GL(glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, format,
                                   width, height, true));
    }
  }
  return _id;
}

GLuint Texture::update() {
  if (!_url.empty() && invalidated()) {
    _loaded = false;
    _loader.clear();
    destroy();
  }
  if (!_loaded) {
    if (_url.empty()) {
      _loaded = true;
    } else {
      if (_loader.load()) {
        _loaded = true;
        if (auto image = _loader.load()) {
          if (image->image.rows) {
            update(image->image);
          }
        }
        _loader.clear();
      }
    }
  }
  return _id;
}

TextureManager &Texture::manager() {
  static TextureManager instance;
  return instance;
}
