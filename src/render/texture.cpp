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
    auto img = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
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
  if (invalidated()) {
    _loaded = false;
    destroy();
  }
  cv::Mat image = img;
  if (image.channels() == 3) {
    cv::cvtColor(image, image, CV_RGB2RGBA);
  }
  cv::flip(image, image, 0);
  create();
  V_GL(glActiveTexture(GL_TEXTURE0));
  V_GL(glBindTexture(GL_TEXTURE_2D, _id));
  V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 4));
  V_GL(glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize()));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                       GL_LINEAR_MIPMAP_LINEAR));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
  V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
  {
    float max_anisotropic = 0.0f;
    V_GL(glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_anisotropic));
    // LOG_DEBUG("max_anisotropic " << max_anisotropic);
    float anisotropic = std::min(16.0f, max_anisotropic);
    V_GL(glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                         anisotropic));
  }
  if (_type == TextureType::Color) {
    V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, image.cols, image.rows,
                      0, GL_BGRA, GL_UNSIGNED_BYTE, image.data));
  }
  if (_type == TextureType::Normal) {
    V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.cols, image.rows, 0,
                      GL_BGRA, GL_UNSIGNED_BYTE, image.data));
  }
  V_GL(glGenerateMipmap(GL_TEXTURE_2D));
  V_GL(glBindTexture(GL_TEXTURE_2D, 0));
  return _id;
}

GLuint Texture::update() {
  if (invalidated()) {
    // LOG_INFO("reloading texture " << _url);
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
