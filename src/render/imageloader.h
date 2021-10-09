#pragma once

#include "../core/log.h"
#include "resource.h"

#include <opencv2/opencv.hpp>

struct ImageLoader {
  std::string url;
  cv::Mat image;
  ImageLoader(const std::string &url) : url(url) {
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
