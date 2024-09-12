// TAMSVIZ
// (c) 2020-2024 Philipp Ruppel

#include "image.h"

#include "log.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <QtWidgets>

bool isImageMessageTypeName(const std::string &message_type_name) {
  return message_type_name == "sensor_msgs/Image" ||
         message_type_name == "sensor_msgs/CompressedImage";
}

bool isImageMessage(const Message &image) {
  return isImageMessageTypeName(image.type()->name());
}

bool tryConvertImageMessageToMat(const Message &image, cv::Mat &mat,
                                 std::string &encoding,
                                 std_msgs::Header &header) {
  if (!isImageMessage(image)) {
    LOG_WARN_THROTTLE(1, "not an image message " << image.type()->name());
    return false;
  }
  if (auto img = image.instantiate<sensor_msgs::Image>()) {
    header = img->header;
    if (sensor_msgs::image_encodings::isBayer(img->encoding) ||
        img->encoding == sensor_msgs::image_encodings::YUV422) {
      try {
        cv_bridge::CvImageConstPtr cv_ptr =
            cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::RGB8);
        mat = cv_ptr->image;
        encoding = cv_ptr->encoding;
        return true;
      } catch (cv_bridge::Exception &e) {
        LOG_ERROR("cv_bridge exception: " << e.what());
      }
    }
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*img);
      mat = cv_ptr->image;
      encoding = cv_ptr->encoding;
      return true;
    } catch (cv_bridge::Exception &e) {
      LOG_ERROR("cv_bridge exception: " << e.what());
    }
  } else if (auto img = image.instantiate<sensor_msgs::CompressedImage>()) {
    header = img->header;
    try {
      cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::RGB8);
      mat = cv_ptr->image;
      encoding = cv_ptr->encoding;
      return true;
    } catch (cv_bridge::Exception &e) {
      LOG_ERROR("cv_bridge exception: " << e.what());
    }
  } else {
    LOG_WARN("unsupported image message type" << image.type()->name());
  }
  if (mat.empty()) {
    LOG_WARN("failed to decode image");
  }
  header = std_msgs::Header();
  return false;
}

bool tryConvertMatToU8(const cv::Mat &in, cv::Mat &out) {
  switch (in.type()) {
    case CV_16UC1:
      in.convertTo(out, CV_8U, 1.0 / 256);
      break;
    case CV_32FC1:
      in.convertTo(out, CV_8U, 255.0);
      break;
    case CV_64FC1:
      in.convertTo(out, CV_8U, 255.0);
      break;
    case CV_16UC3:
      in.convertTo(out, CV_8UC3, 1.0 / 256.0);
      break;
    case CV_32FC3:
      in.convertTo(out, CV_8UC3, 255.0);
      break;
    case CV_64FC3:
      in.convertTo(out, CV_8UC3, 255.0);
      break;
    case CV_16UC4:
      in.convertTo(out, CV_8UC4, 1.0 / 256.0);
      break;
    case CV_32FC4:
      in.convertTo(out, CV_8UC4, 255.0);
      break;
    case CV_64FC4:
      in.convertTo(out, CV_8UC4, 255.0);
      break;
    default:
      out = in;
      break;
  }
  return CV_MAT_DEPTH(out.type()) == CV_8U;
}

bool tryConvertMatToImage(const cv::Mat &in_mat,
                          const std::string &img_encoding, QImage &image) {
  {
    cv::Mat mat;
    if (tryConvertMatToU8(in_mat, mat)) {
      switch (mat.type()) {
        case CV_8UC1:
          image = QImage((uchar *)mat.data, mat.cols, mat.rows, mat.step,
                         QImage::Format_Grayscale8);
          return true;
        case CV_8UC3:
          if (img_encoding == "bgr8") {
            cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
          }
          image = QImage((uchar *)mat.data, mat.cols, mat.rows, mat.step,
                         QImage::Format_RGB888);
          return true;
        case CV_8UC4:
          if (img_encoding == "bgra8") {
            cv::cvtColor(mat, mat, cv::COLOR_BGRA2RGBA);
          }
          image = QImage((uchar *)mat.data, mat.cols, mat.rows, mat.step,
                         QImage::Format_RGBA8888);
          return true;
      }
    }
  }
  LOG_WARN("cv to qt image format not yet supported " << in_mat.type());
  return false;
}
