// TAMSVIZ
// (c) 2020-2024 Philipp Ruppel

#pragma once

#include "message.h"

#include <opencv2/imgproc/imgproc.hpp>

class QImage;

bool isImageMessageTypeName(const std::string &message_type_name);

bool isImageMessage(const Message &image);

bool tryConvertImageMessageToMat(const Message &image, cv::Mat &mat,
                                 std::string &encoding,
                                 std_msgs::Header &header);

bool tryConvertMatToU8(cv::Mat &in, cv::Mat &out);

bool tryConvertMatToImage(const cv::Mat &mat, const std::string &img_encoding,
                          QImage &image);