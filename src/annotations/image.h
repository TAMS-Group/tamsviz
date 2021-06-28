// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "../core/document.h"

class QPainterPath;

struct ImageAnnotationBase : AnnotationBase {
protected:
  ImageAnnotationBase() {}

public:
  PROPERTY(std::vector<Eigen::Vector2d>, controlPoints);
  PROPERTY(std::string, topic);
  bool complete = false;
  std::shared_ptr<QPainterPath> shape;
  virtual void render() = 0;
  virtual void constrain() {}
};
DECLARE_TYPE(ImageAnnotationBase, AnnotationBase);
