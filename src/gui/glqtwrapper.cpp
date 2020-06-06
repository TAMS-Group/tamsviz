// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "glqtwrapper.h"

/*
#include <QtPlatformHeaders/QEGLNativeContext>

QOpenGLContext *egl2qt(void *egl_context, void *egl_display) {
  QOpenGLContext *wrapper = new QOpenGLContext();
  wrapper->setNativeHandle(
      QVariant::fromValue(QEGLNativeContext(egl_context, egl_display)));
  if (!wrapper->create()) {
    throw std::runtime_error("failed to create qt gl wrapper");
  }
  return wrapper;
}
*/
