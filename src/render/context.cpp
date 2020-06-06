// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "context.h"

#include "../core/log.h"

#include <stdexcept>

#define GL_GLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <GL/gl.h>
#include <GL/glext.h>

#define EGL_CHECK_ERROR(str)                                                   \
  {                                                                            \
    auto error = eglGetError();                                                \
    if (error != EGL_SUCCESS) {                                                \
      LOG_ERROR("EGL " << str << " failed with error " << error << std::hex    \
                       << " 0x" << error << std::dec);                         \
    }                                                                          \
  }

#define TRY_EGL(x)                                                             \
  {                                                                            \
    int ret = (x);                                                             \
    EGL_CHECK_ERROR(#x);                                                       \
    if (ret != EGL_TRUE) {                                                     \
      LOG_ERROR("EGL call " << #x << " failed");                               \
    }                                                                          \
  }

#define V_EGL(x)                                                               \
  {                                                                            \
    int ret = (x);                                                             \
    EGL_CHECK_ERROR(#x);                                                       \
    if (ret != EGL_TRUE) {                                                     \
      LOG_FATAL("EGL call " << #x << " failed");                               \
      throw std::runtime_error(#x);                                            \
    }                                                                          \
  }

#define V_EGL_PTR(x)                                                           \
  if (!(x)) {                                                                  \
    EGL_CHECK_ERROR(#x);                                                       \
    throw std::runtime_error("EGL call " #x " failed");                        \
  }

static EGLConfig chooseConfig(EGLDisplay egl_display) {
  EGLConfig egl_config;
  int samples = 16;
  for (; samples >= 0; samples--) {
    const EGLint attributes[] = {EGL_SURFACE_TYPE,
                                 EGL_PBUFFER_BIT,
                                 EGL_BLUE_SIZE,
                                 8,
                                 EGL_GREEN_SIZE,
                                 8,
                                 EGL_RED_SIZE,
                                 8,
                                 EGL_DEPTH_SIZE,
                                 24,
                                 EGL_SAMPLES,
                                 samples,
                                 EGL_RENDERABLE_TYPE,
                                 EGL_OPENGL_BIT,
                                 EGL_NONE};
    EGLint num_configs = 0;
    V_EGL(
        eglChooseConfig(egl_display, attributes, &egl_config, 1, &num_configs));
    if (num_configs > 0) {
      LOG_DEBUG("framebuffer samples " << samples);
      return egl_config;
    }
  }
  throw std::runtime_error("no compatible frame buffer configurations found");
}

GLDisplay::GLDisplay() {
  V_EGL_PTR(egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY));
  EGLint egl_version_major = 0, egl_version_minor = 0;
  V_EGL(eglInitialize(egl_display, &egl_version_major, &egl_version_minor));
  destructor = [this]() {
    if (egl_display) {
      V_EGL(eglTerminate(egl_display));
      egl_display = nullptr;
    }
  };
  V_EGL(eglBindAPI(EGL_OPENGL_API));
}

GLContext::GLContext(GLDisplay *display) : display(display) {
  EGLint attributes[] = {
      EGL_CONTEXT_MAJOR_VERSION,
      3,
      EGL_CONTEXT_MINOR_VERSION,
      2,
      EGL_CONTEXT_OPENGL_PROFILE_MASK,
      EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
      // EGL_CONTEXT_OPENGL_FORWARD_COMPATIBLE,
      // EGL_FALSE,
      // EGL_CONTEXT_OPENGL_DEBUG,
      // EGL_TRUE,
      EGL_NONE,
  };
  V_EGL_PTR(egl_context = eglCreateContext(display->egl_display,
                                           chooseConfig(display->egl_display),
                                           EGL_NO_CONTEXT, attributes));
}
GLContext::~GLContext() {
  if (egl_context) {
    V_EGL(eglDestroyContext(display->egl_display, egl_context));
    egl_context = nullptr;
  }
}

GLSurface::GLSurface(GLDisplay *display) : display(display) {
  int width = 32;
  int height = 32;
  EGLint attribs[] = {
      EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE,
  };
  V_EGL_PTR(
      egl_surface = eglCreatePbufferSurface(
          display->egl_display, chooseConfig(display->egl_display), attribs));
}
GLSurface::GLSurface(GLDisplay *display, long long window) : display(display) {
  EGLint attribs[] = {
      EGL_GL_COLORSPACE, EGL_GL_COLORSPACE_SRGB,
      EGL_RENDER_BUFFER, EGL_BACK_BUFFER,
      EGL_NONE,
  };
  V_EGL_PTR(egl_surface = eglCreateWindowSurface(
                display->egl_display, chooseConfig(display->egl_display),
                window, attribs));
}
GLSurface::~GLSurface() {
  if (egl_surface) {
    V_EGL(eglDestroySurface(display->egl_display, egl_surface));
    egl_surface = nullptr;
  }
}
void GLSurface::swap() {
  V_EGL(eglSwapInterval(display->egl_display, 0));
  V_EGL(eglSwapBuffers(display->egl_display, egl_surface));
}
/*int GLSurface::width() const {
  EGLint ret = 0;
  V_EGL(eglQuerySurface(display->egl_display, egl_surface, EGL_WIDTH, &ret));
  return ret;
}
int GLSurface::height() const {
  EGLint ret = 0;
  V_EGL(eglQuerySurface(display->egl_display, egl_surface, EGL_HEIGHT, &ret));
  return ret;
}*/

GLScope::GLScope(GLContext *context, GLSurface *surface)
    : display(context->display) {
  V_EGL(eglMakeCurrent(context->display->egl_display, surface->egl_surface,
                       surface->egl_surface, context->egl_context));
}
GLScope::~GLScope() {
  V_EGL(eglMakeCurrent(display->egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                       EGL_NO_CONTEXT));
}
