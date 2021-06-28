// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "resource.h"

#include <mutex>

#include <resource_retriever/retriever.h>

#include "opengl.h"

ResourceEvents &ResourceEvents::instance() {
  static ResourceEvents instance;
  return instance;
}

static std::mutex g_cleanup_mutex;
static std::function<void(const std::function<void()> &)> g_cleanup_function =
    nullptr;

void ResourceBase::cleanup(const std::function<void()> &callback) {
  std::lock_guard<std::mutex> lock(g_cleanup_mutex);
  if (g_cleanup_function) {
    g_cleanup_function(callback);
  } else {
    callback();
  }
}

void ResourceBase::setCleanupFunction(
    const std::function<void(const std::function<void()> &)> &callback) {
  std::lock_guard<std::mutex> lock(g_cleanup_mutex);
  g_cleanup_function = callback;
}

static resource_retriever::Retriever &resourceRetreiver() {
  static thread_local resource_retriever::Retriever retriever;
  return retriever;
}

void loadResource(const std::string &url, std::string &data) {
  auto resource = resourceRetreiver().get(url);
  data.assign((const char *)resource.data.get(), resource.size);
}

void loadResource(const std::string &url, std::vector<uint8_t> &data) {
  auto resource = resourceRetreiver().get(url);
  data.assign((const uint8_t *)resource.data.get(),
              (const uint8_t *)resource.data.get() + resource.size);
}
