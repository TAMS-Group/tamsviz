// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "handle.h"

#include "object.h"

uint64_t handleObjectId(const Object *object) {
  return object ? object->id() : 0;
}
