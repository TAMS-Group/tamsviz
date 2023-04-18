// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "tracks.h"

#include "bagplayer.h"
#include "workspace.h"

std::shared_ptr<AnnotationBranch>
AnnotationTrack::branch(const std::shared_ptr<Workspace> &ws, bool create) {
  std::string name = "";
  if (ws->player) {
    name = ws->player->fileName();
  }
  for (auto &branch : branches()) {
    if (branch->name() == name) {
      return branch;
    }
  }
  if (create) {
    auto branch = std::make_shared<AnnotationBranch>();
    branch->name() = name;
    branches().push_back(branch);
    return branch;
  } else {
    return nullptr;
  }
}
