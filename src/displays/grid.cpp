// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "grid.h"

#include "../core/log.h"
#include "../render/mesh.h"

void GridDisplay::renderSync(const RenderSyncContext &context) {
  if (_watcher.changed(color(), cellSize(), cellCount(), lineWidth())) {
    LOG_DEBUG("update grid");
    {
      visualization_msgs::Marker marker;
      marker.id = 1;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.scale.x = lineWidth();
      marker.color.r = color().r();
      marker.color.g = color().g();
      marker.color.b = color().b();
      marker.color.a = color().a();
      double q = cellSize() * cellCount() * 0.5;
      for (size_t i = 0; i < cellCount() + 1; i++) {
        double p = (i - cellCount() * 0.5) * cellSize();
        marker.points.emplace_back();
        marker.points.back().x = p;
        marker.points.back().y = -q;
        marker.points.emplace_back();
        marker.points.back().x = p;
        marker.points.back().y = +q;
        marker.points.emplace_back();
        marker.points.back().x = -q;
        marker.points.back().y = p;
        marker.points.emplace_back();
        marker.points.back().x = +q;
        marker.points.back().y = p;
      }
      update(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.id = 2;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.scale.x = lineWidth();
      marker.color.r = color().r();
      marker.color.g = color().g();
      marker.color.b = color().b();
      marker.color.a = color().a();
      double q = cellSize() * cellCount() * 0.5;
      marker.points.emplace_back();
      marker.points.back().x = +q;
      marker.points.back().y = +q;
      marker.points.emplace_back();
      marker.points.back().x = -q;
      marker.points.back().y = +q;
      marker.points.emplace_back();
      marker.points.back().x = -q;
      marker.points.back().y = -q;
      marker.points.emplace_back();
      marker.points.back().x = +q;
      marker.points.back().y = -q;
      update(marker);
    }
  }
  MarkerFrameDisplayBase::renderSync(context);
}
