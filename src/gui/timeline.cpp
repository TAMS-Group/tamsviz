// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "timeline.h"

#include "../annotations/image.h"
#include "../core/bagplayer.h"
#include "../core/log.h"
#include "../core/topic.h"
#include "../core/tracks.h"
#include "../core/workspace.h"
#include "../core/image.h"
#include "../core/destructor.h"

#include "mainwindow.h"

#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>

static const double track_height = 42;
static const double track_label_width = 200;
static const double track_padding_right = 100;

template <class FNC>
static void updateEnabled(QLayout *layout, const FNC &callback) {
  for (size_t i = 0; i < layout->count(); i++) {
    if (auto *widget = layout->itemAt(i)->widget()) {
      callback(widget);
    }
    if (auto *l = layout->itemAt(i)->layout()) {
      updateEnabled(l, callback);
    }
  }
}

static size_t trackCount(const std::shared_ptr<Workspace> &ws) {
  return (ws->document() && ws->document()->timeline())
             ? ws->document()->timeline()->tracks().size()
             : 0;
}

static double timelineDuration(const std::shared_ptr<Workspace> &ws) {
  return std::max(0.001, ws->player ? ws->player->duration() : 1.0) + 1.0;
}

static std::shared_ptr<TrackBase> trackAt(const std::shared_ptr<Workspace> &ws,
                                          size_t index) {
  if (ws->document() && ws->document()->timeline() &&
      index < ws->document()->timeline()->tracks().size()) {
    return ws->document()->timeline()->tracks()[index];
  } else {
    return nullptr;
  }
}

class ItemBase : public QObject {
  bool ok = true;

 public:
  virtual void sync(const std::shared_ptr<Workspace> &ws) {}
  ItemBase() {
    LockScope()->modified.connect(this, [this]() {
      if (!ok) {
        throw std::runtime_error("timeline item already destroyed");
      }
      sync(LockScope().ws());
    });
  }
  ~ItemBase() { ok = false; }
};

class EditableText : public QGraphicsRectItem, public QObject {
  QString _text;
  QGraphicsProxyWidget *_proxy = nullptr;
  QLineEdit *_edit = nullptr;
  QMargins margins() const {
    int m = std::round(std::min(rect().width(), rect().height()) * 0.25);
    if (_alignment & Qt::AlignHCenter) {
      m = 1;
    }
    return QMargins(m, 0, m, 0);
  }
  QPen _textPen = QPen(QApplication::palette().text(), 1);
  Qt::Alignment _alignment = (Qt::AlignLeft | Qt::AlignVCenter);
  QFont _font = QApplication::font();

 public:
  EditableText(QGraphicsItem *parent = nullptr) : QGraphicsRectItem(parent) {
    setPen(QPen(Qt::NoPen));
  }
  void setFont(const QFont &font) { _font = font; }
  Event<void(const std::string &)> changed;
  std::string text() const { return _text.toStdString(); }
  void setText(const std::string &t) {
    _text = t.c_str();
    update();
  }
  void setTextAlignment(Qt::Alignment a) {
    _alignment = a;
    update();
  }
  void setTextPen(const QPen &pen) {
    _textPen = pen;
    update();
  }
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {
    if (_proxy) {
      return;
    }
    painter->save();
    QGraphicsRectItem::paint(painter, option, widget);
    painter->setFont(_font);
    painter->setPen(_textPen);
    painter->drawText(rect().marginsRemoved(margins()), _text,
                      QTextOption(_alignment));
    painter->restore();
  }
  virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override {
    if (!_edit) {
      scene()->views().front()->setFocusPolicy(Qt::StrongFocus);
      scene()->views().front()->setFocus();
      _edit = new QLineEdit();
      _edit->setFixedSize(rect().width(), rect().height());
      _edit->setAlignment(_alignment);
      _edit->setTextMargins(margins());
      _edit->setText(_text);
      connect(_edit, &QLineEdit::editingFinished, this, [this]() {
        if (_text != _edit->text()) {
          _text = _edit->text();
          LOG_DEBUG("text changed " << text());
          changed(text());
        } else {
          LOG_DEBUG("text unchanged");
        }
        scene()->views().front()->setFocusPolicy(Qt::NoFocus);
        _edit->clearFocus();
        _proxy->clearFocus();
        _edit->deleteLater();
        _edit = nullptr;
        _proxy->deleteLater();
        _proxy = nullptr;
        scene()->clearFocus();
        scene()->views().front()->window()->setFocus();
      });
      _proxy = new QGraphicsProxyWidget(this);
      _proxy->setWidget(_edit);
      _proxy->setPos(rect().x(), rect().y());
      _edit->setFocus(Qt::MouseFocusReason);
      _edit->selectAll();
    }
  }
};

class AnnotationSpanItem : public EditableText {
  class ResizeHandle : public QGraphicsRectItem {
    AnnotationSpanItem *_parent = nullptr;
    int _side = 0;
    double _drag_offset = 0.0;
    QRectF _drag_start_rect;

   public:
    double timeToPosition(double t) {
      LockScope ws;
      double bag_duration = _parent->bag_duration;
      double track_length = _parent->track_length;
      return t * track_length / bag_duration;
    }
    double positionToTime(double p) {
      LockScope ws;
      double bag_duration = _parent->bag_duration;
      double track_length = _parent->track_length;
      return p / track_length * bag_duration;
    }
    ResizeHandle(AnnotationSpanItem *parent, int side)
        : _parent(parent), _side(side) {
      setCursor(Qt::SizeHorCursor);
      setParentItem(parent);
      setBrush(QBrush(Qt::transparent));
      setPen(QPen(Qt::NoPen));
    }
    void update() {
      auto rect = _parent->rect();
      double w = std::min(5.0, rect.width() * 0.25);
      if (_side < 0) {
        rect.setRight(rect.left() + w);
      }
      if (_side > 0) {
        rect.setLeft(rect.right() - w);
      }
      setRect(rect);
      setCursor(Qt::SizeHorCursor);
    }
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
      if (event->button() == Qt::LeftButton) {
        event->accept();
        LOG_DEBUG("begin resize span");
        _drag_start_rect = rect();
        if (_side < 0) {
          _drag_offset = event->scenePos().x() - _parent->rect().left();
        }
        if (_side > 0) {
          _drag_offset = event->scenePos().x() - _parent->rect().right();
        }
      }
    }
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
      if (event->button() == Qt::LeftButton) {
        if (_drag_start_rect != rect()) {
          if (rect().width() < 2.5) {
            ActionScope ws("Delete annotation");
            auto &spans = _parent->_track->branch(ws(), true)->spans();
            spans.erase(
                std::remove(spans.begin(), spans.end(), _parent->_annotation),
                spans.end());
            ws->modified();
          } else {
            ActionScope ws("Resize Annotation Span");
            _parent->commit();
            ws->modified();
          }
        }
      }
    }
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
      if (event->buttons() == Qt::LeftButton) {
        event->accept();
        LOG_DEBUG("resize span");
        auto rect = _parent->rect();
        LockScope ws;
        if (_side < 0) {
          double x = event->scenePos().x() - _drag_offset;
          _parent->snap(x);
          if (auto branch = _parent->_track->branch(ws(), false)) {
            for (auto &span : branch->spans()) {
              if (timeToPosition(span->start() + span->duration()) <
                  _drag_start_rect.center().x()) {
                x = std::max(x,
                             timeToPosition(span->start() + span->duration()));
              }
            }
          }
          rect.setLeft(std::max(0.0, std::min(rect.right(), x)));
        }
        if (_side > 0) {
          double x = event->scenePos().x() - _drag_offset;
          _parent->snap(x);
          if (auto branch = _parent->_track->branch(ws(), false)) {
            for (auto &span : branch->spans()) {
              if (timeToPosition(span->start()) >
                  _drag_start_rect.center().x()) {
                x = std::min(x, timeToPosition(span->start()));
              }
            }
          }
          rect.setRight(std::min(scene()->sceneRect().width() -
                                     track_label_width - track_padding_right,
                                 std::max(rect.left(), x)));
        }
        _parent->setItemRect(rect.x(), rect.y(), rect.width());
        update();
      }
    }
  };

  std::shared_ptr<AnnotationTrack> _track;
  std::shared_ptr<AnnotationSpan> _annotation;
  double track_length = 0;
  double bag_duration = 0.0;
  double timeToPosition(double t) { return t * track_length / bag_duration; }
  double positionToTime(double p) { return p / track_length * bag_duration; }
  QRectF _drag_start_rect;
  ResizeHandle *left_handle = new ResizeHandle(this, -1);
  ResizeHandle *right_handle = new ResizeHandle(this, +1);
  bool _dragged = false;
  std::unordered_set<std::shared_ptr<AnnotationSpan>> _dragged_spans;
  bool _selected = false;
  double _track_color = 0;

  bool snap(double &x) {
    return snap(x, [this](const std::shared_ptr<AnnotationSpan> &span) {
      return span != _annotation;
    });
  }
  template <class F>
  bool snap(double &x, const F &filter) {
    double snap_position = 0;
    bool snapped = false;
    LockScope ws;
    if (ws->player) {
      double l = timeToPosition(ws->player->time());
      if (!snapped || std::abs(x - l) < std::abs(x - snap_position)) {
        snap_position = l;
        snapped = true;
      }
    }
    for (size_t itrack = 0; itrack < trackCount(ws()); itrack++) {
      if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
              trackAt(ws(), itrack))) {
        if (auto branch = track->branch(ws(), false)) {
          for (auto &other_span : branch->spans()) {
            if (!filter(other_span)) {
              continue;
            }
            double l = timeToPosition(other_span->start());
            double r =
                timeToPosition(other_span->start() + other_span->duration());
            if (!snapped || std::abs(x - l) < std::abs(x - snap_position)) {
              snap_position = l;
              snapped = true;
            }
            if (!snapped || std::abs(x - r) < std::abs(x - snap_position)) {
              snap_position = r;
              snapped = true;
            }
          }
        }
      }
    }
    if (std::abs(snap_position - x) < 6.1) {
      x = snap_position;
      return true;
    } else {
      return false;
    }
  }

  static std::unordered_set<AnnotationSpanItem *> &instances() {
    static std::unordered_set<AnnotationSpanItem *> instances;
    return instances;
  }

  QPointF &last_clicked_point = *[]() {
    static QPointF s;
    return &s;
  }();

 public:
  AnnotationSpanItem() {
    setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

    setTextPen(QPen(QBrush(QColor(0, 0, 0)), 1));

    changed.connect([this](const std::string &label) {
      ActionScope ws("Change annotation span label");
      if (label == _track->label()) {
        _annotation->label().clear();
      } else {
        _annotation->label() = label;
      }
      ws->modified();
    });

    instances().insert(this);
  }
  ~AnnotationSpanItem() { instances().erase(this); }

  void setItemRect(double x, double y, double width) {
    setRect(x, y, width, track_height);
    left_handle->update();
    right_handle->update();

    {
      LockScope ws;
      size_t itrack = std::max(0.0, std::round(y / track_height) - 1.0);
      if (auto track = trackAt(ws(), itrack)) {
        setTextPen(QPen(QBrush(QColor(0, 0, 0)), 1));
        QFont font = QApplication::font();
        if (_annotation->label().empty()) {
          setText(track->label());
          font.setItalic(true);
        } else {
          setText(_annotation->label());
        }
        _track_color = track->color();
        if (_selected = ws->selection().contains(_annotation)) {
          setBrush(QBrush(QColor::fromHsvF(track->color(), 0.8, 0.6)));
          setTextPen(QPen(QBrush(QColor(Qt::white)), 1));
        } else {
          setBrush(QBrush(QColor::fromHsvF(track->color(), 0.2, 1.0)));
          setTextPen(QPen(QBrush(QColor(Qt::black)), 1));
        }
        setPen(QPen(QBrush(Qt::black), 1));
        setFont(font);
      }
    }
  }
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {
    EditableText::paint(painter, option, widget);
    painter->setBrush(QBrush(Qt::transparent));
    painter->setPen(QPen(QBrush(QColor(255, 255, 255, 200)), 1, Qt::SolidLine,
                         Qt::SquareCap, Qt::MiterJoin));
    painter->drawRect(rect().marginsRemoved(QMarginsF(1, 1, 1, 1)));
  }
  void update(const std::shared_ptr<Workspace> &ws, size_t track_index,
              const std::shared_ptr<AnnotationTrack> &track,
              const std::shared_ptr<AnnotationSpan> &annotation,
              double track_length, double bag_duration) {
    _track = track;
    _annotation = annotation;
    this->track_length = track_length;
    this->bag_duration = bag_duration;
    setItemRect(track_length * annotation->start() / bag_duration,
                (track_index + 1) * track_height,
                track_length * annotation->duration() / bag_duration);
  }
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsRectItem::mousePressEvent(event);
    LockScope ws;
    if (event->button() == Qt::LeftButton) {
      LOG_DEBUG("begin dragging annotation span");
      _dragged = false;
      event->accept();
      {
        _dragged_spans.clear();
        LockScope ws;
        LOG_DEBUG("select " << _annotation);
        ws->currentAnnotationTrack() = _track;
        if (event->modifiers() & Qt::ShiftModifier) {
          if (!last_clicked_point.isNull()) {
            double left =
                std::min(last_clicked_point.x(), event->scenePos().x());
            double right =
                std::max(last_clicked_point.x(), event->scenePos().x());
            double top =
                std::min(std::floor(last_clicked_point.y() / track_height) *
                             track_height,
                         rect().top());
            double bottom = std::max(
                std::ceil(last_clicked_point.y() / track_height) * track_height,
                rect().bottom());
            for (auto *instance : instances()) {
              if (instance->rect().right() - 1e-6 > left &&
                  instance->rect().left() + 1e-6 < right &&
                  instance->rect().center().y() >= top &&
                  instance->rect().center().y() <= bottom) {
                ws->selection().add(instance->_annotation);
              }
            }
            ws->selection().add(_annotation);
          } else {
            ws->selection().toggle(_annotation);
            if (ws->selection().contains(_annotation)) {
              last_clicked_point = event->scenePos();
            } else {
              last_clicked_point = QPointF();
            }
          }
        } else if (event->modifiers() & Qt::ControlModifier) {
          ws->selection().toggle(_annotation);
          if (ws->selection().contains(_annotation)) {
            last_clicked_point = event->scenePos();
          } else {
            last_clicked_point = QPointF();
          }
        } else {
          if (!ws->selection().contains(_annotation)) {
            ws->selection() = _annotation;
            last_clicked_point = event->scenePos();
          }
          for (auto object : ws->selection().resolve(ws())) {
            if (auto span = std::dynamic_pointer_cast<AnnotationSpan>(object)) {
              _dragged_spans.insert(span);
            }
          }
          for (auto &view : instances()) {
            if (_dragged_spans.find(view->_annotation) !=
                _dragged_spans.end()) {
              view->_drag_start_rect = view->rect();
            }
          }
        }
        left_handle->unsetCursor();
        right_handle->unsetCursor();
        ws->modified();
      }
    }
  }
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsRectItem::mouseMoveEvent(event);
    if (event->buttons() == Qt::LeftButton) {
      if (!_dragged_spans.empty()) {
        LockScope ws;
        _dragged = true;
        double timeline_duration = timelineDuration(ws());
        double max_x = timeToPosition(timeline_duration);
        size_t track_count = trackCount(ws());
        double dx = event->scenePos().x() -
                    event->buttonDownScenePos(Qt::LeftButton).x();
        double di = std::round((event->scenePos().y() -
                                event->buttonDownScenePos(Qt::LeftButton).y()) /
                               track_height);
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            double vi =
                std::round(view->_drag_start_rect.y() / track_height) - 1;
            di = std::min(track_count - 1.0, std::max(0.0, di + vi)) - vi;
          }
        }
        {
          bool snapped = false;
          double best_snap = 0;
          for (auto *view : instances()) {
            if (_dragged_spans.find(view->_annotation) !=
                _dragged_spans.end()) {
              for (double offset : {0.0, view->rect().width()}) {
                double side = view->_drag_start_rect.x() + offset + dx;
                double x = side;
                if (snap(x, [this](
                                const std::shared_ptr<AnnotationSpan> &span) {
                      return _dragged_spans.find(span) == _dragged_spans.end();
                    })) {
                  x -= side;
                  if (!snapped || std::abs(x) < std::abs(best_snap)) {
                    snapped = true;
                    best_snap = x;
                  }
                }
              }
            }
          }
          dx += best_snap;
        }
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            dx = std::max(dx, -view->_drag_start_rect.x());
            dx = std::min(max_x - view->_drag_start_rect.right(), dx);
          }
        }
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            for (auto *other : instances()) {
              if (_dragged_spans.find(other->_annotation) ==
                  _dragged_spans.end()) {
                if ((std::round(view->_drag_start_rect.y() / track_height) +
                     di) == std::round(other->rect().y() / track_height)) {
                  double min_distance =
                      (view->rect().width() + other->rect().width()) * 0.5;
                  double distance = (view->_drag_start_rect.center().x() + dx) -
                                    other->rect().center().x();
                  if (std::abs(distance) < min_distance) {
                    dx += (std::signbit(distance) ? -1.0 : 1.0) * min_distance -
                          distance;
                  }
                }
              }
            }
          }
        }
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            for (auto *other : instances()) {
              if (_dragged_spans.find(other->_annotation) ==
                  _dragged_spans.end()) {
                if ((std::round(view->_drag_start_rect.y() / track_height) +
                     di) == std::round(other->rect().y() / track_height)) {
                  double min_distance =
                      (view->rect().width() + other->rect().width()) * 0.5;
                  double distance = (view->_drag_start_rect.center().x() + dx) -
                                    other->rect().center().x();
                  if (std::abs(distance) < min_distance - 1e-6) {
                    return;
                  }
                }
              }
            }
          }
        }
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            if (view->_drag_start_rect.x() + dx < -1e-6) {
              return;
            }
            if (view->_drag_start_rect.right() + dx > max_x + 1e-6) {
              return;
            }
          }
        }
        for (auto *view : instances()) {
          if (_dragged_spans.find(view->_annotation) != _dragged_spans.end()) {
            auto rect = view->rect();
            view->setItemRect(view->_drag_start_rect.x() + dx,
                              view->_drag_start_rect.y() + di * track_height,
                              rect.width());
          }
        }
      }
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsRectItem::mouseReleaseEvent(event);
    if (instances().find(this) == instances().end()) {
      throw std::runtime_error("already deleted");
    }
    LOG_DEBUG("mouse release event " << this);
    if (event->button() == Qt::LeftButton) {
      if (!_dragged_spans.empty() && _dragged) {
        LOG_DEBUG("finished dragging annotation span");
        event->accept();
        {
          ActionScope ws("Drag Annotation Spans");
          std::vector<QPointer<AnnotationSpanItem>> insts;
          for (auto &view : instances()) {
            insts.emplace_back(view);
          }
          LOG_DEBUG("count " << insts.size());
          for (auto &view : insts) {
            if (view) {
              if (_dragged_spans.find(view->_annotation) !=
                  _dragged_spans.end()) {
                LOG_DEBUG("commit");
                view->commit();
              }
            } else {
              LOG_DEBUG("view gone");
            }
          }
          QTimer::singleShot(0, []() {
            LockScope ws;
            ws->modified();
          });
        }
        _dragged_spans.clear();
      } else {
        if (event->modifiers() == 0) {
          LockScope ws;
          ws->selection() = _annotation;
          ws->modified();
          last_clicked_point = event->buttonDownPos(Qt::LeftButton);
        }
      }
    }
  }

  void commit() {
    LockScope ws;
    if (auto annotation = _annotation) {
      double start = positionToTime(rect().x());
      double duration = positionToTime(rect().width());
      ssize_t itrack =
          std::max(0.0, std::round(rect().y() / track_height) - 1.0);
      if (annotation->start() != start || annotation->duration() != duration ||
          trackAt(ws(), itrack) != _track) {
        if (itrack >= 0 && itrack < trackCount(ws())) {
          // ActionScope ws("Move annotation");
          if (trackAt(ws(), itrack) != _track) {
            auto &spans = _track->branch(ws(), true)->spans();
            spans.erase(std::remove(spans.begin(), spans.end(), _annotation),
                        spans.end());
            if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                    trackAt(ws(), itrack))) {
              track->branch(ws(), true)->spans().push_back(_annotation);
            }
          }
          annotation->start() = start;
          annotation->duration() = duration;
        } else {
          LOG_ERROR("failed to move annotation");
        }
      } else {
        LOG_DEBUG("annotation not moved");
      }
    }
  }
};

class TrackViewBase : public QGraphicsRectItem, public QObject {
  class LabelItem : public EditableText {
    TrackViewBase *_parent = nullptr;
    QGraphicsRectItem *_reorder_marker = new QGraphicsRectItem(this);

   protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
      static std::weak_ptr<TrackBase> last_clicked_track;
      EditableText::mousePressEvent(event);
      if (event->button() == Qt::LeftButton) {
        event->accept();
        LockScope ws;
        LOG_DEBUG("select " << _parent->_track->label());
        if (event->modifiers() & Qt::ControlModifier) {
          ws->selection().toggle(_parent->_track);
        } else if (event->modifiers() & Qt::ShiftModifier) {
          if (auto last_clicked = last_clicked_track.lock()) {
            size_t track_count = trackCount(ws());
            bool selection_flag = false;
            for (size_t i = 0; i < track_count; i++) {
              auto track = trackAt(ws(), i);
              bool range_boundary =
                  (track == last_clicked || track == _parent->_track);
              if (selection_flag || range_boundary) {
                ws->selection().add(track);
              }
              if (range_boundary) {
                selection_flag = !selection_flag;
              }
            }
          } else {
            ws->selection().toggle(_parent->_track);
          }
        } else {
          ws->selection() = _parent->_track;
        }
        if (auto annotation_track =
                std::dynamic_pointer_cast<AnnotationTrack>(_parent->_track)) {
          if (ws->selection().contains(annotation_track)) {
            ws->currentAnnotationTrack() = annotation_track;
            last_clicked_track = annotation_track;
          }
        }
        ws->modified();
        //_reorder_marker->setBrush(
        //    QBrush(QColor::fromHsvF(_parent->_track->color(), 1.0, 0.5)));
        _reorder_marker->setBrush(QBrush(QColor(100, 100, 100)));
        _reorder_marker->setPen(QPen(Qt::NoPen));
      }
    }
    size_t y2i(double y) {
      LockScope ws;
      return std::max((int64_t)0,
                      std::min((int64_t)trackCount(ws()),
                               (int64_t)std::round(y / track_height) - 1));
    }
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
      EditableText::mouseMoveEvent(event);
      event->accept();
      if (event->buttons() == Qt::LeftButton) {
        LOG_DEBUG("drag track");
        double thickness = 6;
        double y =
            (y2i(event->scenePos().y()) + 1) * track_height - thickness * 0.5;
        y = std::max(y, track_height);
        _reorder_marker->setRect(rect().x(), y, rect().width(), thickness);
        _reorder_marker->show();
      }
    }
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
      EditableText::mouseReleaseEvent(event);
      event->accept();
      _reorder_marker->hide();
      if (event->button() == Qt::LeftButton &&
          event->buttonDownScenePos(Qt::LeftButton).y() !=
              event->scenePos().y()) {
        LockScope ws;
        size_t new_index = y2i(event->scenePos().y());
        LOG_DEBUG("dragged from " << _parent->_track_index << " to "
                                  << new_index);
        if (new_index != _parent->_track_index) {
          ActionScope ws("Reorder Tracks");
          auto &tracks = ws->document()->timeline()->tracks();
          auto new_position_it =
              tracks.insert(tracks.begin() + new_index, _parent->_track);
          for (auto it = tracks.begin(); it < tracks.end(); it++) {
            if (*it == _parent->_track && it != new_position_it) {
              tracks.erase(it);
              break;
            }
          }
          QTimer::singleShot(0, []() {
            LockScope ws;
            ws->modified();
          });
        }
      }
    }

   public:
    LabelItem(TrackViewBase *parent) : EditableText(parent), _parent(parent) {}
    virtual void paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) override {
      EditableText::paint(painter, option, widget);
      painter->setBrush(QBrush(Qt::transparent));
      painter->setPen(QPen(QBrush(QColor(255, 255, 255, 200)), 1, Qt::SolidLine,
                           Qt::SquareCap, Qt::MiterJoin));
      painter->drawRect(rect().marginsRemoved(QMarginsF(1, 1, 1, 1)));
    }
  };
  LabelItem *_text = nullptr;
  std::shared_ptr<TrackBase> _track;
  bool _selected = false;

 private:
  double scrollPositionX() const {
    auto *view = scene()->views().first();
    QPointF p = view->mapToScene(view->viewport()->rect().topLeft());
    return p.x();
  }

 protected:
  const size_t _track_index = 0;

 public:
  std::shared_ptr<TrackBase> track() const { return _track; }
  TrackViewBase(size_t track_index, std::shared_ptr<TrackBase> track)
      : _track(track), _track_index(track_index) {
    setBrush(QBrush(Qt::transparent));
    setPen(QPen(Qt::NoPen));

    _text = new LabelItem(this);
    _text->changed.connect(this, [track](const std::string &str) {
      ActionScope ws("Change annotation track label");
      track->label() = str;
      ws->modified();
    });
    _text->setZValue(100000);
  }
  virtual void update(const std::shared_ptr<Workspace> &ws, double width) {
    QRectF rect = scene()->sceneRect();
    double y = track_height * (_track_index + 1);
    setRect(QRectF(scrollPositionX() + track_label_width, y,
                   rect.width() - track_padding_right - track_label_width,
                   track_height));
    _text->setRect(scrollPositionX() - 1, y, track_label_width + 1,
                   track_height);
    setZValue(ws->selection().contains(_track) ? 2 : 1);
    _text->setText(_track->label().c_str());
    if (_selected = ws->selection().contains(_track)) {
      _text->setZValue(100002);
      _text->setBrush(QBrush(QColor::fromHsvF(_track->color(), 0.8, 0.6)));
      _text->setPen(QPen(QApplication::palette().brush(QPalette::Mid), 1));
      _text->setTextPen(QPen(QBrush(Qt::white), 1));

    } else {
      _text->setBrush(QBrush(QColor::fromHsvF(_track->color(), 0.2, 1)));
      _text->setTextPen(QPen(QBrush(Qt::black), 1));
      _text->setPen(QPen(QApplication::palette().brush(QPalette::Mid), 1));
      _text->setZValue(100001);
    }
    {
      QFont font = QApplication::font();
      if (_track->id() == ws->currentAnnotationTrack().id()) {
        font.setBold(true);
      }
      _text->setFont(font);
    }
  }
};

class GraphTree {
  struct Item {
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
    void merge(const Item &other) {
      xmin = std::min(xmin, other.xmin);
      ymin = std::min(ymin, other.ymin);
      xmax = std::max(xmax, other.xmax);
      ymax = std::max(ymax, other.ymax);
    }
  };
  struct Level {
    std::vector<Item> items;
  };
  struct Data {
    std::deque<Level> levels;
  };
  std::shared_ptr<Data> _data;

 public:
  struct Sample {
    double x = 0;
    double ymin = 0;
    double ymax = 0;
  };

 private:
  void sample(size_t ilevel, size_t iitem, double xmin, double xmax,
              double xstep, std::vector<Sample> &samples) {
    auto &item = _data->levels[ilevel].items[iitem];
    double item_end = item.xmax;
    if (iitem + 1 < _data->levels[ilevel].items.size()) {
      auto &item2 = _data->levels[ilevel].items[iitem + 1];
      item_end = item2.xmin;
    }
    double prev_x = item.xmin;
    if (iitem > 0) {
      prev_x = _data->levels[ilevel].items[iitem - 1].xmax;
    }
    if (item_end < xmin || prev_x > xmax) {
      return;
    }
    if (ilevel >= 1 && (item_end - item.xmin) > xstep) {
      sample(ilevel - 1, iitem * 2, xmin, xmax, xstep, samples);
      if (iitem * 2 + 1 < _data->levels[ilevel - 1].items.size()) {
        sample(ilevel - 1, iitem * 2 + 1, xmin, xmax, xstep, samples);
      }
    } else {
      Sample sample;
      sample.x = (item.xmin + item.xmax) / 2;
      sample.ymin = item.ymin;
      sample.ymax = item.ymax;
      samples.push_back(sample);
    }
  }

 public:
  void clear() { _data.reset(); }
  void push(double x, double y) {
    // LOG_DEBUG("push " << x << " " << y);
    if (!_data) {
      _data = std::make_shared<Data>();
    }
    if (_data.use_count() > 1) {
      _data = std::make_shared<Data>(*_data);
    }
    {
      Item item;
      item.xmin = item.xmax = x;
      item.ymin = item.ymax = y;
      if (_data->levels.empty()) {
        _data->levels.emplace_back();
      }
      _data->levels.front().items.push_back(item);
    }
    for (size_t ilevel = 1;; ilevel++) {
      if (_data->levels[ilevel - 1].items.size() <= 1) {
        break;
      }
      if (_data->levels.size() <= ilevel) {
        _data->levels.emplace_back();
      }
      size_t iitem = (_data->levels[ilevel - 1].items.size() - 1) / 2;
      _data->levels[ilevel].items.resize(iitem + 1);
      _data->levels[ilevel].items[iitem] =
          _data->levels[ilevel - 1].items[iitem * 2];
      if (iitem * 2 + 1 < _data->levels[ilevel - 1].items.size()) {
        _data->levels[ilevel].items[iitem].merge(
            _data->levels[ilevel - 1].items[iitem * 2 + 1]);
      }
      // LOG_DEBUG("level " << ilevel << " "
      //                     << _data->levels[ilevel].items.size());
      if (ilevel > 100) {
        throw std::runtime_error("graph tree overflow");
      }
    }
  }
  void sample(double xmin, double xmax, double xstep,
              std::vector<Sample> &samples) {
    if (_data && !_data->levels.empty()) {
      sample(_data->levels.size() - 1, 0, xmin, xmax, xstep, samples);
    }
  }
  void range(double &ymin, double &ymax) {
    if (_data && !_data->levels.empty()) {
      ymin = _data->levels.back().items.front().ymin;
      ymax = _data->levels.back().items.front().ymax;
    }
  }
};

class GraphTrackView : public TrackViewBase {
  std::shared_ptr<GraphTrack> _track;
  size_t _track_height = 0;
  double _track_length = 0;
  double _bag_duration = 0.0;
  QPen _pen;
  double timeToPosition(double t) { return t * _track_length / _bag_duration; }
  double positionToTime(double p) { return p / _track_length * _bag_duration; }
  volatile bool _exit_flag = false;
  std::mutex _load_mutex;
  GraphTree _load_data;
  std::string _load_query, _load_topic;
  double _load_duration = 0;
  std::condition_variable _load_condition;
  std::shared_ptr<BagPlayer> _load_player;
  Watcher _watcher;
  bool _load_flag = false;
  void _update() {
    QObject o;
    QObject::connect(&o, &QObject::destroyed, this,
                     [this](QObject *o) { LockScope()->modified(); });
  }
  std::thread _load_thread{[this]() {
    while (true) {
      std::string topic;
      MessageQuery query;
      MessageParser parser;
      std::shared_ptr<BagPlayer> player;
      double duration = 0;
      {
        std::unique_lock<std::mutex> lock(_load_mutex);
        while (true) {
          if (_exit_flag) {
            LOG_DEBUG("exit");
            return;
          }
          if (_load_flag) {
            _load_flag = false;
            topic = _load_topic;
            duration = _load_duration;
            query = MessageQuery(_load_query);
            player = _load_player;
            _load_data.clear();
            _update();
            if (topic.empty() || query.str().empty() || player == nullptr) {
              LOG_DEBUG("empty");
              continue;
            } else {
              break;
            }
          }
          LOG_DEBUG("wait");
          _load_condition.wait(lock);
        }
      }
      LOG_DEBUG("load graph track data");
      {
        LOG_DEBUG("begin loading graph track data");
        GraphTree data;
        auto t0 = std::chrono::steady_clock::now();
        player->readMessageSamples(
            topic, 0, duration, [&](const std::shared_ptr<const Message> &msg) {
              // LOG_DEBUG("a");
              parser.parse(msg);
              double x = (msg->time() - player->startTime()).toSec() / duration;
              double y = query(parser).toDouble();
              data.push(x, y);
              if (std::chrono::steady_clock::now() >
                  t0 + std::chrono::milliseconds(250)) {
                {
                  std::unique_lock<std::mutex> lock(_load_mutex);
                  _load_data = data;
                }
                _update();
                t0 = std::chrono::steady_clock::now();
              }
              return !_exit_flag && !_load_flag;
            });
        // LOG_DEBUG("b");
        {
          {
            std::unique_lock<std::mutex> lock(_load_mutex);
            _load_data = data;
          }
          _update();
        }
        LOG_DEBUG("graph track data loaded");
      }
    }
  }};

 public:
  GraphTrackView(size_t track_index, std::shared_ptr<GraphTrack> track)
      : TrackViewBase(track_index, track),
        _track_height(track_index),
        _track(track) {}
  ~GraphTrackView() {
    LOG_DEBUG("stop");
    {
      std::unique_lock<std::mutex> lock(_load_mutex);
      _exit_flag = true;
      _load_condition.notify_all();
    }
    LOG_DEBUG("join");
    _load_thread.join();
    LOG_DEBUG("ready");
  }
  virtual void update(const std::shared_ptr<Workspace> &ws,
                      double width) override {
    TrackViewBase::update(ws, width);
    _track_length = width;
    _bag_duration = timelineDuration(ws);
    _pen = QPen(QBrush(QColor::fromHsvF(_track->color(), 1.0, 0.5)), 1.5,
                Qt::SolidLine, Qt::SquareCap, Qt::RoundJoin);
    if (_watcher.changed(ws->player, _bag_duration, _track->query().str(),
                         _track->query().str())) {
      LOG_DEBUG("load graph track");
      std::unique_lock<std::mutex> lock(_load_mutex);
      _load_flag = true;
      _load_player = ws->player;
      _load_duration = _bag_duration;
      _load_query = _track->query().str();
      _load_topic = _track->topic().topic();
      _load_condition.notify_all();
    }
    // LOG_DEBUG(__LINE__);
  }
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {
    QPolygonF polygon;
    {
      GraphTree data;
      {
        std::unique_lock<std::mutex> lock(_load_mutex);
        data = _load_data;
      }
      std::vector<GraphTree::Sample> samples;
      auto *view = scene()->views().first();
      auto view_top_left = view->mapToScene(view->viewport()->rect().topLeft());
      auto view_top_right =
          view->mapToScene(view->viewport()->rect().topRight());
      data.sample(view_top_left.x() / _track_length,
                  view_top_right.x() / _track_length, 1.5 / _track_length,
                  samples);
      LOG_DEBUG("graph track samples " << samples.size());
      if (!samples.empty()) {
        QVector<QPointF> points;
        /*
        double lo = samples.front().ymin;
        double hi = samples.front().ymax;
        for (auto &sample : samples) {
          lo = std::min(lo, sample.ymin);
          hi = std::max(hi, sample.ymax);
        }
        */
        double hi = 0, lo = 0;
        data.range(lo, hi);
        for (size_t isample = 0; isample < samples.size(); isample++) {
          auto &sample = samples[isample];
          points.push_back(QPointF(
              sample.x * _track_length,
              (0.9 - (sample.ymax - lo) / (hi - lo) * 0.8) * track_height +
                  track_height + _track_index * track_height));
        }
        for (ssize_t isample = ssize_t(samples.size()) - 1; isample >= 0;
             isample--) {
          auto &sample = samples[isample];
          points.push_back(QPointF(
              sample.x * _track_length,
              (0.9 - (sample.ymin - lo) / (hi - lo) * 0.8) * track_height +
                  track_height + _track_index * track_height));
        }
        polygon = QPolygonF(points);
      }
    }
    painter->save();
    painter->setRenderHints(QPainter::Antialiasing);
    painter->setPen(_pen);
    // painter->setBrush(QBrush(Qt::transparent));
    painter->setBrush(_pen.brush());
    // painter->drawPolyline(polygon);
    painter->drawPolygon(polygon);
    painter->restore();
  }
  /*
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {
    QPolygonF polygon;
    {
      std::unique_lock<std::mutex> lock(_load_mutex);
      if (!_points.isEmpty()) {
        LOG_DEBUG("draw graph track " << _points.size());
        QVector<QPointF> points2;
        double lo = _points[0].y();
        double hi = _points[0].y();
        for (auto &p : _points) {
          lo = std::min(lo, p.y());
          hi = std::max(hi, p.y());
        }
        LOG_DEBUG("yrange " << lo << " " << hi);
        for (auto p : _points) {
          p.setX(p.x() * _track_length);
          p.setY((0.9 - (p.y() - lo) / (hi - lo) * 0.8) * track_height +
                 track_height + _track_index * track_height);
          points2.push_back(p);
        }
        polygon = QPolygonF(points2);
      }
    }
    painter->save();
    painter->setRenderHints(QPainter::Antialiasing);
    painter->setPen(_pen);
    painter->setBrush(QBrush(Qt::transparent));
    painter->drawPolyline(polygon);
    painter->restore();
  }
  */
};

class AnnotationTrackView : public TrackViewBase {
  const std::shared_ptr<AnnotationTrack> _track;
  QGraphicsRectItem *_item_prototype = nullptr;
  double _new_item_start = 0;
  std::vector<AnnotationSpanItem *> _span_items;
  QGraphicsRectItem *_clip_rect = new QGraphicsRectItem(this);

 private:
  void updateItemPrototype(QGraphicsSceneMouseEvent *event) {
    LockScope ws;
    double track_length =
        scene()->sceneRect().width() - track_label_width - track_padding_right;
    double x = std::max(0.0, std::min(track_length, event->scenePos().x()));
    double wmin = 3;
    if (x == _new_item_start) {
      x += wmin;
    }
    x = std::min(x, track_length);
    if (auto branch = _track->branch(ws(), false)) {
      for (auto &span : branch->spans()) {
        double left = span->start() * track_length / timelineDuration(ws());
        double right = (span->start() + span->duration()) * track_length /
                       timelineDuration(ws());
        if (_new_item_start > right) {
          x = std::max(x, right);
        }
        if (_new_item_start < left) {
          x = std::min(x, left);
        }
      }
    }
    double x0 = std::min(x, _new_item_start);
    double x1 = std::max(x, _new_item_start);
    _item_prototype->setRect(x0, rect().y(), x1 - x0, rect().height());
  }

 protected:
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsItem::mousePressEvent(event);
    LockScope ws;
    double track_length =
        scene()->sceneRect().width() - track_label_width - track_padding_right;
    if (event->button() == Qt::LeftButton) {
      event->accept();
      _new_item_start =
          std::max(0.0, std::min(scene()->sceneRect().width() -
                                     track_padding_right - track_label_width,
                                 event->scenePos().x()));
      {
        double t = _new_item_start * timelineDuration(ws()) / track_length;
        if (auto branch = _track->branch(ws(), false)) {
          for (auto &span : branch->spans()) {
            if (t > span->start() && t < span->start() + span->duration()) {
              return;
            }
          }
        }
      }
      updateItemPrototype(event);
      _item_prototype->setBrush(
          QBrush(QColor::fromHsvF(_track->color(), 1.0, 0.5)));
      _item_prototype->show();
    }
    if (!ws->selection().empty()) {
      ws->selection().clear();
      ws->modified();
    }
  }
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsItem::mouseMoveEvent(event);
    LockScope ws;
    if (event->buttons() & Qt::LeftButton) {
      event->accept();
      updateItemPrototype(event);
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsItem::mouseReleaseEvent(event);
    LockScope ws;
    if (event->button() == Qt::LeftButton) {
      event->accept();
      if (_item_prototype->isVisible()) {
        if (_item_prototype->rect().width() > 3.2) {
          ActionScope ws("Add annotation");
          double track_length = scene()->sceneRect().width() -
                                track_padding_right - track_label_width;
          auto span = std::make_shared<AnnotationSpan>();
          span->start() = _item_prototype->rect().x() * timelineDuration(ws()) /
                          track_length;
          span->duration() = _item_prototype->rect().width() *
                             timelineDuration(ws()) / track_length;
          _track->branch(ws(), true)->spans().push_back(span);
          _item_prototype->hide();
          ws->selection() = span;
          ws->currentAnnotationTrack() = _track;
          ws->modified();
        }
        _item_prototype->hide();
      }
    }
  }

 public:
  AnnotationTrackView(size_t track_index,
                      std::shared_ptr<AnnotationTrack> track)
      : TrackViewBase(track_index, track), _track(track) {
    // setBrush(QApplication::palette().brush(QPalette::Dark));
    setBrush(QBrush(Qt::transparent));
    setPen(QPen(Qt::NoPen));

    _item_prototype = new QGraphicsRectItem();
    _item_prototype->setParentItem(this);
    _item_prototype->setBrush(QBrush(Qt::blue));
    _item_prototype->setPen(QPen(Qt::NoPen));
    _item_prototype->hide();

    _clip_rect->setFlag(ItemClipsChildrenToShape, true);
    _clip_rect->setBrush(QBrush(Qt::transparent));
    _clip_rect->setPen(QPen(Qt::NoPen));
  }
  virtual void update(const std::shared_ptr<Workspace> &ws,
                      double width) override {
    TrackViewBase::update(ws, width);

    auto *view = scene()->views().first();
    auto view_top_left = view->mapToScene(view->viewport()->rect().topLeft());
    auto view_top_right = view->mapToScene(view->viewport()->rect().topRight());
    _clip_rect->setRect(
        QRect(view_top_left.x() + track_label_width, scene()->sceneRect().y(),
              view_top_right.x() - view_top_left.x() - track_label_width,
              scene()->sceneRect().height() + track_height));

    QRectF rect = scene()->sceneRect();
    double y = track_height * (_track_index + 1);
    if (auto branch = _track->branch(ws, false)) {
      for (size_t i = 0; i < branch->spans().size(); i++) {
        if (_span_items.size() <= i) {
          auto *item = new AnnotationSpanItem();
          item->setParentItem(_clip_rect);
          _span_items.push_back(item);
        }
        auto *item = _span_items[i];
        auto &annotation = branch->spans()[i];
        item->update(ws, _track_index, _track, annotation, width,
                     timelineDuration(ws));
      }
      while (_span_items.size() > branch->spans().size()) {
        delete _span_items.back();
        _span_items.pop_back();
      }
    } else {
      while (_span_items.size() > 0) {
        delete _span_items.back();
        _span_items.pop_back();
      }
    }
  }
};

class Scene : public QGraphicsScene, public ItemBase {
  double _width = 300;
  bool _enabled = false;
  std::vector<TrackViewBase *> _tracks;

 private:
  TrackViewBase *createTrackView(size_t index,
                                 const std::shared_ptr<TrackBase> &track) {
    if (auto t = std::dynamic_pointer_cast<AnnotationTrack>(track)) {
      return new AnnotationTrackView(index, t);
    }
    if (auto t = std::dynamic_pointer_cast<GraphTrack>(track)) {
      return new GraphTrackView(index, t);
    }
    throw std::runtime_error("unknown track type");
  }

 public:
  virtual void sync(const std::shared_ptr<Workspace> &ws) override {
    //_enabled = (ws->player != nullptr);
    _enabled = true;
    {
      size_t track_count = trackCount(ws);
      LOG_DEBUG("annotation tracks " << track_count);
      while (_tracks.size() > track_count) {
        delete _tracks.back();
        _tracks.pop_back();
      }
      while (_tracks.size() < track_count) {
        auto track =
            createTrackView(_tracks.size(), trackAt(ws, _tracks.size()));
        addItem(track);
        _tracks.push_back(track);
      }
    }
    updateSceneRect();
    for (size_t i = 0; i < _tracks.size(); i++) {
      if (_tracks[i]->track() != trackAt(ws, i)) {
        delete _tracks[i];
        _tracks[i] = nullptr;
        _tracks[i] = createTrackView(i, trackAt(ws, i));
        addItem(_tracks[i]);
      }
      _tracks[i]->update(ws, _width);
    }
  }
  void updateSceneRect() {
    LockScope ws;
    double height = track_height * (trackCount(ws()) + 1);
    setSceneRect(-track_label_width, 0,
                 track_label_width + _width + track_padding_right, height);
  }
  Scene(QWidget *parent) : QGraphicsScene(parent) {
    updateSceneRect();
    // setBackgroundBrush(QApplication::palette().brush(QPalette::Mid));
    setBackgroundBrush(QApplication::palette().brush(QPalette::Window));
  }
  void wheelEvent(QGraphicsSceneWheelEvent *wheel) {
    // QGraphicsScene::wheelEvent(wheel);
    // double posx = wheel->scenePos().x();
    // LOG_DEBUG("posx " << posx);

    wheel->accept();

    auto *view = views().first();

    QPointF center = view->mapToScene(view->viewport()->rect().center());
    double center2mouse = wheel->scenePos().x() - center.x();
    LOG_DEBUG("zoom " << wheel->scenePos().x() << " " << center.x() << " "
                      << center2mouse);
    center.setX(center.x() + center2mouse);
    center.setX(center.x() / _width);

    double degrees = wheel->delta() * (1.0 / 8);
    double exponent = degrees / 90;
    double factor = std::pow(0.5, -exponent);
    _width *= factor;
    LOG_DEBUG("scene " << -track_label_width << " " << track_label_width << " "
                       << _width << " " << track_label_width + _width);
    updateSceneRect();
    LockScope()->modified();

    center.setX(center.x() * _width);
    center.setX(center.x() - center2mouse);
    view->centerOn(center);

    LockScope()->modified();
  }
  void handleSeek(QGraphicsSceneMouseEvent *event) {
    LOG_DEBUG("seek");
    LockScope ws;
    if (ws->player) {
      event->accept();
      ws->player->seek(std::max(
          0.0,
          std::min(timelineDuration(ws()),
                   event->scenePos().x() * timelineDuration(ws()) / _width)));
    }
  }
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsScene::mousePressEvent(event);
    if (event->isAccepted()) {
      return;
    }
    if (event->button() == Qt::MiddleButton) {
      event->accept();
    }
    if (event->button() == Qt::LeftButton &&
        event->scenePos().y() < track_height) {
      handleSeek(event);
    }
  }
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsScene::mouseMoveEvent(event);
    if (event->isAccepted()) {
      return;
    }
    if (event->buttons() == Qt::MiddleButton) {
      event->accept();
      static int recursion_depth = 0;
      if (recursion_depth == 0) {
        recursion_depth++;
        if (auto *view = views().first()) {
          double dx = event->scenePos().x() - event->lastScenePos().x();
          LOG_DEBUG("dx " << dx);
          view->translate(dx, 0);
          LockScope()->modified();
        }
        recursion_depth--;
      }
    }
    if (event->buttons() & Qt::LeftButton &&
        event->buttonDownScenePos(Qt::LeftButton).y() < track_height) {
      handleSeek(event);
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    QGraphicsScene::mouseReleaseEvent(event);
  }

  void render(QPainter *painter, const QRectF &target, const QRectF &source,
              Qt::AspectRatioMode aspectRatioMode) {
    QGraphicsScene::render(painter, target, source, aspectRatioMode);
  }

  void drawBackground(QPainter *painter, const QRectF &rect) {
    QGraphicsScene::drawBackground(painter, rect);
    painter->save();
    painter->fillRect(rect, QApplication::palette().brush(QPalette::Base));
    painter->setPen(QPen(QApplication::palette().brush(
                             _enabled ? QPalette::Normal : QPalette::Disabled,
                             QPalette::ButtonText),
                         1.0));
    for (double y = track_height; y < sceneRect().height() + track_height * 0.5;
         y += track_height) {
      painter->drawLine(rect.x(), y, rect.x() + rect.width(), y);
      if (y == track_height) {
        auto pen = painter->pen();
        auto brush = pen.brush();
        auto color = brush.color();
        color.setAlpha(color.alpha() / 4);
        brush.setColor(color);
        pen.setBrush(brush);
        painter->setPen(pen);
      }
    }
    painter->restore();
  }
};

class TimeBar : public QGraphicsRectItem, public ItemBase {
 public:
  TimeBar() {
    setBrush(QBrush(Qt::transparent));
    setPen(QPen(Qt::NoPen));
    setZValue(10);
  }
  virtual void sync(const std::shared_ptr<Workspace> &ws) override {
    QRectF rect = scene()->sceneRect();
    setRect(QRectF(0, 0, rect.width() - track_label_width - track_padding_right,
                   track_height - 1));
  }
};

class ScaleItem : public QGraphicsItem, public ItemBase {
  double _duration = 0.001;

 public:
  ScaleItem() {}
  virtual void sync(const std::shared_ptr<Workspace> &ws) override {
    _duration = timelineDuration(ws);
  }
  virtual QRectF boundingRect() const override {
    auto rect = scene()->sceneRect();
    auto *graphics_view = scene()->views().first();
    if (graphics_view) {
      auto clip =
          graphics_view->mapToScene(graphics_view->viewport()->geometry())
              .boundingRect();
      rect.setHeight(clip.y() + clip.height());
    }
    return rect;
  }
  double timeToPosition(double t) {
    auto r = parentItem()->boundingRect();
    return r.x() + r.width() * t / _duration;
  }
  double positionToTime(double p) {
    auto r = parentItem()->boundingRect();
    return (p - r.x()) * _duration / r.width();
  }
  void drawTime(QPainter *painter, double t) {
    double p = timeToPosition(t);
    painter->drawLine(p, 0.0, p, track_height * 0.15);
    QTime tq(0, 0);
    tq = tq.addMSecs(std::round(t * 1000));
    QString tstr = tq.toString("hh:mm:ss.zzz");
    painter->drawText(QRectF(p - 100, 0, 200, track_height),
                      Qt::AlignCenter | Qt::TextSingleLine, tstr);
  }
  void drawTick(QPainter *painter, double t) {
    double p = timeToPosition(t);
    painter->drawLine(p, track_height * 0.85, p, track_height);
  }
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {
    painter->save();

    {
      QColor color = QApplication::palette().color(
          isEnabled() ? QPalette::Normal : QPalette::Disabled,
          QPalette::ButtonText);
      color.setAlpha(color.alpha() / 4);
      painter->setPen(QPen(QBrush(color), 1.0));
    }

    auto *graphics_view = scene()->views().first();
    auto clip = graphics_view->mapToScene(graphics_view->viewport()->geometry())
                    .boundingRect();

    {
      auto rect = clip;
      rect.setY(0);
      rect.setHeight(track_height);
      rect.setX(rect.x() - 1);
      rect.setWidth(rect.width() + 2);
      painter->fillRect(rect, QApplication::palette().brush(QPalette::Button));
      {
        painter->save();
        painter->setPen(
            QPen(QApplication::palette().brush(
                     isEnabled() ? QPalette::Normal : QPalette::Disabled,
                     QPalette::ButtonText),
                 1.0));
        painter->drawLine(rect.x(), track_height, rect.x() + rect.width(),
                          track_height);
        painter->restore();
      }
      painter->drawLine(clip.x() + track_label_width - 1, track_height,
                        clip.x() + track_label_width - 1,
                        clip.y() + clip.height());
    }

    auto r = parentItem()->boundingRect();
    painter->drawLine(r.x() + r.width(), clip.y(), r.x() + r.width(),
                      clip.height());

    painter->setPen(
        QPen(QApplication::palette().brush(
                 isEnabled() ? QPalette::Normal : QPalette::Disabled,
                 QPalette::ButtonText),
             1.0));

    double min_width = QFontMetrics(painter->font()).width("     00:00:00.000");
    double desired_time_step = min_width * _duration / r.width();
    auto step_sizes = {
        0.0001,
        0.0005,
        0.001,
        0.005,
        0.01,
        0.05,
        0.1,
        0.5,
        1.0,
        10.0,
        30.0,
        60.0,
        5.0 * 60,
        10.0 * 60,
        30.0 * 60,
        60.0 * 60,
        3.0 * 60 * 60,
        6.0 * 60 * 60,
        12.0 * 60 * 60,
        24.0 * 60 * 60,
    };
    double time_step = *step_sizes.begin();
    double tick_step = *step_sizes.begin();
    for (auto &s : step_sizes) {
      tick_step = time_step;
      time_step = s;
      if (s > desired_time_step) {
        break;
      }
    }

    double t0 = std::max(
        0.0, std::round(positionToTime(clip.x()) / time_step) * time_step -
                 time_step);
    double tn = std::min(
        _duration,
        std::round(positionToTime(clip.x() + clip.width()) / time_step) *
                time_step +
            time_step);
    for (double t = t0; t == t0 || t < tn; t += tick_step) {
      drawTick(painter, t);
    }
    for (double t = t0; t == t0 || t < tn - desired_time_step; t += time_step) {
      double p = timeToPosition(t);
      drawTime(painter, t);
    }
    drawTick(painter, _duration);
    drawTime(painter, _duration);
    painter->restore();
  }
};

class SeekHead : public QGraphicsRectItem, public ItemBase {
  bool _dragged = false;
  double _drag_offset = 0.0;
  QTimer *timer = new QTimer((ItemBase *)this);

 public:
  SeekHead() {
    setRect(0, -1, 11, track_height);
    auto brush = QBrush(QColor(255, 0, 0));
    setBrush(brush);
    setPen(QPen(QColor(130, 0, 0), 1.0));
    auto *seek_line = new QGraphicsLineItem(5, rect().height() * 0.5, 5, 1000);
    seek_line->setPen(QPen(brush, 3.0));
    seek_line->setParentItem(this);
    timer->setInterval(10);
    QObject::connect(timer, &QTimer::timeout, (ItemBase *)this,
                     [this]() { sync(LockScope().ws()); });
    auto *fx = new QGraphicsDropShadowEffect();
    fx->setBlurRadius(4);
    fx->setOffset(1, 2);
    setGraphicsEffect(fx);
    setZValue(1000000);
    seek_line->setZValue(1000000);
    setCursor(Qt::OpenHandCursor);
  }
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
    if (event->button() == Qt::LeftButton) {
      event->accept();
      _dragged = true;
      setCursor(Qt::ClosedHandCursor);
      _drag_offset = pos().x() + rect().width() * 0.5 - event->scenePos().x();
      sync(LockScope().ws());
    }
  }
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
    auto parent_rect = parentItem()->sceneBoundingRect();
    double x = _drag_offset + event->scenePos().x();
    x = std::max(parent_rect.x(),
                 std::min(parent_rect.x() + parent_rect.width(), x));
    setX(x - rect().width() * 0.5);
    {
      LockScope ws;
      if (auto player = ws->player) {
        player->seek((x - parent_rect.x()) * timelineDuration(ws.ws()) /
                     parent_rect.width());
      }
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    if (event->button() == Qt::LeftButton) {
      _dragged = false;
      setCursor(Qt::OpenHandCursor);
    }
  }
  virtual void sync(const std::shared_ptr<Workspace> &ws) override {
    setZValue(1000000);
    if (_dragged) {
      return;
    }
    if (ws->player) {
      auto r = parentItem()->sceneBoundingRect();
      auto px = ws->player->time() * r.width() / timelineDuration(ws) + r.x() -
                rect().width() * 0.5;
      if (x() != px) {
        setX(px);
        update();
      }
      if (!isVisible()) {
        show();
      }
      if (!timer->isActive()) {
        timer->start();
      }
    } else {
      if (isVisible()) {
        hide();
      }
      if (timer->isActive()) {
        timer->stop();
      }
    }
  }
};

TimelineWidget::TimelineWidget() : QDockWidget("Timeline") {
  LockScope ws;

  QWidget *content_widget = new QWidget();

  auto *main_layout = new QVBoxLayout(content_widget);
  main_layout->setSpacing(0);
  main_layout->setContentsMargins(0, 0, 0, 0);

  auto *playback_bar = new QHBoxLayout(content_widget);
  playback_bar->setSpacing(0);
  playback_bar->setContentsMargins(3, 3, 3, 3);

  QWidget *playback_bar_widget = new QWidget();
  playback_bar_widget->setAutoFillBackground(true);
  playback_bar_widget->setLayout(playback_bar);
  setTitleBarWidget(playback_bar_widget);

  connect(this, &QDockWidget::dockLocationChanged, this,
          [this](Qt::DockWidgetArea area) {
            LOG_DEBUG("timeline dock location changed");
            updateGeometry();
            update();
          });

  auto *playback_bar_left = new QHBoxLayout(content_widget);
  playback_bar_left->setSpacing(0);
  playback_bar->addLayout(playback_bar_left, 4);

  std::unordered_set<QWidget *> always_active_widgets;

  if (0) {
    auto *title = new QLabel(tr("Timeline"));
    playback_bar_left->addWidget(title);
    always_active_widgets.insert(title);
  }

  {
    auto *button = new FlatButton("Create");
    QMenu *menu = new QMenu(this);
    auto types = Type::find<TrackBase>()->list();
    for (auto &type : types) {
      if (!type->constructable()) {
        continue;
      }
      QString label = type->name().c_str();
      connect(menu->addAction(label), &QAction::triggered, this,
              [type, this](bool checked) {
                ActionScope ws("Create Timeline Track");
                auto track = type->instantiate<TrackBase>();
                for (size_t i = 1;; i++) {
                  auto label = std::string() + type->name() + std::to_string(i);
                  bool label_taken = false;
                  for (auto &t : ws->document()->timeline()->tracks()) {
                    if (t->label() == label) {
                      label_taken = true;
                      break;
                    }
                  }
                  if (label_taken) {
                    continue;
                  } else {
                    track->label() = label;
                    break;
                  }
                }
                {
                  std::vector<double> colors;
                  for (auto &t : ws->document()->timeline()->tracks()) {
                    colors.push_back(t->color());
                  }
                  if (colors.size() == 1) {
                    double color = colors.at(0) + 0.5;
                    color -= std::floor(color);
                    track->color() = color;
                  }
                  if (colors.size() >= 2) {
                    std::sort(colors.begin(), colors.end());
                    double best_color = 0;
                    double largest_distance = 0;
                    for (size_t i = 0; i < colors.size(); i++) {
                      size_t j = (i + 1) % colors.size();
                      double distance = colors[j] - colors[i];
                      distance -= std::floor(distance);
                      if (distance > largest_distance) {
                        largest_distance = distance;
                        best_color = colors[i] + distance * 0.5;
                      }
                    }
                    track->color() = best_color - std::floor(best_color);
                  }
                }
                ws->document()->timeline()->tracks().push_back(track);
                ws->modified();
              });
    }
    button->setMenu(menu);
    always_active_widgets.insert(button);
    playback_bar_left->addWidget(button);
  }

  if (0) {
    auto *open_button = new FlatButton("Open");
    always_active_widgets.insert(open_button);
    connect(open_button, &QPushButton::clicked, this, [this]() {
      QString path = QFileDialog::getOpenFileName(
          this, tr("Open Bag"), QString(),
          tr("Bag files (*.bag);;All files (*.*)"));
      if (path.isNull()) {
        return;
      }
      MainWindow::instance()->openBag(path);
    });
    playback_bar_left->addWidget(open_button);
  }

  if (1) {
    auto *open_button = new FlatButton("Data");
    QMenu *menu = new QMenu(this);
    connect(menu, &QMenu::aboutToShow, [menu, this]() {
      LOG_DEBUG("update timeline bag menu");
      menu->clear();
      connect(menu->addAction("Open Bag"), &QAction::triggered, this,
              [this](bool checked) {
                QString path = QFileDialog::getOpenFileName(
                    this, tr("Open Bag"), QString(),
                    tr("Bag files (*.bag);;All files (*.*)"));
                if (path.isNull()) {
                  return;
                }
                MainWindow::instance()->openBag(path);
              });
      LockScope ws;
      {
        auto action = menu->addAction("Close Bag");
        connect(action, &QAction::triggered, this,
                [this](bool checked) { MainWindow::instance()->closeBag(); });
        action->setEnabled(ws->player != nullptr);
      }
      {
        auto *action = menu->addAction("Republish");
        action->setCheckable(true);
        action->setChecked(ws->document()->display()->republish());
        connect(action, &QAction::toggled, []() {
          ActionScope ws{"Republishing"};
          ws->document()->display()->republish() =
              !ws->document()->display()->republish();
          ws->modified();
        });
      }
      std::set<std::string> branches;
      for (auto &track : ws->document()->timeline()->tracks()) {
        LOG_DEBUG("track " << track->label());
        if (auto annotation_track =
                std::dynamic_pointer_cast<AnnotationTrack>(track)) {
          for (auto &branch : annotation_track->branches()) {
            branches.insert(branch->name());
          }
        }
      }
      // if (ws->player) {
      //    branches.insert(ws->player->fileName());
      //}
      if (!branches.empty()) {
        menu->addSeparator();
        for (auto &branch : branches) {
          auto *action = menu->addAction(QString::fromStdString(branch));
          connect(action, &QAction::triggered, this,
                  [this, branch](bool checked) {
                    if (checked) {
                      MainWindow::instance()->findAndOpenBag(branch);
                    } else {
                      MainWindow::instance()->closeBag();
                    }
                  });
          action->setCheckable(true);
          if (auto player = ws->player) {
            action->setChecked(branch == ws->player->fileName());
          }
        }
      }
      /*
      std::map<QString, QString> files;
      for (auto &track : ws->document()->timeline()->tracks()) {
        LOG_DEBUG("track " << track->label());
        if (auto annotation_track =
                std::dynamic_pointer_cast<AnnotationTrack>(track)) {
          LOG_DEBUG("annotation track " << track->label());
          for (auto &branch : annotation_track->branches()) {
            {
              QFileInfo f(
                  QFileInfo(QString::fromStdString(ws->document()->path)).dir(),
                  QString::fromStdString(branch->name()));
              LOG_DEBUG("df " << f.absoluteFilePath().toStdString());
              if (f.exists()) {
                files[f.fileName()] = f.absoluteFilePath();
              }
            }
            if (auto player = ws->player) {
              QFileInfo f(
                  QFileInfo(QString::fromStdString(player->path())).dir(),
                  QString::fromStdString(branch->name()));
              LOG_DEBUG("pf " << f.absoluteFilePath().toStdString());
              if (f.exists()) {
                files[f.fileName()] = f.absoluteFilePath();
              }
            }
          }
        }
      }
      if (!files.empty()) {
        menu->addSeparator();
        for (auto f : files) {
          auto *action = menu->addAction(f.first);
          connect(action, &QAction::triggered, this, [this, f](bool checked) {
            if (checked) {
              MainWindow::instance()->openBag(f.second);
            } else {
              MainWindow::instance()->closeBag();
            }
          });
          action->setCheckable(true);
          if (auto player = ws->player) {
            action->setChecked(f.first.toStdString() == ws->player->fileName());
          }
        }
      }
      */
    });
    open_button->setMenu(menu);
    always_active_widgets.insert(open_button);
    playback_bar_left->addWidget(open_button);
  }

  {
    auto *button = new FlatButton("Export");
    QMenu *menu = new QMenu(this);
    connect(
        menu->addAction("Segments"), &QAction::triggered, this,
        [this](bool checked) {
          try {
            LockScope ws;
            auto player = ws->player;
            if (!player) {
              throw std::runtime_error("No bag opened");
            }
            std::vector<std::pair<std::shared_ptr<AnnotationTrack>,
                                  std::shared_ptr<AnnotationBranch>>>
                tracks_branches;
            for (auto &track : ws->document()->timeline()->tracks()) {
              if (auto annotation_track =
                      std::dynamic_pointer_cast<AnnotationTrack>(track)) {
                if (annotation_track->segmentExport()) {
                  if (annotation_track->segmentPath().empty()) {
                    throw std::runtime_error("No segment export path");
                  }
                  for (auto &branch : annotation_track->branches()) {
                    if (branch->name() == player->fileName()) {
                      tracks_branches.emplace_back(annotation_track, branch);
                    }
                  }
                }
              }
            }
            std::string src_path(player->path());
            QProgressDialog *progress = new QProgressDialog(
                tr("Writing bag segments..."), QString(), 0, 0, this);
            progress->setModal(true);
            progress->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                                     Qt::CustomizeWindowHint);
            progress->show();
            std::string src_name = player->fileName();
            // volatile bool finished = false;
            std::thread([src_path, tracks_branches, progress, src_name]() {
              rosbag::Bag src_bag(src_path, rosbag::bagmode::Read);
              rosbag::View src_view(src_bag);
              ros::Time bag_start = src_view.getBeginTime();
              for (auto &track_branch : tracks_branches) {
                auto &track = track_branch.first;
                auto &branch = track_branch.second;
                std::vector<std::shared_ptr<AnnotationSpan>> spans;
                {
                  LockScope ws;
                  spans = branch->spans();
                }
                for (auto &span : spans) {
                  ros::Time span_start, span_end;
                  std::string dst_path, segment_path;
                  {
                    LockScope ws;
                    segment_path = track->segmentPath();
                    span_start = bag_start + ros::Duration(span->start());
                    span_end = bag_start +
                               ros::Duration(span->start() + span->duration());
                    std::string dst_name = src_name;
                    {
                      std::string ext = ".bag";
                      if (dst_name.size() >= ext.size() &&
                          dst_name.substr(dst_name.size() - ext.size()) ==
                              ext) {
                        dst_name.resize(dst_name.size() - ext.size());
                      }
                    }
                    for (auto &c : dst_name)
                      if (c == '.') c = '_';
                    dst_path =
                        track->segmentPath() + "/" + dst_name + "_" +
                        std::to_string(int(std::round(span->start() * 1000))) +
                        "_" +
                        std::to_string(
                            int(std::round(span->duration() * 1000))) +
                        ".bag";
                    LOG_INFO("write bag " << dst_path << " " << span->start()
                                          << " " << span->duration());
                  }
                  QDir().mkpath(QString::fromStdString(segment_path));
                  rosbag::View span_view(src_bag, span_start, span_end);
                  startOnMainThreadAsync([progress, dst_path]() {
                    progress->setLabelText(tr("Writing bag segment: ") + " " +
                                           QString::fromStdString(dst_path));
                  });
                  rosbag::Bag dst_bag(dst_path, rosbag::bagmode::Write);
                  for (auto &msg : span_view) {
                    LOG_INFO("write message " << dst_path << " "
                                              << msg.getTime());
                    dst_bag.write(msg.getTopic(), msg.getTime(),
                                  msg.instantiate<Message>());
                  }
                }
              }
              startOnMainThreadAsync([progress]() { progress->deleteLater(); });
            }).detach();
            // while (!finished) {
            //   QThread::msleep(10);
            //   QApplication::processEvents();
            // }
            // thread.join();
          } catch (const std::exception &ex) {
            QMessageBox::critical(nullptr, "Error", ex.what());
          }
        });
    connect(
        menu->addAction("Annotated Bag"), &QAction::triggered, this,
        [this](bool checked) {
          QProgressDialog *progress = new QProgressDialog(
              tr("Writing annotated bag..."), QString(), 0, 1000, this);
          progress->setModal(true);
          progress->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                                   Qt::CustomizeWindowHint);
          progress->show();
          auto stop_flag = std::make_shared<volatile bool>(0);
          connect(progress, &QProgressDialog::canceled,
                  [stop_flag]() { *stop_flag = true; });
          LockScope ws;
          auto player = ws->player;
          if (!player) {
            throw std::runtime_error("No bag opened");
          }
          std::string src_path(player->path());
          std::string dst_path(src_path + ".annotated.bag");
          std::string player_file_name = player->fileName();
          QFile::remove(dst_path.c_str());
          std::string src_name = player->fileName();
          // volatile bool finished = false;
          std::thread([src_path, player_file_name, dst_path, progress, src_name,
                       stop_flag]() {
            Destructor progress_destructor([progress]() {
              startOnMainThreadAsync([progress]() { progress->deleteLater(); });
            });
            try {
              {
                rosbag::Bag src_bag(src_path, rosbag::bagmode::Read);
                rosbag::Bag dst_bag(dst_path, rosbag::bagmode::Write);
                LOG_INFO("export annotated bag " << dst_path);
                rosbag::View view(src_bag);
                // double t = 0.0;
                auto out_time = view.getBeginTime();
                ros::Time in_time;
                for (auto &message : view) {
                  if (*stop_flag) {
                    LOG_WARN("annotated bag creating stopped");
                    return;
                  }
                  double t = (message.getTime() - view.getBeginTime()).toSec();
                  {
                    // double t =
                    //     (message.getTime() - view.getBeginTime()).toSec();
                    double tf =
                        t / (view.getEndTime() - view.getBeginTime()).toSec();
                    LOG_DEBUG_THROTTLE(
                        0.2, "writing annotated bag " << tf * 100 << " %");
                    startOnMainThreadAsync([progress, tf]() {
                      progress->setValue(int(tf * 1000));
                    });
                  }
                  // auto out_time = message.getTime();
                  // out_time += ros::Duration(0.5);
                  if (isImageMessageTypeName(message.getDataType())) {
                    // auto message_instance = message.instantiate<Message>();
                    // dst_bag.write(message.getTopic(), message.getTime(),
                    //               message_instance);
                    std::map<std::string,
                             std::vector<std::tuple<
                                 std::shared_ptr<AnnotationTrack>,
                                 std::shared_ptr<AnnotationSpan>,
                                 std::shared_ptr<ImageAnnotationBase>>>>
                        annotation_map;
                    {
                      LockScope ws;
                      for (auto &track : ws->document()->timeline()->tracks()) {
                        if (auto annotation_track =
                                std::dynamic_pointer_cast<AnnotationTrack>(
                                    track)) {
                          for (auto &branch : annotation_track->branches()) {
                            if (branch->name() == player_file_name) {
                              for (auto &span : branch->spans()) {
                                for (auto &annotation : span->annotations()) {
                                  if (auto image_annotation =
                                          std::dynamic_pointer_cast<
                                              ImageAnnotationBase>(
                                              annotation)) {
                                    if (image_annotation->topic() ==
                                        message.getTopic()) {
                                      if (span->start() <= t + 1e-6 &&
                                          span->start() + span->duration() >
                                              t + 1e-6) {
                                        annotation_map[span->label().empty()
                                                           ? track->label()
                                                           : span->label()]
                                            .emplace_back(annotation_track,
                                                          span,
                                                          image_annotation);
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                    if (!annotation_map.empty()) {
                      if (message.getTime() != in_time) {
                        in_time = message.getTime();
                        out_time += ros::Duration(0.3);
                      }
                      auto message_instance = message.instantiate<Message>();
                      dst_bag.write(message.getTopic(), out_time,
                                    message_instance);
                      if (isImageMessageTypeName(message.getDataType())) {
                        cv::Mat image_mat;
                        std::string image_encoding;
                        std_msgs::Header image_header;
                        if (tryConvertImageMessageToMat(
                                *message_instance, image_mat, image_encoding,
                                image_header)) {
                          // image_header.stamp = out_time;
                          LOG_DEBUG("image_encoding " << image_encoding);
                          QImage multi_annotation_image(image_mat.cols,
                                                        image_mat.rows,
                                                        QImage::Format_RGB32);
                          multi_annotation_image.fill(Qt::black);
                          tryConvertMatToImage(image_mat / 2, image_encoding,
                                               multi_annotation_image);
                          // multi_annotation_image.fill(QColor(0, 0, 0, 128));
                          QPainter multi_annotation_painter(
                              &multi_annotation_image);
                          multi_annotation_painter.setCompositionMode(
                              QPainter::CompositionMode_Screen);
                          multi_annotation_painter.setRenderHint(
                              QPainter::Antialiasing, true);
                          for (auto &annotation_pair : annotation_map) {
                            auto &matching_annotations = annotation_pair.second;
                            QImage annotation_image(image_mat.cols,
                                                    image_mat.rows,
                                                    QImage::Format_Grayscale8);
                            {
                              annotation_image.fill(Qt::black);
                              QPainter annotation_painter(&annotation_image);
                              annotation_painter.setRenderHint(
                                  QPainter::Antialiasing, true);
                              {
                                LockScope ws;
                                for (auto &annotation_tuple :
                                     matching_annotations) {
                                  auto &track = std::get<0>(annotation_tuple);
                                  auto &span = std::get<1>(annotation_tuple);
                                  auto &image_annotation =
                                      std::get<2>(annotation_tuple);
                                  auto label = span->label().empty()
                                                   ? track->label()
                                                   : span->label();
                                  LOG_DEBUG("annotation span "
                                            << label << " "
                                            << image_annotation->topic() << " "
                                            << span->start() << " "
                                            << span->duration());
                                  image_annotation->heatmap(annotation_painter,
                                                            Qt::white);
                                  // image_annotation->render();
                                  // if (auto &shape = image_annotation->shape)
                                  // {
                                  //   annotation_painter.setPen(Qt::NoPen);
                                  //   annotation_painter.setBrush(Qt::white);
                                  //   annotation_painter.drawPath(*shape);
                                  // }
                                  image_annotation->heatmap(
                                      multi_annotation_painter,
                                      QColor::fromHsvF(track->color(), 1, 1,
                                                       0.5));
                                }
                              }
                              annotation_painter.end();
                            }
                            //
                            // sensor_msgs::Image annotation_message;
                            // annotation_message.header = image_header;
                            // annotation_message.width =
                            // annotation_image.width();
                            // annotation_message.height =
                            //     annotation_image.height();
                            // annotation_message.step =
                            // annotation_message.width;
                            // annotation_message.encoding =
                            //     sensor_msgs::image_encodings::TYPE_8UC1;
                            // for (size_t y = 0; y < annotation_message.height;
                            //      y++) {
                            //   auto *line = annotation_image.constScanLine(y);
                            //   for (size_t x = 0; x <
                            //   annotation_message.width;
                            //        x++) {
                            //     annotation_message.data.push_back(line[x]);
                            //   }
                            // }
                            //
                            {
                              sensor_msgs::CompressedImage annotation_message;
                              {
                                QByteArray data;
                                {
                                  QBuffer buf(&data);
                                  annotation_image.save(&buf, "PNG");
                                }
                                annotation_message.header = image_header;
                                annotation_message.data.assign(data.begin(),
                                                               data.end());
                              }
                              //
                              std::string annotation_topic = message.getTopic();
                              if (!annotation_topic.empty() &&
                                  annotation_topic[0] != '/') {
                                annotation_topic = "/" + annotation_topic;
                              }
                              annotation_topic = annotation_topic + "/" +
                                                 annotation_pair.first;
                              LOG_DEBUG("annotation_topic "
                                        << annotation_topic);
                              dst_bag.write(annotation_topic, out_time,
                                            annotation_message);
                            }
                            // {
                            //   cv::Mat channel_map(annotation_image.height(),
                            //                       annotation_image.width(),
                            //                       CV_8UC1,
                            //                       annotation_image.scanLine(0));
                            //   cv::cvtColor(channel_map, channel_map,
                            //                COLOR_GRAY2RGB);
                            // }
                          }
                          {
                            sensor_msgs::CompressedImage annotation_message;
                            {
                              QByteArray data;
                              {
                                QBuffer buf(&data);
                                multi_annotation_image.save(&buf, "JPEG");
                              }
                              annotation_message.header = image_header;
                              annotation_message.data.assign(data.begin(),
                                                             data.end());
                            }
                            //
                            std::string annotation_topic = message.getTopic();
                            if (!annotation_topic.empty() &&
                                annotation_topic[0] != '/') {
                              annotation_topic = "/" + annotation_topic;
                            }
                            annotation_topic =
                                annotation_topic + "/annotations";
                            LOG_DEBUG("annotation_topic " << annotation_topic);
                            dst_bag.write(annotation_topic, out_time,
                                          annotation_message);
                          }
                        }
                      }
                    }
                  }
                  // t += 0.5;
                }
                src_bag.close();
                dst_bag.close();
              }
              LOG_SUCCESS("annotated bag written");
            } catch (const std::exception &ex) {
              QMessageBox::critical(nullptr, "Error", ex.what());
            }
          }).detach();
        });
    connect(menu->addAction("Time Spans / CSV"), &QAction::triggered, this,
            [this](bool checked) {
              try {
                LockScope ws;
                auto player = ws->player;
                if (!player) {
                  throw std::runtime_error("No bag player");
                }
                std::string dst_path(player->path() + ".annotations.csv");
                LOG_INFO("writing csv " << dst_path);
                QFile::remove(dst_path.c_str());
                std::ofstream stream(dst_path);
                stream << "start,duration,label" << std::endl;
                for (auto &track : ws->document()->timeline()->tracks()) {
                  if (auto annotation_track =
                          std::dynamic_pointer_cast<AnnotationTrack>(track)) {
                    for (auto &branch : annotation_track->branches()) {
                      if (branch->name() == player->fileName()) {
                        for (auto &span : branch->spans()) {
                          auto label = span->label().empty() ? track->label()
                                                             : span->label();
                          std::string label_escaped;
                          for (auto &c : label) {
                            if (c == '"') {
                              label_escaped.append("\"\"");
                            } else {
                              label_escaped.push_back(c);
                            }
                          }
                          stream << span->start() << "," << span->duration()
                                 << ",\"" << label_escaped << "\"" << std::endl;
                        }
                      }
                    }
                  }
                }
                LOG_SUCCESS("csv written " << dst_path);
              } catch (const std::exception &ex) {
                QMessageBox::critical(nullptr, "Error", ex.what());
              }
            });
    button->setMenu(menu);
    playback_bar_left->addWidget(button);
  }

  if (0) {
    auto *close_button = new FlatButton("Close");
    connect(close_button, &QPushButton::clicked, this,
            [this]() { MainWindow::instance()->closeBag(); });
    playback_bar_left->addWidget(close_button);
  }

  // {
  //   auto *button = new FlatButton("1x");
  //   QMenu *menu = new QMenu(this);
  //   connect(menu, &QMenu::aboutToShow, [menu, this]() {
  //     LOG_DEBUG("update speed menu");
  //     menu->clear();
  //     for (double speed : {
  //              0.25,
  //              0.5,
  //              1.0,
  //              2.0,
  //              4.0,
  //              8.0,
  //          }) {
  //       auto *action = menu->addAction(QString::number(speed) + "x");
  //       connect(action, &QAction::triggered, this, [this, speed](bool
  //       checked) {
  //       });
  //       action->setCheckable(true);
  //       if (auto player = ws->player) {
  //         action->setChecked(branch == ws->player->fileName());
  //       }
  //     }
  //   });
  //   button->setMenu(menu);
  //   playback_bar_left->addWidget(button);
  // }

  playback_bar_left->addStretch(1);

  {
    auto *button = new FlatButton(MATERIAL_ICON("fast_rewind", -0.13));
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      if (ws->player) {
        ws->player->rewind();
      }
    });
    playback_bar->addWidget(button);
  }

  {
    auto *button = new FlatButton(MATERIAL_ICON("skip_previous", -0.1));
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      if (ws->player) {
        double current_time = ws->player->time();
        double seek_time = -std::numeric_limits<double>::max();
        for (size_t i = 0; i < trackCount(ws()); i++) {
          if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                  trackAt(ws(), i))) {
            if (auto branch = track->branch(ws(), false)) {
              for (auto &span : branch->spans()) {
                for (double span_time : {
                         span->start(),
                         span->start() + span->duration(),
                     }) {
                  if (span_time < current_time - 1e-6 &&
                      std::abs(current_time - span_time) <
                          std::abs(current_time - seek_time)) {
                    seek_time = span_time;
                  }
                }
              }
            }
          }
        }
        ws->player->seek(
            std::min(timelineDuration(ws()), std::max(0.0, seek_time)));
      }
    });
    playback_bar->addWidget(button);
  }

  {
    auto *button = new FlatButton(MATERIAL_ICON("stop", -0.1));
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      if (ws->player) {
        ws->player->stop();
      }
    });
    playback_bar->addWidget(button);
  }
  {
    auto *button = new FlatButton(MATERIAL_ICON("play_arrow", -0.12));
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      if (ws->player && ws->document()->timeline()) {
        std::vector<double> annotation_times;
        for (auto &track : ws->document()->timeline()->tracks()) {
          if (auto annotation_track =
                  std::dynamic_pointer_cast<AnnotationTrack>(track)) {
            if (auto branch = annotation_track->branch(ws(), false)) {
              for (auto &span : branch->spans()) {
                annotation_times.push_back(span->start());
                annotation_times.push_back(span->start() + span->duration());
              }
            }
          }
        }
        std::sort(annotation_times.begin(), annotation_times.end());
        annotation_times.erase(
            std::unique(annotation_times.begin(), annotation_times.end()),
            annotation_times.end());
        ws->player->play(annotation_times);
      }
    });
    playback_bar->addWidget(button);
  }

  {
    auto *button = new FlatButton(MATERIAL_ICON("skip_next", -0.1));
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      if (ws->player) {
        double current_time = ws->player->time();
        double seek_time = std::numeric_limits<double>::max();
        for (size_t i = 0; i < trackCount(ws()); i++) {
          if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                  trackAt(ws(), i))) {
            if (auto branch = track->branch(ws(), false)) {
              for (auto &span : branch->spans()) {
                for (double span_time : {
                         span->start(),
                         span->start() + span->duration(),
                     }) {
                  if (span_time > current_time + 1e-6 &&
                      std::abs(current_time - span_time) <
                          std::abs(current_time - seek_time)) {
                    seek_time = span_time;
                  }
                }
              }
            }
          }
        }
        ws->player->seek(
            std::min(timelineDuration(ws()), std::max(0.0, seek_time)));
      }
    });
    playback_bar->addWidget(button);
  }

  auto *playback_bar_right = new QHBoxLayout(content_widget);
  playback_bar_right->setSpacing(0);
  playback_bar->addLayout(playback_bar_right, 4);

  playback_bar_right->addStretch(1);

  {
    auto *button =
        new FlatButton(style()->standardIcon(QStyle::SP_TitleBarCloseButton));
    connect(button, &QPushButton::clicked, this, [this]() {
      LOG_DEBUG("close");
      hide();
    });
    playback_bar->addWidget(button);
    always_active_widgets.insert(button);
  }

  Scene *scene = new Scene(this);

  auto *time_bar = new TimeBar();
  scene->addItem(time_bar);

  auto *scale_item = new ScaleItem();
  scale_item->setParentItem(time_bar);
  scene->addItem(scale_item);

  auto *seek_head = new SeekHead();
  seek_head->setParentItem(time_bar);
  scene->addItem(seek_head);

  class GraphicsView : public QGraphicsView {
   protected:
    virtual void mouseMoveEvent(QMouseEvent *event) override {
      QGraphicsView::mouseMoveEvent(event);
    }
    virtual void mousePressEvent(QMouseEvent *event) override {
      QGraphicsView::mousePressEvent(event);
    }
    virtual void mouseReleaseEvent(QMouseEvent *event) override {
      QGraphicsView::mouseReleaseEvent(event);
    }
    virtual void scrollContentsBy(int dx, int dy) override {
      QGraphicsView::scrollContentsBy(dx, dy);
      LockScope()->modified();
    }

   public:
    GraphicsView() { setMouseTracking(true); }
  };

  auto *view = new GraphicsView();
  view->setParent(this);
  view->setViewport(new QOpenGLWidget(this));
  view->setScene(scene);
  view->setCacheMode(QGraphicsView::CacheNone);
  view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  view->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  view->setFocusPolicy(Qt::NoFocus);
  view->setTransformationAnchor(QGraphicsView::NoAnchor);

  main_layout->addWidget(view, true);

  auto sync = [=]() {
    bool enabled = false;
    {
      LockScope ws;
      enabled = (ws->player != nullptr);
    }

    updateEnabled(playback_bar, [enabled, always_active_widgets](QWidget *w) {
      if (always_active_widgets.find(w) == always_active_widgets.end()) {
        w->setEnabled(enabled);
      }
    });
  };
  sync();
  ws->modified.connect(playback_bar, sync);

  setWidget(content_widget);
}

TimelineWidget::~TimelineWidget() {}
