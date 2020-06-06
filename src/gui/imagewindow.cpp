// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "imagewindow.h"

#include "../core/bagplayer.h"
#include "../core/log.h"
#include "../core/workspace.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <QGraphicsScene>
#include <QGraphicsView>

#include <condition_variable>
#include <mutex>
#include <thread>

struct AnnotationView : QGraphicsItem {
  bool ok = false;
  ImageWindow *_window = nullptr;
  std::shared_ptr<AnnotationTrack> _track;
  std::shared_ptr<AnnotationSpan> _span;
  std::shared_ptr<ImageAnnotationBase> _annotation;
  QPainterPath _visual;
  QPainterPath _collider;
  QColor _color;
  bool _selected = false;
  double _zoom_factor = 1.0;

  struct ControlPointHandle : QGraphicsEllipseItem {
    AnnotationView *_parent = nullptr;
    size_t _control_point_index = 0;
    ControlPointHandle(AnnotationView *parent, size_t control_point_index)
        : QGraphicsEllipseItem(parent), _parent(parent),
          _control_point_index(control_point_index) {
      double r = 5;
      setRect(-r, -r, 2 * r, 2 * r);
      setBrush(QBrush(Qt::white));
      setPen(QPen(QBrush(Qt::black), 0));
      setScale(1.0 / _parent->_zoom_factor);
    }
    void sync(const std::shared_ptr<Workspace> &ws) {
      auto p = _parent->_annotation->controlPoints().at(_control_point_index);
      setPos(p.x(), p.y());
    }
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
      if (_parent->_window->annotation_type) {
        return;
      }
      if (event->button() == Qt::LeftButton) {
        event->accept();
      }
    }
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
      if (_parent->_window->annotation_type) {
        return;
      }
      if (event->buttons() == Qt::LeftButton) {
        event->accept();
        setPos(pos() + (event->scenePos() - event->lastScenePos()));
        LockScope ws;
        auto p0 = _parent->_annotation->controlPoints();
        for (size_t i = 0; i < _parent->control_point_handles.size(); i++) {
          auto *h = _parent->control_point_handles.at(i);
          _parent->_annotation->controlPoints().at(i) =
              Eigen::Vector2d(h->pos().x(), h->pos().y());
        }
        _parent->_annotation->constrain();
        _parent->sync(ws(), _parent->_zoom_factor);
        _parent->updateShape();
        _parent->_annotation->controlPoints() = p0;
      }
    }
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
      if (_parent->_window->annotation_type) {
        return;
      }
      if (event->button() == Qt::LeftButton) {
        event->accept();
        ActionScope ws("Edit Annotation");
        for (size_t i = 0; i < _parent->control_point_handles.size(); i++) {
          auto *h = _parent->control_point_handles.at(i);
          _parent->_annotation->controlPoints().at(i) =
              Eigen::Vector2d(h->pos().x(), h->pos().y());
        }
        _parent->_annotation->constrain();
        ws->modified();
      }
    }
  };
  std::vector<ControlPointHandle *> control_point_handles;

  AnnotationView(ImageWindow *window, const std::shared_ptr<Workspace> &ws,
                 const std::shared_ptr<AnnotationTrack> &track,
                 const std::shared_ptr<AnnotationSpan> &span,
                 const std::shared_ptr<ImageAnnotationBase> &annotation) {
    _window = window;
    _track = track;
    _span = span;
    _annotation = annotation;
  }
  void updateShape() {
    _annotation->render();
    if (_annotation->shape) {
      _visual = *_annotation->shape;
    } else {
      _visual = QPainterPath();
    }
    if (_annotation->controlPoints().size() == 1 && _annotation->complete) {
      setPos(_annotation->controlPoints().front().x(),
             _annotation->controlPoints().front().y());
      _visual.translate(-_annotation->controlPoints().front().x(),
                        -_annotation->controlPoints().front().y());
      setScale(1.0 / _zoom_factor);
      _collider = QPainterPath();
      _collider.addEllipse(_visual.boundingRect());
    } else {
      setPos(0, 0);
      setScale(1);
      _collider = _visual;
    }
  }
  virtual void sync(const std::shared_ptr<Workspace> &ws, double zoom_factor) {
    _zoom_factor = zoom_factor;

    updateShape();
    _selected = ws->selection().contains(_annotation);

    if ((_window->annotation_type == nullptr && _selected &&
         ws->selection().size() == 1 &&
         _annotation->controlPoints().size() >= 2) ||
        (_window->annotation_type != nullptr &&
         _window->new_annotation == _annotation)) {
      control_point_handles.resize(_annotation->controlPoints().size());
      for (size_t i = 0; i < _annotation->controlPoints().size(); i++) {
        auto &h = control_point_handles.at(i);
        if (!h) {
          h = new ControlPointHandle(this, i);
        }
        h->sync(ws);
      }
    } else {
      for (auto *h : control_point_handles) {
        delete h;
      }
      control_point_handles.clear();
    }
    _color = QColor::fromHsvF(_track->color(), 1, 1, 0.5);
  }
  virtual QRectF boundingRect() const override {
    return _collider.boundingRect();
  }
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) override {

    painter->setPen(QPen(QBrush(QColor(0, 0, 0, 255)), 0, Qt::SolidLine,
                         Qt::SquareCap, Qt::MiterJoin));
    auto color = _color;
    // color.setAlpha(_selected ? 255 : 200);
    painter->setBrush(QBrush(color));
    painter->drawPath(_visual);

    if (_selected && _window->annotation_type == nullptr) {

      painter->save();

      painter->setRenderHint(QPainter::Antialiasing, false);

      auto rect = boundingRect().marginsAdded(QMarginsF(4, 4, 4, 4));

      {
        QRegion region;
        auto r = rect.toRect();
        auto d = r.size() * 0.75;
        region += QRegion(r.translated(+d.width(), +d.height()));
        region += QRegion(r.translated(+d.width(), -d.height()));
        region += QRegion(r.translated(-d.width(), +d.height()));
        region += QRegion(r.translated(-d.width(), -d.height()));
        painter->setClipRegion(region);
      }

      painter->setBrush(QBrush(Qt::transparent));

      painter->setPen(QPen(QBrush(Qt::white), 0, Qt::SolidLine, Qt::FlatCap,
                           Qt::MiterJoin));
      painter->drawRect(rect);

      double m = 1.0 / painter->transform().m11();
      rect = rect.marginsAdded(QMarginsF(m, m, m, m));
      painter->setPen(QPen(QBrush(Qt::black), 0, Qt::SolidLine, Qt::FlatCap,
                           Qt::MiterJoin));
      painter->drawRect(rect);

      painter->restore();
    }
  }
  virtual QPainterPath shape() const override { return _collider; }
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
    /*if (event->isAccepted()) {
      return;
  }*/
    if (_window->annotation_type) {
      return;
    }
    if (event->button() == Qt::LeftButton) {
      LockScope ws;
      if (auto timeline = ws->document()->timeline()) {
        if (_window->annotation_type == nullptr) {
          event->accept();
          if (event->modifiers() == Qt::ControlModifier) {
            ws->selection().toggle(_annotation);
          } else if (event->modifiers() == Qt::ShiftModifier) {
            ws->selection().add(_annotation);
          } else {
            if (!ws->selection().contains(_annotation)) {
              ws->selection() = _annotation;
            }
          }
          ws->modified();
        }
      }
    }
  }
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
    /*if (event->isAccepted()) {
      return;
  }*/
    if (_window->annotation_type) {
      return;
    }
    if (event->buttons() == Qt::LeftButton) {
      for (auto &p : _window->annotation_views) {
        auto *view = p.second;
        if (view->_selected) {
          view->setPos(view->pos() +
                       (event->scenePos() - event->lastScenePos()));
        }
      }
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    /*if (event->isAccepted()) {
      return;
  }*/
    if (_window->annotation_type) {
      return;
    }
    if (event->button() == Qt::LeftButton) {
      if (event->scenePos() != event->buttonDownScenePos(Qt::LeftButton)) {
        ActionScope ws("Move Annotations");
        auto delta =
            (event->scenePos() - event->buttonDownScenePos(Qt::LeftButton)) /*/
            (_zoom_factor / _window->zoom())*/
            ;
        for (auto &p : _window->annotation_views) {
          auto *view = p.second;
          if (view->_selected) {
            auto annotation = p.first;
            for (auto &point : annotation->controlPoints()) {
              point.x() += delta.x();
              point.y() += delta.y();
            }
          }
        }
        ws->modified();
      }
    }
  }
};

ImageWindow::ImageWindow() {

  {
    auto *button = new FlatButton();
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    QMenu *menu = new QMenu(this);
    button->setMenu(menu);
    button->setText("Image Topic");
    addToolWidget(button);
    connect(menu, &QMenu::aboutToShow, this, [this, menu]() {
      // LOG_DEBUG("update topic menu");
      menu->clear();
      auto topics = LockScope()->listTopics(
          ros::message_traits::DataType<sensor_msgs::Image>::value());
      if (topics.empty()) {
        connect(menu->addAction("<no image topics found>"), &QAction::triggered,
                this, [this](bool) {
                  ActionScope ws("Topic");
                  this->topic() = "";
                  ws->modified();
                });
      } else {
        connect(menu->addAction("<none>"), &QAction::triggered, this,
                [this](bool) {
                  ActionScope ws("Topic");
                  this->topic() = "";
                  ws->modified();
                });
        for (auto &topic : topics) {
          connect(menu->addAction(topic.c_str()), &QAction::triggered, this,
                  [topic, this](bool) {
                    ActionScope ws("Topic");
                    this->topic() = topic;
                    ws->modified();
                  });
        }
      }
    });
    LockScope ws;
    ws->modified.connect(button, [this, button]() {
      LockScope ws;
      button->setText(topic().empty() ? "Image Topic" : topic().c_str());
    });
  }

  {
    auto *button = new FlatButton();
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    QMenu *menu = new QMenu(this);
    button->setMenu(menu);
    addToolWidget(button);
    {
      QString label = "Select / Edit";
      button->setText(label);
      connect(menu->addAction(label), &QAction::triggered, this,
              [label, button, this](bool checked) {
                LockScope ws;
                annotation_type = nullptr;
                button->setText(label);
                ws->modified();
              });
    }
    for (auto &type : Type::find<ImageAnnotationBase>()->list()) {
      if (!type->constructable()) {
        continue;
      }
      QString label = type->name().c_str();
      label = label.replace("ImageAnnotation", "");
      connect(menu->addAction(label), &QAction::triggered, this,
              [type, label, button, this](bool checked) {
                LockScope ws;
                annotation_type = type;
                button->setText(label);
                ws->modified();
              });
    }
  }

  {
    auto *button = new FlatButton();
    button->setIcon(MATERIAL_ICON("zoom_out_map"));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      zoom() = 1.0;
      center().x() = 0.5;
      center().y() = 0.5;
      ws->modified();
    });
    addToolWidgetRight(button);
  }

  class MessageBuffer {
    std::mutex _mutex;
    std::shared_ptr<sensor_msgs::Image> _image;
    std::condition_variable _put_condition;
    std::thread _worker;
    QPixmap _pixmap;
    bool _stop = false;
    ros::Time _image_time, _pixmap_time;

  public:
    Event<void()> ready;
    MessageBuffer() {
      _worker = std::thread([this]() {
        while (true) {
          // LOG_DEBUG("image a");
          std::shared_ptr<sensor_msgs::Image> image;
          {
            std::unique_lock<std::mutex> lock(_mutex);
            while (true) {
              if (_stop) {
                return;
              }
              if (_image) {
                image = _image;
                _image = nullptr;
                break;
              }
              _put_condition.wait(lock);
            }
          }
          if (image == nullptr) {
            {
              std::unique_lock<std::mutex> lock(_mutex);
              _pixmap = QPixmap();
            }
            continue;
          }
          // LOG_DEBUG("image b");
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(*image,
                                         sensor_msgs::image_encodings::RGBA8);
          } catch (cv_bridge::Exception &e) {
            LOG_ERROR("cv_bridge exception: " << e.what());
            {
              std::unique_lock<std::mutex> lock(_mutex);
              _pixmap = QPixmap();
            }
            ready();
            continue;
          }
          if (cv_ptr->image.data == nullptr) {
            {
              std::unique_lock<std::mutex> lock(_mutex);
              _pixmap = QPixmap();
            }
            ready();
            continue;
          }
          cv::Mat mat = cv_ptr->image;
          // mat *= 1.5;
          // cv::GaussianBlur(mat, mat, cv::Size(0, 0), 0.5, 0.5);
          auto pixmap =
              QPixmap::fromImage(QImage((uchar *)mat.data, mat.cols, mat.rows,
                                        mat.step, QImage::Format_RGBA8888));
          {
            std::unique_lock<std::mutex> lock(_mutex);
            _pixmap = pixmap;
            _pixmap_time = _image_time;
          }
          ready();
        }
      });
    }
    ~MessageBuffer() {
      {
        std::unique_lock<std::mutex> lock(_mutex);
        _stop = true;
        _put_condition.notify_one();
      }
      _worker.join();
    }
    void putImage(const std::shared_ptr<Message> &msg) {
      std::unique_lock<std::mutex> lock(_mutex);
      _image = nullptr;
      _image_time = ros::Time();
      if (msg) {
        if (auto image = msg->instantiate<sensor_msgs::Image>()) {
          _image = image;
          _image_time = msg->time();
          _put_condition.notify_one();
        }
      }
      _put_condition.notify_one();
    }
    void fetchPixmap(QPixmap *pixmap, ros::Time *time) {
      std::unique_lock<std::mutex> lock(_mutex);
      *pixmap = _pixmap;
      *time = _pixmap_time;
    }
  };
  std::shared_ptr<MessageBuffer> buffer = std::make_shared<MessageBuffer>();

  class GraphicsView : public QGraphicsView {
    class GraphicsScene : public QGraphicsScene {
      GraphicsView *_parent = nullptr;
      QGraphicsRectItem *_selection_rect = nullptr;
      // bool _press_already_accepted = false;

    protected:
      virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
        QGraphicsScene::mousePressEvent(event);
        // LOG_DEBUG("mouse press accepted " << event->isAccepted());
        if (event->button() == Qt::LeftButton) {
          //_press_already_accepted = event->isAccepted();
          if (event->modifiers() == 0 && !event->isAccepted()) {
            // LOG_DEBUG("clear selection");
            LockScope ws;
            ws->selection().clear();
            ws->modified();
          }
          event->accept();
        }
      }
      virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
        QGraphicsScene::mouseMoveEvent(event);
        // LOG_DEBUG("mouse move accepted " << event->isAccepted());
        if (_parent->_parent->annotation_type == nullptr) {
          if (event->buttons() == Qt::LeftButton && !event->isAccepted()) {
            event->accept();
            auto a = event->buttonDownScenePos(Qt::LeftButton);
            auto b = event->scenePos();
            _selection_rect->setRect(
                std::min(a.x(), b.x()), std::min(a.y(), b.y()),
                std::abs(a.x() - b.x()), std::abs(a.y() - b.y()));
            if (_selection_rect->rect().width() > 1e-6 &&
                _selection_rect->rect().height() > 1e-6) {
              _selection_rect->show();
            }
          }
        }
      }
      virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
        QGraphicsScene::mouseReleaseEvent(event);
        // LOG_DEBUG("mouse release accepted " << event->isAccepted());
        // LOG_DEBUG("mouse release");
        if (event->button() == Qt::LeftButton && !event->isAccepted()) {
          event->accept();
          if (_selection_rect->isVisible()) {
            LockScope ws;
            // LOG_DEBUG("rectangle selection");
            /*if (event->modifiers() == 0) {
              LOG_DEBUG("clear selection");
              ws->selection().clear();
          }*/
            QPainterPath selection_path;
            selection_path.addRect(_selection_rect->rect());
            for (auto &p : _parent->_parent->annotation_views) {
              auto *view = p.second;
              // LOG_DEBUG("checking annotation");
              if (selection_path.intersects(
                      view->shape().translated(view->pos()))) {
                // LOG_DEBUG("annotation selected");
                ws->selection().add(p.first);
              }
            }
            ws->modified();
            _selection_rect->hide();
          }
        }
      }

    public:
      GraphicsScene(GraphicsView *parent)
          : QGraphicsScene(parent), _parent(parent) {
        _selection_rect = new QGraphicsRectItem();
        _selection_rect->hide();
        _selection_rect->setBrush(QBrush(Qt::transparent));
        _selection_rect->setPen(QPen(QBrush(Qt::red), 0));
        _selection_rect->setZValue(10);
        addItem(_selection_rect);
      }
    };
    ImageWindow *_parent = nullptr;
    std::shared_ptr<MessageBuffer> _buffer;
    GraphicsScene *_scene = nullptr;
    QPixmap _image_pixmap;
    ros::Time _pixmap_time;
    QGraphicsPixmapItem *_image_item = nullptr;
    double windowScale() const {
      return std::min(width() * 1.0 / std::max(1, _image_pixmap.width()),
                      height() * 1.0 / std::max(1, _image_pixmap.height()));
    }
    double zoomFactor() const { return windowScale() * _parent->zoom(); }

  private:
    void sync() {
      // scene()->setSceneRect(_image_pixmap.rect());
      {
        double s = 1000000;
        scene()->setSceneRect(-s, -s, 2 * s, 2 * s);
      }
      _image_item->setPixmap(_image_pixmap);
      {
        LockScope ws;
        /*if (_parent->new_annotation && _parent->new_annotation_span &&
            ws()->selection().empty()) {
          _parent->new_annotation = nullptr;
      }*/
        {
          resetTransform();
          double zoom = zoomFactor();
          scale(zoom, zoom);
          centerOn(_parent->center().x() * _image_pixmap.width(),
                   _parent->center().y() * _image_pixmap.height());
        }
        for (auto &pair : _parent->annotation_views) {
          pair.second->ok = false;
        }
        if (ws->player) {
          if (auto timeline = ws->document()->timeline()) {
            double current_time =
                (_pixmap_time - ws->player->startTime()).toSec() + 1e-6;
            for (auto &track_base : timeline->tracks()) {
              if (auto track =
                      std::dynamic_pointer_cast<AnnotationTrack>(track_base)) {
                if (auto branch = track->branch()) {
                  for (auto &span : branch->spans()) {
                    if (span->start() > current_time ||
                        span->start() + span->duration() <= current_time) {
                      continue;
                    }
                    for (auto &annotation_base : span->annotations()) {
                      if (auto annotation =
                              std::dynamic_pointer_cast<ImageAnnotationBase>(
                                  annotation_base)) {
                        auto &view = _parent->annotation_views[annotation];
                        if (view == nullptr) {
                          view = new AnnotationView(_parent, ws(), track, span,
                                                    annotation);
                          scene()->addItem(view);
                        }
                        view->ok = true;
                        double zoom_factor = zoomFactor();
                        view->sync(ws(), zoom_factor);
                      }
                    }
                  }
                }
              }
            }
          }
        }
        if (auto annotation = _parent->new_annotation) {
          auto &view = _parent->annotation_views[annotation];
          view->ok = true;
          double zoom_factor = zoomFactor();
          view->sync(ws(), zoom_factor);
        }
        for (auto it = _parent->annotation_views.begin();
             it != _parent->annotation_views.end();) {
          if (it->second->ok) {
            it++;
          } else {
            delete it->second;
            it = _parent->annotation_views.erase(it);
          }
        }
      }
      update();
    }

  protected:
    virtual void resizeEvent(QResizeEvent *event) override {
      QGraphicsView::resizeEvent(event);
      sync();
    }

  public:
    GraphicsView(const std::shared_ptr<MessageBuffer> &buffer,
                 ImageWindow *parent)
        : _buffer(buffer), _parent(parent) {
      _scene = new GraphicsScene(this);
      setCacheMode(QGraphicsView::CacheNone);
      setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
      setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      setRenderHints(QPainter::Antialiasing);
      setMouseTracking(true);
      setFocusPolicy(Qt::ClickFocus);
      setScene(_scene);
      _scene->setBackgroundBrush(Qt::black);
      _image_item = new QGraphicsPixmapItem();
      _image_item->setTransformationMode(Qt::SmoothTransformation);
      _scene->addItem(_image_item);
      buffer->ready.connect(this, [this]() {
        // TODO: is this safe?
        QObject o;
        QObject::connect(&o, &QObject::destroyed, this, [this](QObject *o) {
          _buffer->fetchPixmap(&_image_pixmap, &_pixmap_time);
          sync();
        });
      });
      LockScope()->modified.connect(this, [this]() { sync(); });
      // grabShortcut(Qt::Key_Escape, Qt::WidgetShortcut);
    }
    void focusOutEvent(QFocusEvent *event) {
      // LOG_DEBUG("focus lost");
      QGraphicsView::focusOutEvent(event);
      _parent->new_annotation = nullptr;
      sync();
    }
    /*void keyPressEvent(QKeyEvent *event) {
      QGraphicsView::keyPressEvent(event);
      LOG_DEBUG("key press");
      if (_parent->new_annotation) {
        if (event->key() == Qt::Key_Escape) {
          _parent->new_annotation = nullptr;
          sync();
        }
      }
    }*/
    void wheelEvent(QWheelEvent *wheel) {
      double degrees = wheel->angleDelta().y() * (1.0 / 8);
      double exponent = degrees / 90;
      double factor = std::pow(0.5, -exponent);
      {
        LockScope ws;
        auto mouse_pos = mapToScene(_last_mouse_pos);
        _parent->zoom() *= factor;
        _parent->zoom() =
            std::min(maxZoom(), std::max(minZoom(), _parent->zoom()));
        sync();
        auto d = (mouse_pos - mapToScene(_last_mouse_pos));
        _parent->center().x() += d.x() / std::max(1, _image_pixmap.width());
        _parent->center().y() += d.y() / std::max(1, _image_pixmap.height());
        ws->modified();
      }
    }
    QPoint _last_mouse_pos = QPoint(0, 0);
    QPoint _middle_down_pos = QPoint(0, 0);
    Eigen::Vector2d snap(const QPoint &p) {
      if (_parent->new_annotation &&
          _parent->new_annotation->controlPoints().size() > 2) {
        auto q = _parent->new_annotation->controlPoints().front();
        double distance = (mapFromScene(q.x(), q.y()) - p).manhattanLength();
        if (distance <= 15) {
          return q;
        }
      }
      auto q = mapToScene(p);
      return Eigen::Vector2d(q.x(), q.y());
    }
    virtual void mouseMoveEvent(QMouseEvent *event) override {
      if (_parent->new_annotation) {
        event->accept();
        LockScope ws;
        auto p = snap(event->pos());
        _parent->new_annotation->controlPoints().back().x() = p.x();
        _parent->new_annotation->controlPoints().back().y() = p.y();
        _parent->new_annotation->constrain();
        sync();
      }
      QGraphicsView::mouseMoveEvent(event);
      event->accept();
      if (event->buttons() == Qt::RightButton ||
          event->buttons() == Qt::MiddleButton) {
        LockScope ws;
        auto d = mapToScene(event->pos()) - mapToScene(_last_mouse_pos);
        _parent->center().x() -=
            d.x() * 1.0 / std::max(1, _image_pixmap.width());
        _parent->center().y() -=
            d.y() * 1.0 / std::max(1, _image_pixmap.height());
        ws->modified();
      }

      _last_mouse_pos = event->pos();
    }

    virtual void mousePressEvent(QMouseEvent *event) override {
      if (_parent->new_annotation) {
        event->accept();
      }
      QGraphicsView::mousePressEvent(event);
      event->accept();
      if (event->button() == Qt::MiddleButton) {
        _middle_down_pos = event->pos();
      }
      if (event->button() == Qt::RightButton) {
        if (_parent->new_annotation) {
          _parent->new_annotation = nullptr;
          sync();
        }
      }
      if (event->button() == Qt::LeftButton) {
        auto scene_pos = mapToScene(event->pos());
        LockScope ws;
        if (auto timeline = ws->document()->timeline()) {
          if (ws->player) {
            if (_parent->annotation_type) {
              ActionScope ws("Annotate");
              double current_time =
                  (_pixmap_time - ws->player->startTime()).toSec() + 1e-6;
              std::shared_ptr<AnnotationTrack> current_track =
                  ws->currentAnnotationTrack().resolve(ws());
              if (!current_track) {
                for (auto &track_base : timeline->tracks()) {
                  if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                          track_base)) {
                    if (auto branch = track->branch()) {
                      for (auto &span : branch->spans()) {
                        if (span->start() <= current_time &&
                            span->start() + span->duration() >= current_time) {
                          current_track = track;
                          break;
                        }
                      }
                      if (current_track) {
                        break;
                      }
                    }
                  }
                }
              }
              if (!current_track) {
                for (auto &track_base : timeline->tracks()) {
                  if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                          track_base)) {
                    current_track = track;
                    break;
                  }
                }
              }
              if (!current_track) {
                timeline->tracks().push_back(
                    current_track = std::make_shared<AnnotationTrack>());
              }
              ws->currentAnnotationTrack() = current_track;
              // ws->selection() = current_track;
              std::shared_ptr<AnnotationSpan> current_span;
              if (auto branch = current_track->branch()) {
                for (auto &span : branch->spans()) {
                  if (span->start() <= current_time &&
                      span->start() + span->duration() >= current_time) {
                    current_span = span;
                    break;
                  }
                }
              }
              if (current_span == nullptr) {
                current_span = std::make_shared<AnnotationSpan>();
                current_span->start() = current_time;
                current_span->duration() = 0.1;
                {
                  double start = 0;
                  double duration = 0;
                  if (ws->player->findMessageTimeSpan(
                          _parent->topic(), current_time, &start, &duration)) {
                    // LOG_DEBUG("message time span " << start << " " <<
                    // duration);
                    current_span->start() = start;
                    current_span->duration() = duration;
                  } else {
                    // LOG_DEBUG("no current message");
                  }
                }
                current_track->branch(true)->spans().push_back(current_span);
              }
              ws->selection() = current_span;
              _parent->new_annotation_span = current_span;
              if (!_parent->new_annotation) {
                _parent->new_annotation =
                    _parent->annotation_type
                        ->instantiate<ImageAnnotationBase>();
                _parent->new_annotation->controlPoints().emplace_back(
                    scene_pos.x(), scene_pos.y());
                _parent->new_annotation->constrain();
                _parent->new_annotation->render();
                auto &view = _parent->annotation_views[_parent->new_annotation];
                view =
                    new AnnotationView(_parent, ws(), current_track,
                                       current_span, _parent->new_annotation);
                scene()->addItem(view);
              }
              if (_parent->new_annotation->complete) {
                ActionScope ws("New Annotation");
                if (_parent->new_annotation->controlPoints().size() >= 2 &&
                    _parent->new_annotation->controlPoints().front() ==
                        _parent->new_annotation->controlPoints().back()) {
                  _parent->new_annotation->controlPoints().pop_back();
                }
                current_span->annotations().push_back(_parent->new_annotation);
                ws->selection() = _parent->new_annotation;
                _parent->new_annotation = nullptr;
              } else {
                auto point = snap(event->pos());
                _parent->new_annotation->controlPoints().emplace_back(point);
                _parent->new_annotation->constrain();
              }
              ws->modified();
            }
          }
        }
      }
      _last_mouse_pos = event->pos();
    }
    virtual void mouseReleaseEvent(QMouseEvent *event) override {
      if (_parent->new_annotation) {
        {
          LockScope ws;
          _parent->new_annotation->render();
          if (_parent->new_annotation->complete &&
              _parent->new_annotation->controlPoints().size() == 2) {
            auto a = _parent->new_annotation->controlPoints().at(0);
            auto b = _parent->new_annotation->controlPoints().at(1);
            double distance =
                (mapFromScene(a.x(), a.y()) - mapFromScene(b.x(), b.y()))
                    .manhattanLength();
            LOG_DEBUG("diagonal " << distance);
            if (distance > 15) {
              ActionScope ws("New Annotation");
              _parent->new_annotation_span->annotations().push_back(
                  _parent->new_annotation);
              ws->selection() = _parent->new_annotation;
              _parent->new_annotation = nullptr;
              ws->modified();
            }
          }
        }
        event->accept();
      }
      QGraphicsView::mouseReleaseEvent(event);
      event->accept();
      _last_mouse_pos = event->pos();
    }
  };

  auto *view = new GraphicsView(buffer, this);
  // view->setInteractive(true);
  // view->setDragMode(QGraphicsView::ScrollHandDrag);
  setContentWidget(view);

  {
    LockScope ws;
    ws->modified.connect(this, [this, buffer]() {
      LockScope ws;
      if (topic().empty()) {
        subscriber = nullptr;
        buffer->putImage(nullptr);
      } else {
        if (!subscriber || subscriber->topic()->name() != topic()) {
          buffer->putImage(nullptr);
          subscriber = std::make_shared<Subscriber<Message>>(
              topic(), shared_from_this(),
              [buffer](const std::shared_ptr<Message> &msg) {
                buffer->putImage(msg);
              });
        }
      }
    });
  }
}
