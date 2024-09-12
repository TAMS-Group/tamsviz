// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "../render/opengl.h"

#include "imagewindow.h"

#include "../core/bagplayer.h"
#include "../core/log.h"
#include "../core/workspace.h"
#include "../core/image.h"

#include "../render/shader.h"

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tamsviz/InputEvent.h>

#include <QGraphicsScene>
#include <QGraphicsView>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QGLContext>

// #include <QGraphicsShaderEffect>

struct AnnotationViewBase : QGraphicsItem {
  bool ok = false;
  virtual void sync(const std::shared_ptr<Workspace> &ws, double zoom_factor) {}
};

// struct SpanAnnotationView : AnnotationViewBase {
//   QGraphicsTextItem text;
//   SpanAnnotationView(const std::string &label)
//       : text(QString::fromStdString(label), this) {}
// };

struct ImageAnnotationView : AnnotationViewBase {
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
    ImageAnnotationView *_parent = nullptr;
    size_t _control_point_index = 0;
    ControlPointHandle(ImageAnnotationView *parent, size_t control_point_index)
        : QGraphicsEllipseItem(parent),
          _parent(parent),
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

  ImageAnnotationView(ImageWindow *window, const std::shared_ptr<Workspace> &ws,
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
  virtual void sync(const std::shared_ptr<Workspace> &ws,
                    double zoom_factor) override {
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
    if (_window->annotation_type) {
      return;
    }
    if (event->buttons() == Qt::LeftButton) {
      for (auto &p : _window->annotation_views) {
        if (auto *view = dynamic_cast<ImageAnnotationView *>(p.second)) {
          if (view->_selected) {
            view->setPos(view->pos() +
                         (event->scenePos() - event->lastScenePos()));
          }
        }
      }
    }
  }
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override {
    if (_window->annotation_type) {
      return;
    }
    if (event->button() == Qt::LeftButton) {
      if (event->scenePos() != event->buttonDownScenePos(Qt::LeftButton)) {
        ActionScope ws("Move Annotations");
        auto delta =
            (event->scenePos() - event->buttonDownScenePos(Qt::LeftButton));
        for (auto &p : _window->annotation_views) {
          if (auto *view = dynamic_cast<ImageAnnotationView *>(p.second)) {
            if (view->_selected) {
              auto annotation = p.first;
              for (auto &point : annotation->controlPoints()) {
                point.x() += delta.x();
                point.y() += delta.y();
              }
            }
          }
        }
        ws->modified();
      }
    }
  }
};

class ImageWindowMessageBuffer {
  std::mutex _mutex;
  bool _pending = false;
  std::shared_ptr<const Message> _image;
  std::condition_variable _put_condition;
  std::thread _worker;
  // QPixmap _pixmap;
  bool _stop = false;
  ros::Time _image_time, _out_time;
  // ImageWindowOptions _options_in, _out_opts;
  cv::Mat _out_mat;
  std::string _out_encoding;
  std_msgs::Header _out_header;

  static void removeNotFinite(cv::Mat &mat) {
    switch (mat.type()) {
      case CV_32FC1:
      case CV_32FC2:
      case CV_32FC3:
      case CV_32FC4:
        for (size_t y = 0; y < mat.rows; y++) {
          auto *pixel = mat.ptr<float>(y);
          size_t n = mat.cols * mat.channels();
          for (size_t x = 0; x < n; x++) {
            if (!std::isfinite(*pixel)) {
              *pixel = 0;
            }
            pixel++;
          }
        }
        break;
      case CV_64FC1:
      case CV_64FC2:
      case CV_64FC3:
      case CV_64FC4:
        for (size_t y = 0; y < mat.rows; y++) {
          auto *pixel = mat.ptr<double>(y);
          size_t n = mat.cols * mat.channels();
          for (size_t x = 0; x < n; x++) {
            if (!std::isfinite(*pixel)) {
              *pixel = 0;
            }
            pixel++;
          }
        }
        break;
    }
  }

  // static bool msg2mat(const std::shared_ptr<const Message> &image, cv::Mat
  // &mat,
  //                     std::string &encoding, std_msgs::Header &header) {
  //   if (auto img = image->instantiate<sensor_msgs::Image>()) {
  //     header = img->header;
  //     if (sensor_msgs::image_encodings::isBayer(img->encoding) ||
  //         img->encoding == sensor_msgs::image_encodings::YUV422) {
  //       try {
  //         cv_bridge::CvImageConstPtr cv_ptr =
  //             cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::RGB8);
  //         mat = cv_ptr->image;
  //         encoding = cv_ptr->encoding;
  //         return true;
  //       } catch (cv_bridge::Exception &e) {
  //         LOG_ERROR("cv_bridge exception: " << e.what());
  //       }
  //     }
  //     try {
  //       cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*img);
  //       mat = cv_ptr->image;
  //       encoding = cv_ptr->encoding;
  //       return true;
  //     } catch (cv_bridge::Exception &e) {
  //       LOG_ERROR("cv_bridge exception: " << e.what());
  //     }
  //   } else if (auto img = image->instantiate<sensor_msgs::CompressedImage>())
  //   {
  //     header = img->header;
  //     try {
  //       cv_bridge::CvImageConstPtr cv_ptr =
  //           cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::RGB8);
  //       mat = cv_ptr->image;
  //       encoding = cv_ptr->encoding;
  //       return true;
  //     } catch (cv_bridge::Exception &e) {
  //       LOG_ERROR("cv_bridge exception: " << e.what());
  //     }
  //   } else {
  //     LOG_WARN("unsupported image message type" << image->type()->name());
  //   }
  //   if (mat.empty()) {
  //     LOG_WARN("failed to decode image");
  //   }
  //   header = std_msgs::Header();
  //   return false;
  // }

 public:
  Event<void()> ready;
  ImageWindowMessageBuffer() {
    _worker = std::thread([this]() {
      while (true) {
        std::shared_ptr<const Message> image;
        // ImageWindowOptions options;
        {
          std::unique_lock<std::mutex> lock(_mutex);
          while (true) {
            if (_stop) {
              return;
            }
            if (_pending) {
              _pending = false;
              image = _image;
              // options = _options_in;
              break;
            }
            _put_condition.wait(lock);
          }
        }
        if (image == nullptr) {
          {
            std::unique_lock<std::mutex> lock(_mutex);
            // _pixmap = QPixmap();
            _out_mat = cv::Mat();
            _out_encoding = "";
            _out_header = std_msgs::Header();
          }
          continue;
        }

        PROFILER("image processing thread");

        // LOG_DEBUG("imagewindow start processing");

        cv::Mat mat;
        std::string encoding;
        std_msgs::Header header;
        bool ok = tryConvertImageMessageToMat(*image, mat, encoding, header);
        if (!ok) {
          LOG_WARN_THROTTLE(1, "failed to convert image");
          continue;
        }

        removeNotFinite(mat);

        // auto pixmap = QPixmap::fromImage(mat2image(mat, encoding));
        {
          PROFILER("image processing sync");
          std::unique_lock<std::mutex> lock(_mutex);
          // _pixmap = pixmap;
          _out_mat = mat;
          _out_time = _image_time;
          // _out_opts = options;
          _out_encoding = encoding;
          _out_header = header;
        }

        // LOG_DEBUG("imagewindow processing finished, image size "
        //           << pixmap.width() << " x " << pixmap.height());

        // LOG_DEBUG("imagewindow processing finished, image size "
        //           << mat.cols << " x " << mat.rows);

        {
          PROFILER("image processing ready");
          ready();
        }
      }
    });
  }
  ~ImageWindowMessageBuffer() {
    {
      std::unique_lock<std::mutex> lock(_mutex);
      _stop = true;
      _put_condition.notify_one();
    }
    _worker.join();
  }
  void putImage(const std::shared_ptr<const Message> &msg) {
    // LOG_DEBUG("imagewindow queue put");
    PROFILER();
    std::unique_lock<std::mutex> lock(_mutex);
    _pending = true;
    _image = nullptr;
    _image_time = ros::Time();
    if (msg) {
      _image = msg;
      _image_time = msg->time();
    }
    _put_condition.notify_one();
  }
  // void refresh(const ImageWindowOptions &options) {
  //   LOG_DEBUG("imagewindow queue refresh");
  //   std::unique_lock<std::mutex> lock(_mutex);
  //   _options_in = options;
  //   _pending = true;
  //   _put_condition.notify_one();
  // }
  // void fetchPixmap(QPixmap *pixmap, ros::Time *time,
  //                  ImageWindowOptions *options) {
  //   LOG_DEBUG("imagewindow queue fetch");
  //   std::unique_lock<std::mutex> lock(_mutex);
  //   *pixmap = _pixmap;
  //   *time = _pixmap_time;
  //   *options = _options_out;
  // }
  void pull(cv::Mat *mat, ros::Time *time,  // ImageWindowOptions *options,
            std::string *encoding, std_msgs::Header *header) {
    // LOG_DEBUG("imagewindow queue fetch");
    PROFILER();
    std::unique_lock<std::mutex> lock(_mutex);
    if (mat) *mat = _out_mat;
    if (time) *time = _out_time;
    if (encoding) *encoding = _out_encoding;
    if (header) *header = _out_header;
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
      menu->clear();
      auto topics = LockScope()->listTopics({
          "sensor_msgs/Image",
          "sensor_msgs/CompressedImage",
      });
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
      QString label = "Interact";
      button->setText(label);
      connect(menu->addAction(label), &QAction::triggered, this,
              [label, button, this](bool checked) {
                LockScope ws;
                annotation_mode = false;
                annotation_type = nullptr;
                button->setText(label);
                ws->modified();
              });
    }
    {
      QString label = "Select / Edit";
      button->setText(label);
      connect(menu->addAction(label), &QAction::triggered, this,
              [label, button, this](bool checked) {
                LockScope ws;
                annotation_mode = true;
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
                annotation_mode = true;
                annotation_type = type;
                button->setText(label);
                ws->modified();
              });
    }
  }

  std::shared_ptr<ImageWindowMessageBuffer> buffer =
      std::make_shared<ImageWindowMessageBuffer>();

  _message_buffer = buffer;

  // _refresh_callback = [buffer](const ImageWindowOptions &options) {
  //   // buffer->refresh(options);
  // };

  {
    auto *button = new FlatButton();
    button->setIcon(MATERIAL_ICON("photo_camera", 0.0));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this, buffer]() {
      // QPixmap screenshot;
      cv::Mat screenshot;
      ros::Time time;
      ImageWindowOptions opts;
      std::string encoding;
      buffer->pull(&screenshot, &time, &encoding, nullptr);
      if (screenshot.empty()) {
        QMessageBox::warning(nullptr, "Error",
                             "No image received. Select image topic.");
        return;
      }
      std::string namesuggest;
      {
        LockScope ws;
        namesuggest = topic();
      }
      while (namesuggest.size() && !std::isalnum(namesuggest.front())) {
        namesuggest = namesuggest.substr(1);
      }
      for (auto &c : namesuggest) {
        if (!std::isalnum(c)) {
          c = '_';
        }
      }
      {
        LockScope ws;
        if (ws->player) {
          std::string bagname = ws->player->fileName();
          for (auto &c : bagname) {
            if (!std::isalnum(c)) {
              c = '_';
            }
          }
          namesuggest = QFileInfo(ws->player->path().c_str())
                            .absoluteDir()
                            .filePath((bagname + "-" + namesuggest).c_str())
                            .toStdString();
          namesuggest +=
              "_" + std::to_string(int64_t(ws->player->time() * 1000));
        }
      }
      namesuggest =
          namesuggest + "_" + std::to_string(ros::WallTime::now().toNSec());
      namesuggest += ".png";
      QString file_name = QFileDialog::getSaveFileName(
          this, tr("Save Screenshot"), namesuggest.c_str(),
          tr("Images (*.png *.jpg)"));
      if (!file_name.isEmpty()) {
        if (QFileInfo(file_name).suffix().isEmpty()) {
          if (!file_name.endsWith(".")) {
            file_name += ".";
          }
          file_name += "png";
        }
        // if (screenshot.save(file_name)) {
        if (cv::imwrite(file_name.toStdString().c_str(), screenshot)) {
          LOG_SUCCESS("screenshot saved: " << file_name.toStdString());
        } else {
          LOG_ERROR("failed to save screenshot: " << file_name.toStdString());
          QMessageBox::warning(nullptr, "Error", "Failed to save image.");
        }
      }
    });
    addToolWidgetRight(button);
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

  class GraphicsPixmapItemGL : public QGraphicsRectItem {
    // std::unique_ptr<Shader> _shader;
    // std::unique_ptr<QOpenGLTexture> _texture;
    std::unique_ptr<Texture> _texture;
    cv::Mat _mat;
    ImageWindowOptions _options;
    std::string _encoding;
    bool _dirty = true;

   public:
    size_t width() const { return _mat.cols; }
    size_t height() const { return _mat.rows; }
    void setImage(const cv::Mat &mat, const std::string &encoding) {
      // return;
      _mat = mat;
      _encoding = encoding;
      _dirty = true;
      auto r = rect();
      r.setWidth(mat.cols);
      r.setHeight(mat.rows);
      setRect(r);
    }
    void setOptions(const ImageWindowOptions &options) {
      _options = options;
      // update();
    }
    virtual void paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) override {
      // return;

      PROFILER();

      if (_mat.empty()) {
        return;
      }

      // LOG_DEBUG("paint image " << this << " begin");

      // return;

      // LOG_INFO(typeid(painter->device()).name());
      painter->beginNativePainting();

      bool texture_2d_was_enabled = false;
      V_GL(texture_2d_was_enabled = glIsEnabled(GL_TEXTURE_2D));
      V_GL(glEnable(GL_TEXTURE_2D));

      GLint old_texture = 0;
      V_GL(glGetIntegerv(GL_TEXTURE_BINDING_2D, &old_texture));

      GLint old_program = 0;
      V_GL(glGetIntegerv(GL_CURRENT_PROGRAM, &old_program));

      static std::unique_ptr<Shader> _shader;
      if (!_shader) {
        _shader.reset(new Shader(
            "package://" ROS_PACKAGE_NAME "/shaders/image_vert.glsl",
            "package://" ROS_PACKAGE_NAME "/shaders/image_frag.glsl", false));
      }
      // glClearColor(1, 0, 0, 1);
      // glClear(GL_COLOR_BUFFER_BIT);
      if (!_texture) {
        _texture.reset(new Texture(TextureType::Linear));
      }
      // QOpenGLTexture tex(pixmap().toImage());

      // _texture->update(_mat);
      if (_dirty) {
        _dirty = false;
        // LOG_WARN("uploading image");
        V_GL(glActiveTexture(GL_TEXTURE0));
        V_GL(glBindTexture(GL_TEXTURE_2D, _texture->id()));
        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP));
        V_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP));
        switch (_mat.elemSize()) {
          case 1:
            V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
            break;
          case 2:
            V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 2));
            break;
          case 3:
            V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
            break;
          default:
            V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 4));
            break;
        }
        V_GL(glPixelStorei(GL_UNPACK_ROW_LENGTH, _mat.step / _mat.elemSize()));

        if (_encoding == sensor_msgs::image_encodings::RGB8) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, _mat.cols, _mat.rows, 0,
                            GL_RGB, GL_UNSIGNED_BYTE, _mat.data));

        } else if (_encoding == sensor_msgs::image_encodings::BGR8) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, _mat.cols, _mat.rows, 0,
                            GL_BGR, GL_UNSIGNED_BYTE, _mat.data));

        } else if (_encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, _mat.cols, _mat.rows, 0,
                            GL_RED, GL_UNSIGNED_BYTE, _mat.data));

        } else if (_encoding == sensor_msgs::image_encodings::TYPE_8UC2) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, _mat.cols, _mat.rows, 0,
                            GL_RG, GL_UNSIGNED_BYTE, _mat.data));

        } else if (_encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, _mat.cols, _mat.rows, 0,
                            GL_RGB, GL_UNSIGNED_BYTE, _mat.data));

        } else if (_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
          V_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, _mat.cols, _mat.rows, 0,
                            GL_RED, GL_FLOAT, _mat.data));

        } else {
          LOG_ERROR_THROTTLE(1, "unsupported image format " << _encoding);
        }

        V_GL(glPixelStorei(GL_UNPACK_ROW_LENGTH, 0));
        V_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, 4));
      } else {
        // LOG_SUCCESS("keeping image texture");
      }

#if 1

      {
        _shader->use();
        V_GL(glUniform1f(glGetUniformLocation(_shader->program(), "brightness"),
                         _options.brightness));
        V_GL(glUniform1f(glGetUniformLocation(_shader->program(), "saturation"),
                         _options.saturation));
        V_GL(
            glUniform1i(glGetUniformLocation(_shader->program(), "tonemapping"),
                        (int)_options.toneMapping));
        V_GL(glUniform1i(glGetUniformLocation(_shader->program(), "channels"),
                         (int)_mat.channels()));
      }

      auto rect = boundingRect();
      // LOG_INFO(rect.left() << " " << rect.top() << " " << rect.right() << " "
      //                      << rect.bottom());
      ((QGLContext *)QGLContext::currentContext())
          ->drawTexture(rect, _texture->id());
      V_GL(glBindTexture(GL_TEXTURE_2D, 0));

#endif

      // auto rect = boundingRect();
      // float rect[4 * 2] = {
      //     rect.left(),  rect.top(),     //
      //     rect.right(), rect.top(),     //
      //     rect.right(), rect.bottom(),  //
      //     rect.left(),  rect.bottom(),  //
      // };

      V_GL(glBindTexture(GL_TEXTURE_2D, old_texture));
      if (!texture_2d_was_enabled) V_GL(glDisable(GL_TEXTURE_2D));

      V_GL(glUseProgram(old_program));

      painter->endNativePainting();

      // QGraphicsPixmapItem::paint(painter, option, widget);

      // LOG_DEBUG("paint image " << this << " ready");
    }
  };

  class GraphicsView : public QGraphicsView {
    class GraphicsScene : public QGraphicsScene {
      GraphicsView *_parent = nullptr;
      QGraphicsRectItem *_selection_rect = nullptr;

     protected:
      virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override {
        QGraphicsScene::mousePressEvent(event);
        if (event->button() == Qt::LeftButton) {
          if (event->modifiers() == 0 && !event->isAccepted()) {
            LockScope ws;
            ws->selection().clear();
            ws->modified();
          }
          event->accept();
        }
      }
      virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override {
        QGraphicsScene::mouseMoveEvent(event);
        if (_parent->_parent->annotation_mode &&
            _parent->_parent->annotation_type == nullptr) {
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
        if (event->button() == Qt::LeftButton && !event->isAccepted()) {
          event->accept();
          if (_selection_rect->isVisible()) {
            LockScope ws;
            QPainterPath selection_path;
            selection_path.addRect(_selection_rect->rect());
            for (auto &p : _parent->_parent->annotation_views) {
              auto *view = p.second;
              if (selection_path.intersects(
                      view->shape().translated(view->pos()))) {
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

      virtual void drawForeground(QPainter *painter,
                                  const QRectF &rect) override {
        PROFILER();
        QGraphicsScene::drawForeground(painter, rect);
        painter->save();
        painter->resetTransform();
        _parent->_parent->paintAnnotationHUD(painter,
                                             _parent->_parent->annotation_type);
        {
          auto &text = _parent->_annotation_span_text;
          if (!text.isEmpty()) {
            auto rect = QRect(QPoint(0, 0), painter->window().size());
            rect = rect.marginsRemoved(QMargins(0, 0, 0, 7));
            auto flags = Qt::AlignBottom | Qt::AlignHCenter;
            auto trect = painter->fontMetrics().boundingRect(rect, flags, text);
            trect = trect.marginsAdded(QMargins(5, 2, 5, 2));
            painter->fillRect(trect, QBrush(QColor(0, 0, 0, 150)));
            painter->setPen(QPen(QBrush(Qt::white), 0));
            painter->drawText(rect, flags, text);
          }
        }
        painter->restore();
      }
    };
    ImageWindow *_parent = nullptr;
    std::shared_ptr<ImageWindowMessageBuffer> _buffer;
    GraphicsScene *_scene = nullptr;
    cv::Mat _image_mat;
    std_msgs::Header _image_header;
    ros::Time _image_time;
    std::string _image_encoding;
    QString _annotation_span_text;
    GraphicsPixmapItemGL *_image_item = nullptr;
    double windowScale() const {
      return std::min(width() * 1.0 / std::max(1, _image_mat.cols),
                      height() * 1.0 / std::max(1, _image_mat.rows));
    }
    double zoomFactor() const { return windowScale() * _parent->zoom(); }

   private:
    void sync() {
      // LOG_DEBUG("image sync begin");
      PROFILER();
      if (QApplication::instance()->thread() != QThread::currentThread()) {
        throw std::runtime_error("image view sync called on background thread");
      }
      {
        double s = 1000000;
        scene()->setSceneRect(-s, -s, 2 * s, 2 * s);
      }
      // _image_item->update(_image_mat, _image_options, _image_encoding);
      _annotation_span_text.clear();
      {
        LockScope ws;
        _image_item->setOptions(_parent->options());
        {
          resetTransform();
          double zoom = zoomFactor();
          scale(zoom, zoom);
          centerOn(_parent->center().x() * _image_mat.cols,
                   _parent->center().y() * _image_mat.rows);
        }
        for (auto &pair : _parent->annotation_views) {
          pair.second->ok = false;
        }
        if (ws->player) {
          if (auto timeline = ws->document()->timeline()) {
            double current_time =
                (_image_time - ws->player->startTime()).toSec() + 1e-6;
            for (auto &track_base : timeline->tracks()) {
              if (auto track =
                      std::dynamic_pointer_cast<AnnotationTrack>(track_base)) {
                if (auto branch = track->branch(ws(), false)) {
                  for (auto &span : branch->spans()) {
                    if (span->start() > current_time ||
                        span->start() + span->duration() <= current_time) {
                      continue;
                    }
                    if (!_annotation_span_text.isEmpty()) {
                      _annotation_span_text += ", ";
                    }
                    _annotation_span_text += QString::fromStdString(
                        (span->label().empty() ? track->label()
                                               : span->label()));
                    for (auto &annotation_base : span->annotations()) {
                      if (auto annotation =
                              std::dynamic_pointer_cast<ImageAnnotationBase>(
                                  annotation_base)) {
                        if (annotation->topic() != _parent->topic()) {
                          continue;
                        }
                        auto &view = _parent->annotation_views[annotation];
                        if (view == nullptr) {
                          view = new ImageAnnotationView(_parent, ws(), track,
                                                         span, annotation);
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
      // LOG_DEBUG("image sync ready");
    }

   protected:
    virtual void resizeEvent(QResizeEvent *event) override {
      QGraphicsView::resizeEvent(event);
      sync();
    }

   public:
    GraphicsView(ImageWindow *parent)
        : _buffer(parent->_message_buffer), _parent(parent) {
      _scene = new GraphicsScene(this);
      setViewport(new QOpenGLWidget(this));
      setCacheMode(QGraphicsView::CacheNone);
      setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
      setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      setRenderHints(QPainter::Antialiasing);
      setMouseTracking(true);
      setFocusPolicy(Qt::ClickFocus);
      setScene(_scene);
      _scene->setBackgroundBrush(Qt::black);
      _image_item = new GraphicsPixmapItemGL();
      // _image_item->setTransformationMode(Qt::SmoothTransformation);
      // _image_item->setGraphicsEffect(new QGraphicsShaderEffect());
      _scene->addItem(_image_item);
      parent->_message_buffer->ready.connect(this, [this]() {
        PROFILER("image window buffer->ready");
        // TODO: is this safe?
        QObject o;
        QObject::connect(&o, &QObject::destroyed, this, [this](QObject *o) {
          //_buffer->fetchPixmap(&_image_pixmap, &_pixmap_time);
          // sync();
          PROFILER("image window buffer->ready 2");
          startOnMainThreadAsync([this]() {
            PROFILER("buffer->ready 3");
            _buffer->pull(&_image_mat, &_image_time,  // &_image_options,
                          &_image_encoding, &_image_header);
            _image_item->setImage(_image_mat, _image_encoding);
            sync();
          });
        });
      });
      LockScope()->modified.connect(this, [this]() {
        // sync();
        PROFILER("image window document modified");
        startOnMainThreadAsync([this]() {
          PROFILER("image window document modified sync");
          sync();
        });
      });
      viewport()->setMouseTracking(true);
    }
    void focusOutEvent(QFocusEvent *event) {
      QGraphicsView::focusOutEvent(event);
      _parent->new_annotation = nullptr;
      sync();
    }
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
        _parent->center().x() += d.x() / std::max(1, _image_mat.cols);
        _parent->center().y() += d.y() / std::max(1, _image_mat.rows);
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
    uint8_t convertMouseButtons(int qt) {
      uint8_t ret = 0;
      if (qt & Qt::LeftButton) ret |= tamsviz::InputEvent::LEFT_BUTTON;
      if (qt & Qt::RightButton) ret |= tamsviz::InputEvent::RIGHT_BUTTON;
      if (qt & Qt::MiddleButton) ret |= tamsviz::InputEvent::MIDDLE_BUTTON;
      return ret;
    }
    bool publishEvent(QEvent *event) {
      LockScope ws;
      if (_parent->annotation_mode) return false;
      if (!_parent->_event_publisher) return false;
      if (_parent->_event_publisher.getNumSubscribers() == 0) return false;
      std::string t;
      switch (event->type()) {
        case QEvent::MouseButtonPress:
          t = tamsviz::InputEvent::MOUSE_PRESS;
          break;
        case QEvent::MouseButtonRelease:
          t = tamsviz::InputEvent::MOUSE_RELEASE;
          break;
        case QEvent::MouseMove:
          t = tamsviz::InputEvent::MOUSE_MOVE;
          break;
        case QEvent::MouseButtonDblClick:
          t = tamsviz::InputEvent::DOUBLE_CLICK;
          break;
        case QEvent::Enter:
          t = tamsviz::InputEvent::MOUSE_ENTER;
          break;
        case QEvent::Leave:
          t = tamsviz::InputEvent::MOUSE_LEAVE;
          break;
        default:
          LOG_DEBUG("unidentified event " << event->type());
          return false;
      }
      // event->accept();
      tamsviz::InputEvent m;
      m.event_type = t;
      m.image_topic = _parent->topic();
      m.window_name = _parent->name();
      if (auto *mouse_event = dynamic_cast<QMouseEvent *>(event)) {
        m.buttons_pressed = convertMouseButtons(mouse_event->buttons());
        m.event_button = convertMouseButtons(mouse_event->button());
        m.window_point.x = mouse_event->x();
        m.window_point.y = mouse_event->y();
        auto p = mapToScene(mouse_event->pos());
        m.image_point.x = p.x();
        m.image_point.y = p.y();
      }
      m.window_size.width = _parent->width();
      m.window_size.height = _parent->height();
      m.image_size.width = _image_item->width();
      m.image_size.height = _image_item->height();
      m.header.frame_id = _image_header.frame_id;
      m.header.stamp = ros::Time::now();
      m.image_stamp = _image_header.stamp;
      LOG_DEBUG("publishing mouse event for " << _parent->name() << " "
                                              << _parent->topic());
      _parent->_event_publisher.publish(m);
      return true;
    }
    // virtual bool event(QEvent *event) override {
    //   bool ret = QGraphicsView::event(event);
    //   if (publishEvent(event)) ret = true;
    //   event->accept();
    //   ret = true;
    //   return ret;
    // }
    virtual void mouseDoubleClickEvent(QMouseEvent *event) override {
      QGraphicsView::mouseDoubleClickEvent(event);
      publishEvent(event);
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
        _parent->center().x() -= d.x() * 1.0 / std::max(1, _image_mat.cols);
        _parent->center().y() -= d.y() * 1.0 / std::max(1, _image_mat.rows);
        ws->modified();
      }
      _last_mouse_pos = event->pos();
      publishEvent(event);
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
                  (_image_time - ws->player->startTime()).toSec() + 1e-6;
              std::shared_ptr<AnnotationTrack> current_track =
                  ws->currentAnnotationTrack().resolve(ws());
              if (!current_track) {
                for (auto &track_base : timeline->tracks()) {
                  if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                          track_base)) {
                    if (auto branch = track->branch(ws(), false)) {
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
              std::shared_ptr<AnnotationSpan> current_span;
              if (auto branch = current_track->branch(ws(), false)) {
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
                    current_span->start() = start;
                    current_span->duration() = duration;
                  }
                }
                current_track->branch(ws(), true)
                    ->spans()
                    .push_back(current_span);
              }
              ws->selection() = current_span;
              _parent->new_annotation_span = current_span;
              if (!_parent->new_annotation) {
                _parent->new_annotation =
                    _parent->annotation_type
                        ->instantiate<ImageAnnotationBase>();
                _parent->new_annotation->topic() = _parent->topic();
                _parent->new_annotation->controlPoints().emplace_back(
                    scene_pos.x(), scene_pos.y());
                _parent->new_annotation->constrain();
                _parent->new_annotation->render();
                auto &view = _parent->annotation_views[_parent->new_annotation];
                view = new ImageAnnotationView(_parent, ws(), current_track,
                                               current_span,
                                               _parent->new_annotation);
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
      publishEvent(event);
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
            // LOG_DEBUG("diagonal " << distance);
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
      publishEvent(event);
    }
    virtual void leaveEvent(QEvent *event) override {
      QGraphicsView::leaveEvent(event);
      publishEvent(event);
    }
    virtual void enterEvent(QEvent *event) override {
      QGraphicsView::enterEvent(event);
      publishEvent(event);
    }
  };

  auto *view = new GraphicsView(this);
  setContentWidget(view);
}

void ImageWindow::refresh() {
  LOG_DEBUG("refresh");
  ContentWindowBase::refresh();
  this->window()->update();
  // if (_options_watcher.changed(options())) {
  //   if (_refresh_callback) {
  //     _refresh_callback(options());
  //   }
  // }
  if (topic().empty()) {
    subscriber = nullptr;
    _message_buffer->putImage(nullptr);
  } else {
    if (!subscriber || subscriber->topic()->name() != topic()) {
      _message_buffer->putImage(nullptr);
      auto buffer = _message_buffer;
      subscriber = std::make_shared<Subscriber<Message>>(
          topic(), shared_from_this(),
          [buffer](const std::shared_ptr<const Message> &msg) {
            buffer->putImage(msg);
          },
          false);
    }
  }
  // if (_event_publishing_watcher.changed(enableEventPublishing(),
  //                                       mouseEventSuffix(), topic())) {
  //   if (enableEventPublishing() && topic() != "" && mouseEventSuffix() != "")
  //   {
  //     std::string topic_name = topic() + mouseEventSuffix();
  //     LOG_DEBUG("advertising mouse event topic " << topic_name);
  //     ros::NodeHandle node_handle("~");
  //     _event_publisher =
  //         node_handle.advertise<tamsviz::InputEvent>(topic_name, 100, false);
  //   } else {
  //     LOG_DEBUG("mouse event publishing disabled");
  //     _event_publisher = ros::Publisher();
  //   }
  // }
  LockScope ws;
  if (_event_publishing_watcher.changed(
          ws->document()->display()->publishInputEvents())) {
    if (ws->document()->display()->publishInputEvents()) {
      LOG_DEBUG("advertising event topic");
      ros::NodeHandle node_handle;
      _event_publisher = node_handle.advertise<tamsviz::InputEvent>(
          "/tamsviz/input", 100, false);
    } else {
      LOG_DEBUG("mouse event publishing disabled");
      _event_publisher = ros::Publisher();
    }
  }
}
