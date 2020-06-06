// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "guicommon.h"

#include <ros/package.h>
#include <ros/ros.h>

static int g_icon_size = 128;
static int g_icon_alpha = 180;

void startOnMainThreadAsync(const std::function<void()> &fun) {
  QObject o;
  QObject::connect(&o, &QObject::destroyed, QCoreApplication::instance(),
                   [fun](QObject *o) { fun(); });
}

QIcon _createTextIcon(const QString &str) {
  int w = g_icon_size;
  int h = g_icon_size;
  QPixmap pixmap(w, h);
  pixmap.fill(QColor(255, 255, 255, 0));
  QFont font;
  font.setPixelSize(h * 0.8);
  {
    QPainter p(&pixmap);
    p.setFont(font);
    p.setPen(QColor(0, 0, 0, g_icon_alpha));
    p.drawText(QRect(0, 0, w, h), str,
               QTextOption(Qt::AlignHCenter | Qt::AlignVCenter));
  }
  int m = 2;
  pixmap = pixmap.copy(QGraphicsPixmapItem(pixmap)
                           .opaqueArea()
                           .boundingRect()
                           .toRect()
                           .marginsAdded(QMargins(m, m, m, m)));
  return QIcon(pixmap);
}

struct IconLoader {
  QFont font;
  QMap<QString, QChar> glyph_map;
  IconLoader(const std::string &font_path, const std::string &map_path) {
    font = [&]() {
      std::string path =

          font_path;
      int id = QFontDatabase::addApplicationFont(path.c_str());
      if (id < 0) {
        LOG_ERROR("failed to load font " << path);
        return QFont();
      }
      QString family = QFontDatabase::applicationFontFamilies(id).at(0);
      QFont font(family);
      LOG_DEBUG("font loaded " << family.toStdString() << " " << id << " "
                               << path);
      return font;
    }();
    glyph_map = [&]() {
      QFile file(map_path.c_str());
      file.open(QFile::ReadOnly);
      QMap<QString, QChar> map;
      for (auto &line : QString(file.readAll()).split("\n")) {
        auto tokens = line.trimmed().split(" ");
        if (tokens.size() == 2) {
          map[tokens[0]] = QChar(tokens[1].toInt(nullptr, 16));
        }
      }
      return map;
    }();
  }
  QIcon load(const QString &str, double margin) {
    int w = g_icon_size;
    int h = g_icon_size;
    QPixmap pixmap(w, h);
    pixmap.fill(QColor(255, 255, 255, 0));

    {
      LOG_DEBUG("icon glyph " << str.toStdString() << " "
                              << glyph_map[str].unicode());
      if (margin >= 0.0) {
        font.setPixelSize(h * 0.5);
      } else {
        font.setPixelSize(h);
      }
      {
        QPainter p(&pixmap);
        p.setFont(font);
        p.setPen(QColor(0, 0, 0, g_icon_alpha));
        p.drawText(QRect(0, 0, w, h), Qt::AlignHCenter | Qt::AlignVCenter,
                   glyph_map[str]);
      }
      if (margin >= 0.0) {
        auto rect =
            QGraphicsPixmapItem(pixmap).opaqueArea().boundingRect().toRect();
        int margins =
            std::round(std::sqrt(rect.width() * rect.height()) * margin);
        pixmap = pixmap.copy(
            rect.marginsAdded(QMargins(margins, margins, margins, margins)));
      } else {
        auto rect = pixmap.rect();
        int margins = std::round(std::sqrt(rect.width() * rect.height()) *
                                 std::abs(margin));
        pixmap = pixmap.copy(
            rect.marginsRemoved(QMargins(margins, margins, margins, margins)));
      }
    }
    return QIcon(pixmap);
  }
};

QIcon _createMaterialIcon(const QString &str, double padding) {
  static IconLoader loader(
      ros::package::getPath(ROS_PACKAGE_NAME) +
          "/3rdparty/material-design-icons/MaterialIcons-Regular.ttf",
      ros::package::getPath(ROS_PACKAGE_NAME) +
          "/3rdparty/material-design-icons/codepoints.txt");
  // return loader.load(str, 0.1);
  return loader.load(str, padding);
}

QIcon _create_FA_R_Icon(const QString &str, double padding) {
  static IconLoader loader(ros::package::getPath(ROS_PACKAGE_NAME) +
                               "/3rdparty/fontawesome/fa-regular-400.ttf",
                           ros::package::getPath(ROS_PACKAGE_NAME) +
                               "/3rdparty/fontawesome/codepoints.txt");
  // return loader.load(str, 0.6);
  return loader.load(str, padding);
}

QIcon _create_FA_S_Icon(const QString &str, double padding) {
  static IconLoader loader(ros::package::getPath(ROS_PACKAGE_NAME) +
                               "/3rdparty/fontawesome/fa-solid-900.ttf",
                           ros::package::getPath(ROS_PACKAGE_NAME) +
                               "/3rdparty/fontawesome/codepoints.txt");
  // return loader.load(str, 0.4);
  return loader.load(str, padding);
}

void FlatButton::init() {
  setFlat(true);
  setFocusPolicy(Qt::TabFocus);
  setAutoDefault(false);
  // setMinimumWidth(1);
  // setMinimumSize(1, 1);
  // setStyleSheet("* { border: none; }");
  // setStyleSheet("* { background: none; }");
  /*setPopupMode(InstantPopup);
  setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setStyleSheet("text-align: center;");*/
  // setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

FlatButton::~FlatButton() { // LOG_DEBUG("FlatButton::~FlatButton");
}

void FlatButton::paintEvent(QPaintEvent *event) {
  QStylePainter p(this);
  QStyleOptionButton option;
  initStyleOption(&option);
  p.drawControl(QStyle::CE_PushButton, option);
  // QPushButton::paintEvent(event);
}

QSize FlatButton::minimumSizeHint() const {
  // return QToolButton::minimumSizeHint();
  // return QPushButton::minimumSizeHint();
  return QAbstractButton::minimumSizeHint();
  // return QSize(0, 0);
}

QSize FlatButton::sizeHint() const {
  int width = QFontMetrics(font()).width(text()) +
              style()->pixelMetric(QStyle::PM_ButtonMargin) * 2;
  if (!icon().isNull() || menu() != nullptr) {
    width += style()->pixelMetric(QStyle::PM_ButtonMargin);
    width += style()->pixelMetric(QStyle::PM_ButtonIconSize);
  }
  return QSize(width, QPushButton::sizeHint().height());
  // return QAbstractButton::sizeHint();
  /*QStyleOptionButton opt;
  initStyleOption(&opt);
  return style()->sizeFromContents(QStyle::CT_ToolButton, &opt, QSize(0, 0),
                                   this);*/
  // return QToolButton::sizeHint();
  // return QSize(QAbstractButton::sizeHint().width(),
  //               QPushButton::sizeHint().height());
  // return QPushButton::sizeHint();
  // return QAbstractButton::sizeHint();
  // return QSize(1, 1);
}
