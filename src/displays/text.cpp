// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "text.h"

#include "../core/workspace.h"
#include "../render/mesh.h"
#include "../render/renderlist.h"
#include "../render/texture.h"
#include "shapes.h"

#include <QFont>
#include <QPainter>
#include <QPixmap>
#include <QRawFont>
#include <QTextLayout>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <unordered_map>
#include <unordered_set>

class Glyph {
  cv::Mat _image;
  QPointF _origin;
  QRectF _rect;

public:
  const cv::Mat &image() const { return _image; }
  const QPointF &origin() const { return _origin; }
  const QRectF &rect() const { return _rect; }
  Glyph(QFont font, int font_size, uint32_t glyph) {

    int oversampling = 4;
    int margin = oversampling * 2;

    font.setPixelSize(font_size);

    QRawFont raw_font = QRawFont::fromFont(font);

    QPainterPath path = raw_font.pathForGlyph(glyph);

    QRect rect = path.boundingRect()
                     .marginsAdded(QMarginsF(margin, margin, margin, margin))
                     .toAlignedRect();

    _rect = rect;
    _origin = -rect.topLeft();

    rect = QRect(rect.x() * oversampling, rect.y() * oversampling,
                 rect.width() * oversampling, rect.height() * oversampling);

    path = QTransform().scale(oversampling, oversampling).map(path);

    path.translate(-rect.topLeft());

    QPixmap pixmap(rect.width(), rect.height());
    pixmap.fill(QColor(0, 0, 0, 255));
    {
      QPainter painter(&pixmap);
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setFont(font);
      painter.setPen(QPen(QBrush(Qt::white), 1));
      painter.setBrush(QBrush(Qt::white));
      painter.fillPath(path, QBrush(Qt::white));
    }

    QImage image = pixmap.toImage();
    cv::Mat mat(image.height(), image.width(), CV_8UC4,
                const_cast<uint8_t *>(image.bits()),
                static_cast<size_t>(image.bytesPerLine()));

    cv::cvtColor(mat, mat, cv::COLOR_RGB2GRAY);

    {
      cv::Mat mat2 = mat * 0.5;
      size_t n = oversampling * 2;
      cv::Mat m;
      for (size_t i = 1; i <= n; i++) {
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Point(i * 2 + 1, i * 2 + 1));
        cv::erode(mat, m, kernel);
        mat2 = cv::max(mat2, m * (0.5 + i * 0.5 / n));
        cv::dilate(mat, m, kernel);
        mat2 = cv::max(mat2, m * (0.5 - i * 0.5 / n));
      }
      mat = mat2;
    }

    cv::resize(mat, mat, cv::Size(), 1.0 / oversampling, 1.0 / oversampling,
               cv::INTER_AREA);

    _image = mat;
  }
  static std::shared_ptr<Glyph> instance(const QFont &font, int font_size,
                                         uint32_t glyph) {
    auto key = std::make_tuple(font, font_size, glyph);
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    static std::map<std::tuple<QFont, int, uint32_t>, std::shared_ptr<Glyph>>
        cache;
    if (cache.size() > 1000) {
      LOG_DEBUG("clearing glyph cache");
      cache.clear();
    }
    auto &v = cache[key];
    if (!v) {
      LOG_DEBUG("rendering glyph " << glyph);
      v = std::make_shared<Glyph>(font, font_size, glyph);
    }
    return v;
  }
};

class Font {
  std::shared_ptr<Texture> _texture;
  cv::Mat _image;
  std::vector<QRectF> _texture_rects;
  std::vector<QRectF> _mesh_rects;
  std::vector<QPointF> _origins;
  std::unordered_map<uint32_t, size_t> _glyph_indices;
  QFont _font;
  int _font_size = 0;
  static double pack(std::vector<QRect *> rr, int width) {
    // LOG_DEBUG("pack " << width);
    int row_start = 0;
    int y = 0;
    while (true) {
      int row_rects = 0;
      int row_width = 0;
      while (row_start + row_rects < rr.size()) {
        auto *r = rr[row_start + row_rects];
        if (row_width + r->width() < width) {
          row_width += r->width();
          row_rects++;
        } else {
          break;
        }
      }
      if (row_rects == 0) {
        break;
      }
      int y2 = y;
      int x = 0;
      for (int j = row_start; j < row_start + row_rects; j++) {
        auto *rect = rr[j];
        rect->translate(x - rect->x(), y - rect->y());
        x += rect->width();
        y2 = std::max(y2, y + rect->height());
      }
      y = y2;
      row_start += row_rects;
    }
    // LOG_DEBUG("cost " << y + width);
    return y + width;
  }
  static void pack(std::vector<QRect> &rects) {
    std::vector<QRect *> rr;
    for (auto &rect : rects) {
      rr.push_back(&rect);
    }
    std::sort(rr.begin(), rr.end(),
              [&](QRect *a, QRect *b) { return a->height() < b->height(); });
    int width = 1;
    for (auto &r : rects) {
      width = std::max(width, r.width() + 1);
    }
    for (int w = width; w < 1024 * 32; w *= 2) {
      if (pack(rr, w) < pack(rr, width)) {
        width = w;
      }
    }
    pack(rr, width);
  }

public:
  static std::shared_ptr<Font>
  instance(const QFont &font, int font_size,
           const std::unordered_set<uint32_t> &new_glyph_indices) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    static std::shared_ptr<Font> instance;
    auto merged_glyph_indices = new_glyph_indices;
    if (instance) {
      for (auto &p : instance->_glyph_indices) {
        merged_glyph_indices.insert(p.first);
      }
      if (merged_glyph_indices.size() > 200) {
        LOG_DEBUG("clearing glyph set");
        merged_glyph_indices = new_glyph_indices;
      }
    }
    if (instance) {
      if (instance->_font != font || instance->_font_size != font_size) {
        instance = nullptr;
      }
    }
    if (instance) {
      for (auto &i : new_glyph_indices) {
        if (instance->_glyph_indices.find(i) ==
            instance->_glyph_indices.end()) {
          instance = nullptr;
          break;
        }
      }
    }
    if (!instance) {
      LOG_DEBUG("building font texture for " << merged_glyph_indices.size()
                                             << " glyphs");
      instance = std::make_shared<Font>(font, font_size, merged_glyph_indices);
    } else {
      LOG_DEBUG("re-using font texture");
    }
    return instance;
  }

  Font(const QFont &font, int font_size,
       const std::unordered_set<uint32_t> &glyph_indices) {

    _font = font;
    _font_size = font_size;

    std::vector<std::shared_ptr<Glyph>> glyphs;
    for (auto &glyph_index : glyph_indices) {
      auto glyph = Glyph::instance(font, font_size, glyph_index);
      _glyph_indices[glyph_index] = glyphs.size();
      glyphs.emplace_back(glyph);
    }

    std::vector<QRect> pack_rects;
    {
      for (auto &glyph : glyphs) {
        pack_rects.emplace_back(0, 0, glyph->image().cols, glyph->image().rows);
      }
    }
    pack(pack_rects);

    int width = 0;
    int height = 0;
    for (auto &rect : pack_rects) {
      width = std::max(width, rect.right() + 1);
      height = std::max(height, rect.bottom() + 1);
    }
    LOG_DEBUG("font texture size " << width << " " << height);
    cv::Mat image(height, width, CV_8UC1);
    image.setTo(cv::Scalar(0));

    for (size_t i = 0; i < glyphs.size(); i++) {
      auto &rect = pack_rects[i];
      auto &glyph = glyphs[i];
      glyph->image().copyTo(
          image(cv::Rect(rect.x(), rect.y(), rect.width(), rect.height())));
      _mesh_rects.emplace_back(glyph->rect());
      _texture_rects.emplace_back(
          rect.x() * 1.0 / width, rect.y() * 1.0 / height,
          rect.width() * 1.0 / width, rect.height() * 1.0 / height);
      _origins.emplace_back(glyph->origin());
    }

    _image = image;
  }

  const std::shared_ptr<Texture> &texture() {
    if (!_texture) {
      _texture = std::make_shared<Texture>(TextureType::Linear);
      _texture->mipmap(false);
      _texture->update(_image);
    }
    return _texture;
  }

  const QRectF &textureRect(uint32_t glyph) const {
    auto iter = _glyph_indices.find(glyph);
    if (iter != _glyph_indices.end()) {
      return _texture_rects[iter->second];
    } else {
      throw std::runtime_error("glyph error");
    }
  }

  const QRectF &meshRect(uint32_t glyph) const {
    auto iter = _glyph_indices.find(glyph);
    if (iter != _glyph_indices.end()) {
      return _mesh_rects[iter->second];
    } else {
      throw std::runtime_error("glyph error");
    }
  }

  const QPointF &origin(uint32_t glyph) const {
    auto iter = _glyph_indices.find(glyph);
    if (iter != _glyph_indices.end()) {
      return _origins[iter->second];
    } else {
      throw std::runtime_error("glyph error");
    }
  }
};

TextRenderer::TextRenderer(const std::shared_ptr<Material> &material) {
  if (material) {
    _material = material;
  } else {
    ObjectScope ws;
    _material = std::make_shared<Material>();
  }
  _material_renderer = std::make_shared<MaterialRenderer>(_material);
}

void TextRenderer::renderSync(const RenderSyncContext &context) {
  _material_renderer->renderSync(context);
  _parent_pose = context.pose;
  SceneNode::renderSync(context);
}

void TextRenderer::renderAsync(const RenderAsyncContext &context) {

  _material_renderer->renderAsync(context);

  if (_watcher.changed(_view_facing, _text, _size, _offset)) {

    if (_text.empty()) {

      _mesh = nullptr;
      _texture = nullptr;

    } else {

      int font_size = 64;

      QFont font;
      font.setBold(true);
      font.setPixelSize(font_size);

      QTextLayout text_layout;
      text_layout.setFont(font);
      text_layout.setText(text().c_str());
      text_layout.beginLayout();
      while (true) {
        auto line = text_layout.createLine();
        if (line.isValid()) {
          line.setLineWidth(1000000000);
        } else {
          break;
        }
      }
      text_layout.endLayout();

      std::unordered_set<uint32_t> glyph_indices;
      for (auto &glyph_run : text_layout.glyphRuns()) {
        for (auto &glyph_index : glyph_run.glyphIndexes()) {
          glyph_indices.insert(glyph_index);
        }
      }

      auto texture_font = Font::instance(font, font_size, glyph_indices);

      _texture = texture_font->texture();

      MeshData mesh;

      for (auto &glyph_run : text_layout.glyphRuns()) {
        for (size_t i = 0; i < glyph_run.positions().size(); i++) {

          auto pos = glyph_run.positions()[i];
          auto index = glyph_run.glyphIndexes()[i];
          pos -= texture_font->origin(index);
          auto tex_rect = texture_font->textureRect(index);
          auto mesh_rect = texture_font->meshRect(index);
          auto bounds = glyph_run.rawFont().boundingRect(index);
          auto origin = texture_font->origin(index);

          mesh_rect = QRectF(pos.x(), mesh_rect.y(), mesh_rect.width(),
                             mesh_rect.height());

          mesh.indices.emplace_back(mesh.positions.size() + 0);
          mesh.indices.emplace_back(mesh.positions.size() + 2);
          mesh.indices.emplace_back(mesh.positions.size() + 1);
          mesh.indices.emplace_back(mesh.positions.size() + 0);
          mesh.indices.emplace_back(mesh.positions.size() + 3);
          mesh.indices.emplace_back(mesh.positions.size() + 2);

          mesh.texcoords.emplace_back(tex_rect.left(), 1.0 - tex_rect.bottom());
          mesh.texcoords.emplace_back(tex_rect.left(), 1.0 - tex_rect.top());
          mesh.texcoords.emplace_back(tex_rect.right(), 1.0 - tex_rect.top());
          mesh.texcoords.emplace_back(tex_rect.right(),
                                      1.0 - tex_rect.bottom());

          mesh.positions.emplace_back(mesh_rect.left(), -mesh_rect.bottom(), 0);
          mesh.positions.emplace_back(mesh_rect.left(), -mesh_rect.top(), 0);
          mesh.positions.emplace_back(mesh_rect.right(), -mesh_rect.top(), 0);
          mesh.positions.emplace_back(mesh_rect.right(), -mesh_rect.bottom(),
                                      0);
        }
      }

      mesh.scale(_size * 1.0 / font_size);

      if (mesh.positions.size()) {
        float xmin = mesh.positions.front().x();
        float xmax = mesh.positions.front().x();

        float ymin = mesh.positions.front().y();
        float ymax = mesh.positions.front().y();

        for (auto &p : mesh.positions) {
          xmin = std::min(xmin, p.x());
          ymin = std::min(ymin, p.y());

          xmax = std::max(xmax, p.x());
          ymax = std::max(ymax, p.y());
        }

        for (auto &p : mesh.positions) {
          p.x() -= (xmin + xmax) * 0.5;
        }
      }

      mesh.translate(Eigen::Vector3f(_offset.x(), _offset.y(), 0.0));

      if (_view_facing) {
        for (auto &p : mesh.positions) {
          mesh.extras.emplace_back(p.x(), p.y(), 3, 0);
          p = Eigen::Vector3f(0, 0, 0);
        }
      }

      _mesh = std::make_shared<Mesh>(mesh);
    }
  }

  if (_mesh) {

    MaterialBlock material = _material_renderer->block();
    material.color_texture = _texture->id();
    material.flags |= 1;
    material.flags |= 2;
    context.render_list->push(material);

    RenderOptions options;
    if (!_view_facing) {
      options.double_sided = true;
    }
    context.render_list->push(_mesh, options);

    InstanceBlock instance;
    instance.setPose(_parent_pose.matrix().cast<float>());

    context.render_list->push(instance);
  }

  SceneNode::renderAsync(context);
}

bool TextRenderer::pick(uint32_t id) const {
  return _material_renderer->id() == id;
}

TextDisplay::TextDisplay() {
  _renderer = node()->create<TextRenderer>(_material);
}

void TextDisplay::renderSync(const RenderSyncContext &context) {
  _material->color().r() = color().r() * brightness();
  _material->color().g() = color().g() * brightness();
  _material->color().b() = color().b() * brightness();
  _material->opacity() = opacity();
  _renderer->text(text());
  _renderer->offset(offset());
  _renderer->size(size());
  _renderer->viewFacing(viewFacing());
  MeshDisplayBase::renderSync(context);
}
