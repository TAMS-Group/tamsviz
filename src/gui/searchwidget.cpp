// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "searchwidget.h"

#include "../core/bagplayer.h"
#include "../core/tracks.h"
#include "../core/workspace.h"

#include "mainwindow.h"

SearchWidget::SearchWidget() : QDockWidget("Search") {

  auto sync = _sync;

  QWidget *content_widget = new QWidget();

  auto *main_layout = new QVBoxLayout(content_widget);
  main_layout->setSpacing(0);
  main_layout->setContentsMargins(0, 0, 0, 0);

  auto *search_box = new QLineEdit(content_widget);
  main_layout->addWidget(search_box);
  search_box->setPlaceholderText("Search annotations...");
  connect(search_box, &QLineEdit::textEdited, this,
          [sync](const QString &text) {
            std::unique_lock<std::mutex> lock(sync->_search_mutex);
            sync->_search_query = text.toStdString();
            sync->_search_request = true;
            sync->_search_condition.notify_all();
          });

  struct HeaderView : QHeaderView {
    HeaderView(Qt::Orientation orientation, QWidget *parent = nullptr)
        : QHeaderView(orientation, parent) {}
    virtual QSize sizeHint() const override {
      QSize size = QHeaderView::sizeHint();
      return size;
    }
    virtual int sizeHintForRow(int row) const override { return 0; }
    virtual bool hasHeightForWidth() const override { return true; }
    virtual int heightForWidth(int w) const override {
      return sizeHint().height();
    }
  };
  auto *header_view = new HeaderView(Qt::Horizontal, content_widget);
  QStringList _headers = {{
      "Bag",
      "Track",
      "Span",
      "Start",
  }};
  auto *header_model =
      new QStandardItemModel(1, _headers.size(), content_widget);
  header_model->setHorizontalHeaderLabels(_headers);
  header_view->setModel(header_model);
  header_view->setSectionResizeMode(QHeaderView::Stretch);
  header_view->setSortIndicatorShown(true);
  header_view->setSectionsClickable(true);
  header_view->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  header_view->setSizeIncrement(QSize(1, 1));
  header_view->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  header_view->setMinimumSize(0, 0);
  connect(header_view, &QHeaderView::sortIndicatorChanged, this,
          [this](int logicalIndex, Qt::SortOrder order) {
            _sync->_sort_index = logicalIndex;
            _sync->_sort_ascending = (order != Qt::AscendingOrder);
            updateSearch(_sync);
          });
  main_layout->addWidget(header_view);

  auto *result_list = new QTreeView(content_widget);
  main_layout->addWidget(result_list);
  result_list->setHeaderHidden(true);
  result_list->setIndentation(4);

  setWidget(content_widget);

  class SearchResultModel : public QAbstractTableModel {
    QWidget *_parent = nullptr;
    std::shared_ptr<const SearchResults> _search_results;

  public:
    SearchResultModel(QWidget *parent)
        : QAbstractTableModel(parent), _parent(parent) {}
    QVariant data(const QModelIndex &index, int role) const override {
      if (role == Qt::DisplayRole) {
        if (_search_results) {
          if (index.column() == 0 &&
              index.row() < _search_results->items.size()) {
            auto &item = _search_results->items[index.row()];
            std::string text = item.span_label +
                               (item.span_label.empty() ? "" : " - ") +
                               item.track_label + "\n    " + item.branch_name +
                               "\n    " + std::to_string(item.start) + " " +
                               std::to_string(item.duration);
            return QString::fromStdString(text);
          } else {
            return QVariant("No results found");
          }
        } else {
          return QVariant("Searching...");
        }
      }
      if (role == Qt::TextAlignmentRole) {
        if (!_search_results || _search_results->items.empty()) {
          return QVariant(Qt::AlignCenter);
        }
      }
      if (role == Qt::ForegroundRole) {
        if (!_search_results || _search_results->items.empty()) {
          return QVariant(
              _parent->palette().brush(QPalette::Disabled, QPalette::Text));
        }
      }
      return QVariant();
    }
    Qt::ItemFlags flags(const QModelIndex &index) const override {
      if (!_search_results || _search_results->items.empty()) {
        return Qt::ItemNeverHasChildren;
      } else {
        return Qt::ItemIsSelectable | Qt::ItemNeverHasChildren |
               Qt::ItemIsEnabled | Qt::ItemIsEditable;
      }
    }
    int rowCount(const QModelIndex &parent = QModelIndex()) const override {
      if (_search_results) {
        return std::max(size_t(1), _search_results->items.size());
      } else {
        return 1;
      }
    }
    int columnCount(const QModelIndex &parent = QModelIndex()) const override {
      return 1;
    }
    void setSearchResults(
        const std::shared_ptr<const SearchResults> &search_results) {
      beginResetModel();
      _search_results = search_results;
      endResetModel();
      LOG_DEBUG("search results changed");
      if (search_results) {
        LOG_DEBUG("item count " << search_results->items.size());
      }
    }
    std::shared_ptr<const SearchResults> searchResults() const {
      return _search_results;
    }
  };

  SearchResultModel *result_model = new SearchResultModel(result_list);
  result_list->setModel(result_model);

  class ItemDelegate : public QStyledItemDelegate {
    SearchResultModel *_model = nullptr;
    QObject *_parent = nullptr;

  public:
    ItemDelegate(SearchResultModel *model, QObject *parent)
        : QStyledItemDelegate(parent), _model(model), _parent(parent) {}
    virtual QWidget *createEditor(QWidget *parent,
                                  const QStyleOptionViewItem &option,
                                  const QModelIndex &index) const override {
      LOG_DEBUG("open row " << index.row());
      if (auto results = _model->searchResults()) {
        if (index.row() < results->items.size()) {
          LockScope ws;
          MainWindow::instance()->findAndOpenBag(
              results->items[index.row()].branch_name);
          if (ws->player->fileName() ==
              results->items[index.row()].branch_name) {
            ActionScope ws("Select");
            auto span = results->items[index.row()].span;
            ws->selection() = span;
            if (span) {
              ws->player->seek(span->start() + span->duration() * 0.5);
            }
          }
        }
      }
      return nullptr;
    }
    virtual void paint(QPainter *painter, const QStyleOptionViewItem &option,
                       const QModelIndex &index) const override {

      QStyleOptionViewItem option2 = option;
      initStyleOption(&option2, index);

      {
        QStyleOptionViewItem option3 = option2;
        option3.text = "";
        option3.state |= QStyle::State_Enabled;
        option3.state |= QStyle::State_Active;
        QStyle *style = QApplication::style();
        style->drawControl(QStyle::CE_ItemViewItem, &option3, painter, nullptr);
      }

      if (1) {
        painter->save();
        painter->setPen(QPen(option2.palette.brush(QPalette::Text), 0));
        painter->setFont(option2.font);
        painter->drawText(option2.rect, option2.displayAlignment, option2.text);
        painter->restore();
      }

      if (0) {
        painter->save();
        painter->setPen(QPen(option2.palette.brush(QPalette::Text), 0));
        painter->setFont(option2.font);

        QTextLayout text_layout(option2.text, option2.font, painter->device());
        text_layout.draw(painter,
                         QPoint(option2.rect.left(), option2.rect.top()));

        painter->restore();
      }
    }
  };
  result_list->setItemDelegate(new ItemDelegate(result_model, this));

  result_list->setEditTriggers(QAbstractItemView::DoubleClicked |
                               QAbstractItemView::EditKeyPressed);

  class SearchQuery {
    std::string _str;
    std::vector<std::string> _tokens;

  public:
    SearchQuery() {}
    SearchQuery(const std::string &query) : _str(query) {
      std::istringstream qstream(query);
      while (qstream) {
        std::string token;
        qstream >> token;
        if (!token.empty()) {
          _tokens.push_back(token);
        }
      }
    }
    const std::string &str() const { return _str; }
    bool match(const std::string &label) const {
      for (auto &token : _tokens) {
        if (std::search(label.begin(), label.end(), token.begin(), token.end(),
                        [](char a, char b) {
                          return std::tolower(a) == std::tolower(b);
                        }) == label.end()) {
          return false;
        }
      }
      return true;
    }
    bool empty() const { return _tokens.empty(); }
  };

  auto doSearch = [this, sync, result_model](const SearchQuery &query) {
    LOG_DEBUG("start search " << query.str());
    auto search_results = std::make_shared<SearchResults>();
    search_results->query_empty = query.empty();
    {
      size_t itrack = 0;
      size_t ibranch = 0;
      size_t ispan = 0;
      while (true) {
        if (sync->_search_exit || sync->_search_request) {
          LOG_DEBUG("search aborted " << query.str());
          return;
        }
        SearchResultItem search_result_item;
        {
          LockScope ws;
          auto &tracks = ws->document()->timeline()->tracks();
          if (itrack >= tracks.size()) {
            break;
          }
          if (auto track =
                  std::dynamic_pointer_cast<AnnotationTrack>(tracks[itrack])) {
            auto &branches = track->branches();
            if (ibranch >= branches.size()) {
              itrack++;
              ibranch = 0;
              continue;
            }
            auto &branch = branches[ibranch];
            auto &spans = branch->spans();
            if (ispan >= spans.size()) {
              ibranch++;
              ispan = 0;
              continue;
            }
            auto &span = spans[ispan];
            search_result_item.branch_name = branch->name();
            search_result_item.track_label = track->label();
            search_result_item.span_label = span->label();
            search_result_item.start = span->start();
            search_result_item.duration = span->duration();
            search_result_item.span = span;
          }
        }
        if (query.match(search_result_item.track_label) ||
            query.match(search_result_item.span_label)) {
          search_results->items.push_back(search_result_item);
        }
        ispan++;
      }
    }
    std::sort(search_results->items.begin(), search_results->items.end(),
              [sync](const SearchResultItem &a, const SearchResultItem &b) {
                std::array<int, 4> cmp = {{
                    compareValues(a.branch_name, b.branch_name),
                    compareValues(a.track_label, b.track_label),
                    compareValues(a.span_label, b.span_label),
                    compareValues(a.start, b.start),
                }};
                int sig = (sync->_sort_ascending ? -1 : +1);
                if (cmp[sync->_sort_index] != 0) {
                  return cmp[sync->_sort_index] == sig;
                }
                for (auto &v : cmp) {
                  if (v != 0) {
                    return v == sig;
                  }
                }
                return false;
              });
    bool changed = false;
    if (sync->_previous_search_results &&
        (sync->_previous_search_results->items.size() ==
         search_results->items.size())) {
      for (size_t i = 0; i < search_results->items.size(); i++) {
        if (sync->_previous_search_results->items[i].span !=
            search_results->items[i].span) {
          changed = true;
          break;
        }
      }
    } else {
      changed = true;
    }
    if (changed) {
      startOnMainThreadAsync([this, result_model, search_results]() {
        result_model->setSearchResults(search_results);
      });
    }
    sync->_previous_search_results = search_results;
    LOG_DEBUG("search finished " << query.str() << " "
                                 << search_results->items.size());
  };

  _search_thread = std::thread([this, sync, result_model, doSearch]() {
    while (true) {
      SearchQuery query;
      {
        std::unique_lock<std::mutex> lock(sync->_search_mutex);
        while (true) {
          if (sync->_search_exit) {
            return;
          }
          if (sync->_search_request) {
            sync->_search_request = false;
            query = SearchQuery(sync->_search_query);
            break;
          }
          sync->_search_condition.wait(lock);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (sync->_search_request || sync->_search_exit) {
        continue;
      }
      doSearch(query);
    }
  });

  updateSearch(sync);

  {
    LockScope()->modified.connect(sync, [sync]() { updateSearch(sync); });
  }
}

void SearchWidget::updateSearch(const std::shared_ptr<SearchSync> &sync) {
  std::unique_lock<std::mutex> lock(sync->_search_mutex);
  if (sync->_visible) {
    sync->_search_request = true;
    sync->_search_condition.notify_all();
  }
}

SearchWidget::~SearchWidget() {
  {
    std::unique_lock<std::mutex> lock(_sync->_search_mutex);
    _sync->_search_exit = true;
    _sync->_search_condition.notify_all();
  }
  _search_thread.join();
}
