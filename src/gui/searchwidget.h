// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "guicommon.h"

#include "../core/tracks.h"

#include <condition_variable>
#include <mutex>
#include <thread>

class SearchWidget : public QDockWidget {

  struct SearchResultItem {
    std::string span_label, track_label, branch_name;
    std::shared_ptr<AnnotationSpan> span;
    double start = 0;
    double duration = 0;
  };

  struct SearchResults {
    bool query_empty = false;
    std::vector<SearchResultItem> items;
  };

  struct SearchSync {
    std::mutex _search_mutex;
    std::condition_variable _search_condition;
    std::string _search_query;
    volatile bool _sort_ascending = true;
    volatile size_t _sort_index = 0;
    volatile bool _search_exit = false;
    volatile bool _search_request = false;
    volatile bool _visible = false;
    std::shared_ptr<SearchResults> _previous_search_results;
  };
  std::shared_ptr<SearchSync> _sync = std::make_shared<SearchSync>();

  std::thread _search_thread;

  template <class T> static inline int compareValues(const T &a, const T &b) {
    if (a < b) {
      return -1;
    }
    if (b < a) {
      return +1;
    }
    return 0;
  }

  static void updateSearch(const std::shared_ptr<SearchSync> &sync);

public:
  SearchWidget();
  ~SearchWidget();
  virtual void showEvent(QShowEvent *event) override {
    QDockWidget::showEvent(event);
    _sync->_visible = true;
    updateSearch(_sync);
  }
  virtual void hideEvent(QHideEvent *event) override {
    QDockWidget::hideEvent(event);
    _sync->_visible = false;
  }
};
