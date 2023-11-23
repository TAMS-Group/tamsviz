// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "filewidget.h"

#include "mainwindow.h"

FileWidget::FileWidget() : QDockWidget("Files") {}

FileWidget::~FileWidget() {}

void FileWidget::showEvent(QShowEvent* event) {
  if (!widget()) {
    LOG_INFO("creating file browser");

    QWidget* content_widget = new QWidget();

    auto* main_layout = new QVBoxLayout(content_widget);
    main_layout->setSpacing(0);
    main_layout->setContentsMargins(0, 0, 0, 0);

    auto* search_box = new QLineEdit(content_widget);
    main_layout->addWidget(search_box);
    search_box->setPlaceholderText("Filter... (default: *.bag *.tmv)");

    auto root = QDir::currentPath();
    auto* tree_widget = new QTreeView(content_widget);
    main_layout->addWidget(tree_widget);
    auto* file_model = new QFileSystemModel(tree_widget);
    file_model->setResolveSymlinks(false);
    file_model->setRootPath(root);
    file_model->setNameFilterDisables(false);
    tree_widget->setModel(file_model);
    tree_widget->setRootIndex(file_model->index(root));
    tree_widget->setSortingEnabled(true);
    // tree_widget->header()->setStretchLastSection(false);
    // tree_widget->header()->setSectionResizeMode(QHeaderView::Fixed);
    // tree_widget->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    // tree_widget->header()->setSectionResizeMode(QHeaderView::Interactive);
    tree_widget->header()->setSectionHidden(1, true);
    tree_widget->header()->setSectionHidden(2, true);
    tree_widget->header()->resizeSection(0, 500);
    connect(tree_widget, &QTreeView::doubleClicked,
            [](const QModelIndex& index) {
              auto* file_model = (const QFileSystemModel*)index.model();
              auto file_path = file_model->filePath(index);
              LOG_INFO("open " << file_path.toStdString());
              MainWindow::instance()->openAny(file_path);
            });

    auto update_filters = [file_model](const QString& filter = "") {
      if (filter.isEmpty()) {
        file_model->setNameFilters({"*.bag", "*.tmv"});
      } else {
        file_model->setNameFilters({filter});
        // QString f = filter.split(QRegExp("\\s")).join("*");
        // file_model->setNameFilters({"*" + f + "*.bag", "*" + f + "*.tmv"});
      }
    };
    update_filters();

    connect(search_box, &QLineEdit::textEdited, this,
            [update_filters](const QString& text) {
              LOG_INFO("filter " << text.toStdString());
              update_filters(text);
            });

    setWidget(content_widget);
  }
  QDockWidget::showEvent(event);
}

void FileWidget::hideEvent(QHideEvent* event) { QDockWidget::hideEvent(event); }