#pragma once

#include <QWidget>

class QTreeWidget;
class QTreeWidgetItem;

// Replacement for CProjectWnd (ProjectWnd.h). Mirrors the page hierarchy
// from CMainFrame::pageConst[] (MainFrm.cpp): Connect and DAQ are top-level
// pages, CP Memory nests under CP. Selecting an item asks MainWindow to
// raise the matching page in the central QStackedWidget - the Qt analog of
// CMainFrame::OnSelectView()/GetProjectWnd().SelectPage().
class ProjectPane : public QWidget
{
    Q_OBJECT

public:
    explicit ProjectPane(QWidget *parent = nullptr);

signals:
    void pageSelected(const QString &pageId);

private:
    QTreeWidget *m_tree;
};
